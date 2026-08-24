//! # Peer checked Atomic RingBuffer example.
//!
//! It creates a single ringbuffer and attaches 4 producers and 2 consumers on top of it. Each
//! producer is tasked with publishing a random u16 number from 200 to 2048, together with a thread
//! id and a sequence counter. 
//!
//! Consumers will then, dequeue the values from the ringbuffer and pass it to each other for peer 
//! validation. If an identical dequeue'd value is detected, the colision is registered in a global
//! duplicate counter that both consumers have access to.
//!
//! The stats of the run are logged at the end with:
//!
//! - Processed messages from each consumer.
//! - Run timer.
//! - Duplicate count.
//!
//! This code is merely a proof of concept of how the atomic matrix works and should be used as an
//! example.
//!
//! ## Break down
//!
//! ### Producers
//!
//! Producers are tasked with submitting a message to be then consumed on the ringbuffer. The
//! message contains the id of the thread that pushed this message (a simple u8 id), the number
//! generated at random, and the sequential increment of this message. All messages have a
//! sequential signature of Num of previous message + 1 to differentiate values from each other, as
//! the birthday paradox could apply on such a short range.
//!
//! In case the message fails because the ringbuffer is full, the producer will sleep for 100
//! millisecond before waking up and trying again. This retry happens indefinately until the
//! required number of messages are passed.
//!
//! ### Consumers
//!
//! Consumers receive a copy of the ringbuffer and two blocks of type [`Mesg`] at spawning (one 
//! block for itself, and the block of its peer), which will be used for the peer matching. The 
//! consumers can be broken down into two major tasks, dequeueing and peer matching.
//!
//! - Dequeueing
//!
//! At the beginning of each round, both consumers will dequeue a message from the ringbuffer and
//! register its value, be it [`Some`] or [`None`], to the thread owned block in the shape of a 
//! `Mesg` with status 0 (waiting). 
//!
//! - Peer matching
//!
//! Peer matching is an atomic handshake that requires both participants to agree on a value before
//! they can dequeue another one. After registering its own value, the thread enters a loop that
//! checks its own block and its peer's until a change in the atomic status is detected.
//!
//! In its own block, it checks for states 1 (pass), or 2 (fail). If state 1 is detected, it changes
//! the status of the Mesg to 3 (stale), and increases the messages counter. If it's state 2, it
//! also changes the status to 3, but also increases a global atomic counter (also in the form of a
//! block) that sinalizes a race condition (both consumers caught the same value).
//!
//! On the peer block, it will only react to state 0, when it matches the values of the Mesg against
//! its own dequeue values, changing the state to either 1 or 2 based on the results, and to state 4
//! (break even) and 5 (none), where it will change its own state to 1 and increase the counter.
//!
//! This behaviour happens until the thread breaks even from the ringbuffer dequeueing (all messages
//! from the producers have been passed). Both self and peer matching can only happen once, and both
//! are required to happen before another dequeue can be executed.
//!
//! - Break even
//!
//! When a consumer receives 20.000 Nones back from dequeueing, it publishes a Mesg with a pseudo 
//! nill value (all value fields are zero), sets its own state to 5, evaluates the peer block to 1
//! automatically, and increments a none_breach guard. If a consumer receives 10 sequential
//! none_breach increments, it will set its block state to 5 and break out of the dequeueing loop, 
//! considering all its operations done.
//!
//! If a successful dequeue happens after a none_guard has been triggered, the none_breach counter
//! is reseted.
//!
//! ### End
//!
//! All these metrics are then thrown into stdout for show

use atomic_matrix::{
    internals::collections::atomic_ringbuffer::{AtomicRingBuffer, Behaviour},
    prelude::*,
};
use std::{
    thread, 
    time::{Instant, Duration},
    sync::atomic::{AtomicU8, Ordering},
};
use rand::prelude::*;

/// const controllers for invariant values.
const LOOPS: u32 = 1_000_000;
const SENDER_NUM: u32 = 4;
const RB_SIZE: usize = 8192;

/// Message struct used for the peer-matching protocol. Status is the atomic handshake checkpoint.
struct Mesg {
    thread_id: u8,
    num: u16,
    seq: u32,
    status: AtomicU8,
}

/// Sender procedure.
///
/// It uses its own copy of the ringbuffer to push a message containing the current sender id, the
/// randomly generated number, and the incremented sequence of the current thread.
///
/// ### Params
/// @rb: The atomic ringbuffer instance that holds a tuple of u8, u16, and u32. \
/// @id: The id of the current sender.
fn sender(rb: &AtomicRingBuffer<(u8, u16, u32), RB_SIZE>, id: u8) {
    let mut count: u32 = 0;
    let mut rng = rand::rng();
    let mut seq: u32 = 0;

    while count < LOOPS {
        let num = rng.random_range(200..2048);
        match rb.enqueue((id, num, seq)) {
            Ok(_) => {
                count = count.checked_add(1).unwrap();
                seq = seq.checked_add(1).unwrap();
                continue;
            },
            Err(_) => {
                thread::sleep(Duration::from_millis(100));
                continue;
            }
        };

    }

    println!("Sent: {} messages", count);
}

/// Receiver procedure
///
/// It executes the dequeue on the ringbuffer, and the two way peer-matching protocol.
///
/// ### Params
/// @rb: The atomic ringbuffer instance that holds a tuple of u8, u16, and u32. \
/// @recv_block: The current thread owned block for peer-matching. \
/// @peer_block: The concurrent receiver owned block for peer-matching. \
/// @race_counter: The global race counter to be incremented when a duplicate is caught. \
/// @s_handler: A SharedHandler instance for AtomicMatrix operations. \
/// @recv_id: An identifier for the current consumer. Used for log purposes.
fn receiver(
    rb: &AtomicRingBuffer<(u8, u16, u32), RB_SIZE>, 
    recv_block: &Block<Mesg>, 
    peer_block: &Block<Mesg>,
    race_counter: &Block<u32>,
    s_handler: SharedHandler,
    recv_id: u8,
) {
    // Quick author note:
    // Look mom, no mutexes!

    let mut none_guard: u32 = 0;
    let mut none_breach: u32 = 0;
    let mut count: u32 = 0;

    let mut my_ref = s_handler.read_mut(recv_block).unwrap();
    let peer_ref = s_handler.read(peer_block).unwrap();

    // We first initialize our message struct as Stale.
    *my_ref = Mesg {
        thread_id: 0,
        num: 0,
        seq: 0,
        status: AtomicU8::new(3),
    };

    loop {
        match rb.dequeue() {
            Some((id, num, seq)) => {
                // Reset the none_breach flag for successful dequeues
                none_breach = 0;
                let mut ops_count = 0;

                // We register a new peer-match request with status Waiting.
                my_ref.thread_id = id;
                my_ref.num = num;
                my_ref.seq = seq;
                my_ref.status.store(0, Ordering::Release);

                let mut matched = false;
                let mut peer_matched = false;
                
                // Peer-matching protocol. It repeats until both match ops have been completed.
                loop {
                    // First, we check if our peer evaluated us. This happens only once, when the
                    // peer submitted the answer.
                    if !matched {
                        match my_ref.status.load(Ordering::Acquire) {
                            1 => {
                                my_ref.status.store(3, Ordering::Release);
                                matched = true;
                                ops_count += 1;
                            },
                            2 => {
                                my_ref.status.store(3, Ordering::Release);
                                s_handler.inline_mut(race_counter, |mut count| {
                                    *count += 1;
                                }).unwrap();
                                matched = true;
                                ops_count += 1;
                            },
                            _ => {}
                        }
                    }

                    // Then we evaluate our peer. This also happens only once.
                    if !peer_matched {
                        match peer_ref.status.load(Ordering::Acquire) {
                            0 => {
                                if peer_ref.thread_id == id && peer_ref.num == num && peer_ref.seq == seq {
                                    peer_ref.status.store(2, Ordering::Release);
                                } else {
                                    peer_ref.status.store(1, Ordering::Release);
                                }

                                peer_matched = true;
                                ops_count += 1;
                            },
                            4..5 => {
                                my_ref.status.store(1, Ordering::Release);
                                peer_matched = true;
                                ops_count += 1;
                            },
                            _ => {},
                        }
                    }

                    // Break if both ops are completed; repeat.
                    if ops_count >= 2 {
                        break;
                    }
                }

                count = count.checked_add(1).unwrap();
            }
            None => {
                none_guard = none_guard.checked_add(1).unwrap();

                if none_breach > 10 {
                    // We got a none_breach! We pass our peer, mark our block as done and leave.
                    if peer_ref.status.load(Ordering::Acquire) == 0 {
                        peer_ref.status.store(1, Ordering::Release);
                    };
                    my_ref.status.store(4, Ordering::Release);
                    break;
                } else if none_guard > 20_000 {
                    // We triggered the none_guard. We pass our peer, register a nill dequeue, and
                    // sleep for 300 milliseconds.
                    if peer_ref.status.load(Ordering::Acquire) == 0 {
                        peer_ref.status.store(1, Ordering::Release);
                    };
                    my_ref.thread_id = 0;
                    my_ref.num = 0;
                    my_ref.seq = 0;
                    my_ref.status.store(5, Ordering::Release);
                    println!("None guard!");
                    println!("Received: {} | Duplicates: {} | id: {recv_id}", count, *s_handler.read(race_counter).unwrap());
                    thread::sleep(Duration::from_millis(300));
                    none_breach += 1;
                    none_guard = 0;
                } 
            }
        };
    }

    println!("Received {count} messages on recv {recv_id}");
}

/// This part is way to self explanatory for me to break it down lol.
fn main() {
    let handler = AtomicMatrix::bootstrap(None, memory_scale::custom::mb::<5>()).unwrap();
    let ring_buffer = AtomicRingBuffer::<(u8, u16, u32), RB_SIZE>::new(handler.share(), Behaviour::Wait(300)).unwrap();

    let recv_a_block = handler.allocate::<Mesg>().unwrap();
    let recv_b_block = handler.allocate::<Mesg>().unwrap();
    let global_race_counter = handler.allocate::<u32>().unwrap();

    let instant = Instant::now();

    thread::scope(|s| {
        let mut sender_count = 0;

        while sender_count < SENDER_NUM {
            let id = sender_count + 1;
            s.spawn(move || sender(ring_buffer, id as u8));
            sender_count += 1;
        }
        s.spawn(|| receiver(ring_buffer, &recv_a_block, &recv_b_block, &global_race_counter, handler.share(), 1));
        s.spawn(|| receiver(ring_buffer, &recv_b_block, &recv_a_block, &global_race_counter, handler.share(), 2));
    });

    let end = instant.elapsed();

    println!("Done in {} us", end.as_micros());
    println!("Caught {} duplicates", *handler.read(&global_race_counter).unwrap());

    unsafe { handler.die().unwrap() };
}
