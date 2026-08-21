use atomic_matrix::{
    internals::collections::atomic_ringbuffer::{AtomicRingBuffer, Behaviour},
    prelude::*,
};
use std::{thread, time::Instant};

fn sender(loops: u32, rb: &AtomicRingBuffer<u16, 1024>) {
    let mut count: u32 = 0;
    for _ in 0..loops {
        rb.enqueue(276).unwrap();
        count = count.checked_add(1).unwrap();
    }

    println!("Sent: {} messages", count);
}

fn receiver(loops: u32, rb: &AtomicRingBuffer<u16, 1024>) {
    let mut count: u32 = 0;
    while count < loops * 3 {
        let _ = match rb.dequeue() {
            Some(_) => {
                count = count.checked_add(1).unwrap();
            }
            None => {}
        };
    }

    println!("Received: {} messages", count);
}

fn main() {
    const LOOPS: u32 = 1_000_000;

    let handler = AtomicMatrix::bootstrap(None, 5 * 1024 * 1024).unwrap();
    let ring_buffer = AtomicRingBuffer::<u16, 1024>::new(handler.share(), Behaviour::Wait(300)).unwrap();

    let instant = Instant::now();

    thread::scope(|s| {
        s.spawn(|| sender(LOOPS, ring_buffer));
        s.spawn(|| sender(LOOPS, ring_buffer));
        s.spawn(|| sender(LOOPS, ring_buffer));
        s.spawn(|| receiver(LOOPS, ring_buffer));
    });

    let end = instant.elapsed();

    println!("Done in {} us", end.as_micros());

    unsafe { handler.die().unwrap() };
}
