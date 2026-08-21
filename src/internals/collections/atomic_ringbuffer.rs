//! The AtomicRingBuffer works as a thread safe, circular message passing queue
//! where multiple processes can hang as Producers and Consumers at the same time.
//!
//! Being able to atomically move the tail without crashing any other process that
//! is trying to dequeue messages at the same time, while producers can atomically
//! publish new data at the head, and any concurrent thread will simply receive the
//! next available offset to be used, the struct grants a multi-process dynamic
//! RingBuffer that can be independently loaded into each subscribing thread.
//!
//! # Architecture.
//!
//! The ringbuffer is composed of a single dedicated block inside the matrix.
//!
//! [Matrix Block]
//!     |
//!     |--> [AtomicRingBuffer (Metadata)]
//!     |
//!     |--> [Offset 1]
//!     |--> [Offset 2]
//!     ... up to N offsets, user defined.
//!
//! Allocation contests for the next offset happens in a stage queue to avoid
//! publishing an offset that contains no message, as this could SEGFAULT any read
//! attempts on the contested head because it was published to reserve it from other
//! producers but no data was written yet.
//!
//! Dequeuing is also strainght forward, as moving the tail forward is also a guarded
//! operation.
//!
//! # Zero Copy and Safety
//!
//! The struct follows the core implementation of the matrix as a full zero copy data
//! structure that can safely be shared across threads and processes through atomic
//! coordination. No need for Arc or Box wrappers, nor complex hazard pointers logic.
//! Simply clone the struct to other threads, or construct it from an independent
//! process and operate within the declared utilization protocol of the system.
//!
//! This means that, although you can pass and construct the same ringbuffer all over,
//! following the correct system utilization is up to you.

use crate::internals::error_collection::BufferErrors;
use crate::prelude::*;
use std::{
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    cell::UnsafeCell,
};

/// The header state for the [`AtomicRingBuffer`]. It isolates the block
/// from any other context present in the matrix.
///
/// # Note
///
/// All states should be declared with an unique number so logics don't
/// overlap.
pub const STATE_RINGBUFFER: u32 = 10_000;
pub const WAIT_RETRY_CAP: u16 = 256;
pub const CAS_RETRIES_CAP: u16 = 256;

/// Holds the behaviour state for when the ring buffer fills up.
pub enum Behaviour {
    /// Drops the current package.
    Drop,
    /// Overwrites the next entry and moves the tail and head ahead.
    Overwrite,
    /// Wait for an amount of time specified by the user before
    /// retrying.
    Wait(u32),
}

/// The coordination structure for the ring buffer.
///
/// It holds all the metadata related to the ring buffer registered in
/// the block, as well as High-Level functions to safely read/write data
/// in the buffer itself.
pub struct AtomicRingBuffer<T, const N: usize> {
    behaviour: Behaviour,
    start: u32,
    end: u32,
    reserved_head: AtomicU32,
    commited_head: AtomicU32,
    slots: [UnsafeCell<T>; N],
    tail: AtomicU32,
    has_message: AtomicBool,
    address: u32,
}

// Safety: as the buffer inherits the zero-copy, lock-free traits from
// the matrix by default, and not allowing direct manipulation through a
// mut reference, it is safe to be shared across different threads with-
// out causing race conditions (contentions are still implied).
unsafe impl<T, const N: usize> Send for AtomicRingBuffer<T, N> {}
unsafe impl<T, const N: usize> Sync for AtomicRingBuffer<T, N> {}

impl<'a, T: Default + Copy, const N: usize> AtomicRingBuffer<T, N> {
    /// Internal helper function to move the current pointer forward in
    /// the buffer.
    ///
    /// It adds the size of the message slot to the provided offset and
    /// returns the new pointer, either to the next slot forward, or to
    /// the beginning of the buffer if it has reached the end.
    ///
    /// # Params
    /// @offset: The current offset to be moved forward.
    ///
    /// # Returns
    /// A pointer value to the next slot to be used.
    #[inline]
    fn advance(&self, slot: u32) -> u32 {
        let next = slot + 1;
        if next >= self.end { self.start } else { next }
    }

    /// Returns a new [`AtomicRingBuffer`] reference.
    ///
    /// It encapsulates the handler to facilitate matrix manipulation and
    /// configures itself with the provided slot quantity and behaviour.
    /// Then it allocates a new buffer including:
    ///
    /// AtomicRingBuffer size + Slot Size * Slot Quantity
    ///
    /// And writes the buffer at the beginning of this block. A reference
    /// to the struct in the matrix is then returned to be used.
    ///
    /// # Params
    /// @slots: The quantity of slots required. \
    /// @handler_ref: A reference to the matrix handler. \
    /// @behaviour: The full buffer behaviour.
    ///
    /// # Returns
    /// A reference to the written buffer in the matrix.
    ///
    /// # Panic!
    /// The code will panic if it fails to allocate a block for the buffer.
    pub fn new(
        handler_ref: SharedHandler,
        behaviour: Behaviour,
    ) -> Option<&'a Self> {
        let block = handler_ref.allocate::<AtomicRingBuffer<T, N>>().ok()?;

        let slots: [UnsafeCell<T>; N] = std::array::from_fn(|_| UnsafeCell::new(T::default()));

        handler_ref.set_state(&block, STATE_RINGBUFFER).ok()?;

        if handler_ref.inline_mut(&block, |mut rb| {
            *rb = Self {
                behaviour,
                start: 0,
                end: slots.len() as u32 - 1,
                reserved_head: AtomicU32::new(0),
                commited_head: AtomicU32::new(0),
                slots,
                tail: AtomicU32::new(0),
                has_message: AtomicBool::new(false),
                address: block.header_offset(),
            }
        }).is_none() {
            return None;
        };

        let rb_guard = handler_ref.read(&block).ok()?;
        let rb: &'a Self = unsafe { &*(&*rb_guard as *const Self) };

        Some(rb)
    }

    /// Adds a new message to the [`AtomicRingBuffer`] and updates the
    /// current head. If the buffer is deemed full (next_head == current_tail),
    /// it applies the logic related to the behaviour attached at creation.
    ///
    /// It first loads the current head and tail and checks if the allocation
    /// clashes with the current unread messages:
    ///
    /// - **Drop**: Drops the package without registering it to the ring
    /// buffer and move along. Completelly non-blocking.
    /// - **Wait(N)**: Hint loop for the CPU *N* times before retrying the
    /// allocation, in hope it will have space next time.
    /// - **Overwrite**: Not implemented yet. Figuring out how to make it
    /// work under concurrency.
    ///
    /// Then it tries to perform a weak CAS the current head with the next. If
    /// successful, write the message and commit it in the read queue. If not,
    /// restart the loop from the beginning until a allocation is successful, or
    /// 300 retries are reached.
    ///
    /// # Params
    /// @item: The value to be queued in the buffer.
    ///
    /// # Errors
    /// @DropBehaviour: Happens when drop is stablished as behaviour, expected. \
    /// @BufferFull: The allocation exceeded the stablished ammount of wait
    /// iterations \
    /// @TooManyProducers: There are too many producers attached to this buffer
    /// and its not possible to allocate a slot before the buffer fills up.
    pub fn enqueue(&self, item: T) -> Result<(), BufferErrors> {
        let mut cas_retries: u16 = 0;
        let mut wait_iterations: u16 = 0;

        let slot: u32 = loop {
            let current_head = self.reserved_head.load(Ordering::Relaxed);
            let next_head = self.advance(current_head);
            let current_tail = self.tail.load(Ordering::Acquire);

            if next_head == current_tail {
                match self.behaviour {
                    Behaviour::Drop => return Err(BufferErrors::DropBehaviour),
                    Behaviour::Wait(i) => {
                        for _ in 0..i {
                            std::hint::spin_loop();
                        }
                        wait_iterations += 1;
                        if wait_iterations > WAIT_RETRY_CAP {
                            return Err(BufferErrors::BufferFull);
                        }
                        continue;
                    }
                    Behaviour::Overwrite => {
                        unimplemented!()
                    }
                }
            }

            match self.reserved_head.compare_exchange_weak(
                current_head,
                next_head,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break current_head,
                Err(_) => {
                    std::hint::spin_loop();
                    cas_retries += 1;
                    if cas_retries > CAS_RETRIES_CAP {
                        return Err(BufferErrors::TooManyProducers);
                    }
                    continue;
                }
            }
        };

        unsafe {
            let slot_ptr = self.slots[slot as usize].get();
            std::ptr::write(slot_ptr, item) 
        };

        while self.commited_head.load(Ordering::Acquire) != slot {
            std::hint::spin_loop();
        }

        let next = self.advance(slot);
        self.commited_head.store(next, Ordering::Release);
        self.has_message.store(true, Ordering::Release);

        Ok(())
    }

    /// Returns the current tail from the buffer and moves the pointer
    /// forward.
    ///
    /// First it checks the current tail against the commited head to validate
    /// the presence of new messages. If the values
    ///
    /// # Returns
    /// Either a life time specified reference to the block data, or
    /// None if the block is empty
    pub fn dequeue(&self) -> Option<T> {
        let current_tail = self.tail.load(Ordering::Relaxed);
        let next_tail = self.advance(current_tail);

        if current_tail == self.commited_head.load(Ordering::Acquire) {
            return None;
        }

        let data = unsafe { self.slots[current_tail as usize].get().read() };

        self.tail.store(next_tail, Ordering::Release);

        if next_tail == self.commited_head.load(Ordering::Acquire) {
            self.has_message.store(false, Ordering::Release);
        }

        return Some(data)
    }

    /// Returns the current head from the buffer without moving the pointer
    /// forward, making the message remain valid.
    ///
    /// It does the same process as `dequeue()`, but it removes the step
    /// of moving the pointers and releasing the `has_message` flag.
    ///
    /// # Returns
    /// Either a reference to the block data, or None if the block is
    /// empty
    pub fn peek(&self) -> Option<T> {
        let current_tail = self.tail.load(Ordering::Acquire);

        if current_tail == self.commited_head.load(Ordering::Acquire) {
            return None;
        }

        let data = unsafe { self.slots[current_tail as usize].get().read() };

        Some(data)
    }

    /// Returns the current status of the has_message flag
    pub fn has_message(&self) -> bool {
        self.has_message.load(Ordering::Acquire)
    }

    /// Returns the current head available to write
    pub fn current_head(&self) -> u32 {
        self.commited_head.load(Ordering::Relaxed)
    }

    /// Returns the current tail available to read.
    pub fn current_tail(&self) -> u32 {
        self.tail.load(Ordering::Relaxed)
    }

    pub fn address(&self) -> u32 {
        self.address
    }
}
