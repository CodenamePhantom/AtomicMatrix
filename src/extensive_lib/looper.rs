//! # Looper
//!
//! #### Iterator implementation for the matrix
//!
//! Looper is a iterator library that implements the Iterator trait from
//! Rust into AtomicMatrix. It integrates the trait logic into the chaining
//! system already present into the matrix to enable sequential traversal.
//!
//! Following the logic of [`MatrixHandler`] This method provides access to
//! typed data visualization directly into [`LoopWindow`] through the method
//! `.view_data_as::<T>()`, which returns a life time specified reference T.
//! As well as providing the ability to read data as raw bytes.
//!
//! # Thread Sharing
//!
//! [`LoopWindow`] is [`Send`] and [`Sync`] - once obtained, it can be freely
//! shared across threads, as it only holds references. Note however that race
//! conditions on concurrent read/write into the underlying block are still
//! possible and must be handled by the caller.
//!
//! [`Looper`] itself is not thread-safe (it holds mutable iteration state).
//! To iterate from multiple threads, create one iterator per thread or collect
//! data into a [`vec`] first.
use std::sync::atomic::Ordering;

use crate::prelude::*;

/// The Iterator object implementation for the matrix.
///
/// It encapsules a reference to the [`MatrixHandler`] API and implements the
/// std Iterator trait native from rust, with a few logic adaptations to run
/// inside the matrix life-cycle.
pub struct Looper {
    handler_ref: SharedHandler,
    current_offset: u32,
    end_offset: u32,
}

/// The view object that is returned as Iterate during operations.
///
/// It provides operation abstractions on the [`RelativePtr`] and the
/// [`MatrixHandler`] objects related to the context of iterating over the
/// matrix.
pub struct LoopWindow {
    rel_ptr: RelativePtr<u8>,
    handler: SharedHandler,
}

/// Default Iterator implementation on [`Looper`], with the addition
/// of using the matrix chain as next instead of normal indexes.
impl<'a> Iterator for Looper {
    type Item = LoopWindow;

    /// Queries the next block in the matrix based on the current item offset
    /// related to base_ptr() + the size of the block. If the size is 0 or the
    /// offset surpasses the boundaries of the matrix, the iteration stop.
    /// Else, returns an [`LoopWindow`] of the current record.
    ///
    /// # Returns
    /// Either the [`LoopWindow`] of the current record, or None.
    fn next(&mut self) -> Option<Self::Item> {
        if self.current_offset >= self.end_offset {
            return None;
        }

        let rel_ptr = unsafe { self.handler_ref.matrix().query(self.current_offset) };
        let header = unsafe { rel_ptr.resolve_header(self.handler_ref.base_ptr()) };

        let size = header.size.load(Ordering::Acquire);
        if size == 0 {
            return None;
        }

        self.current_offset = self
            .current_offset
            .checked_add(size)
            .expect("offset overflow");

        Some(LoopWindow::new(rel_ptr, self.handler_ref))
    }
}

impl Looper {
    /// Creates a new [`Looper`] instance over the current handler.
    ///
    /// # Returns:
    /// An instance of Self.
    pub fn new(handler_ref: SharedHandler) -> Self {
        let len = (16 + (std::mem::size_of::<AtomicMatrix>() as u32) + 15) & !15;
        let end = handler_ref.matrix().sector_boundaries.load(Ordering::Acquire);

        Self {
            handler_ref,
            current_offset: len,
            end_offset: end,
        }
    }
}

// SAFETY: LoopWindow contains only one RelativePtr<u8> (an offset u32,
// essentially imutable by design) and an imutable reference &MatrixHandler.
// The resolution to real pointer always goes through the handler, who
// manages its own thread safety. No mutable state is contained in the
// Struct.
unsafe impl Send for LoopWindow {}
unsafe impl Sync for LoopWindow {}

impl<'a> LoopWindow {
    /// Creates a new [`LoopWindow`] instance of a record.
    ///
    /// # Params:
    /// @rel_ptr: The [`RelativePtr`] of the current record. \
    /// @handler: A reference to the handler owned by the caller.
    ///
    /// # Returns:
    /// An instance of Self.
    pub fn new(rel_ptr: RelativePtr<u8>, handler: SharedHandler) -> Self {
        Self { rel_ptr, handler }
    }

    /// Returns a reference to the current record data typed as T.
    pub fn view_data_as<T>(&self) -> &'a T {
        let block = Block::<T>::from_offset(self.rel_ptr.offset());
        let res = unsafe { self.handler.read(&block).unwrap() };

        res
    }

    /// Returns a reference to the current record data as a raw pointer.
    pub fn view_data_raw(&self) -> &'a [u8] {
        unsafe {
            let data_ptr = self.rel_ptr.resolve(self.handler.base_ptr());
            let header = self.view_header();
            let size = header.size.load(Ordering::Acquire) - 32 - 16;
            std::slice::from_raw_parts(data_ptr, size as usize)
        }
    }

    /// Returns a reference to the header of this record
    pub fn view_header(&self) -> &'a BlockHeader {
        unsafe { self.rel_ptr.resolve_header(self.handler.base_ptr()) }
    }

    /// Returns the offset of the current record
    pub fn view_offset(&self) -> u32 {
        self.rel_ptr.offset()
    }
}
