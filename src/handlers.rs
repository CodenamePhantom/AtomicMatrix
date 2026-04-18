//! # Matrix High-Level API Handles
//!
//! This module encapsulates the matrix raw primitives into a more ergonomic API
//! that abstracts a lot of manual and repetitive work that has to be executed in
//! order to correctly interact with the matrix, as well as some safe pre-baked
//! functions that add more extensibility over what can be done generally.
//!
//! # Abstraction Layers
//!
//! ```text
//! [ internals -> Matrix Internal Frameworks ]   + iter, workers, tables, ...
//!     * builds on
//! [ MatrixHandler ]                             + typed blocks, lifecycle, sharing
//!     * escape hatch
//! [ AtomicMatrix ]                              + raw offsets, sizes, bytes
//!     *
//! [ /dev/shm ]                                  * physical shared memory
//! ```
//!
//! # Handler Scope
//!
//! The handler owns the SHM mapping and provides:
//! - Typed block allocation (`allocate<T>`) and deallocation (`free<T>`)
//! - Raw byte allocation for unknown types (`allocate_raw`)
//! - Zero-copy typed read and write on allocated blocks
//! - User-defined lifecycle state management (states 49+)
//! - Atomic state transitions with user-defined ordering
//! - Thread sharing via [`SharedHandler`]
//! - Escape hatches to the raw matrix and base pointer
//!
//! > Any high-level datasets and operators will be implemented in the **internals**
//! > folder.
//!
//! # Lifecycle States
//!
//! States 0–48 are reserved for internal matrix operations:
//! - `0` — `STATE_FREE`
//! - `1` — `STATE_ALLOCATED`
//! - `2` — `STATE_ACKED`
//! - `3` — `STATE_COALESCING`
//!
//! States 49 and above are available for user-defined lifecycles.
//! The matrix coalescing engine ignores any state beyond the ones described above —
//! a block in state 112 is never reclaimed automatically. Call `free()` explicitly
//! when done.
//!
//! **Note:** States 4–48 are reserved for future internal state management
//! implementations that have not been planned yet. Better safe than sorry.
//!
//! # Thread Sharing
//!
//! [`MatrixHandler`] owns the mmap and is not `Clone`. Use `share()` to produce a
//! [`SharedHandler`] that can be sent to other threads. The original handler must
//! outlive all shared handles derived from it.

use std::sync::atomic::Ordering;
use crate::matrix::core::{ AtomicMatrix, BlockHeader, RelativePtr };
use memmap2::MmapMut;

/// Minimum state value available for user-defined lifecycles.
/// States 0–48 are reserved for internal matrix and future framework use.
/// Currently only 0–3 are assigned — the remaining range (4–48) is reserved
/// for future internal lifecycle states without breaking user code.
pub const USER_STATE_MIN: u32 = 49;

/// Errors produced by [`MatrixHandler`] and [`SharedHandler`] operations.
#[derive(Debug, PartialEq)]
pub enum HandlerError {
    /// The allocator could not find a free block. Either OOM or contention.
    AllocationFailed(String),
    /// Caller attempted to set or transition to a reserved internal state (0–48).
    ReservedStatus(u32),
    /// Atomic state transition failed — block was not in the expected state.
    /// Contains the actual state found.
    TransitionFailed(u32),
    /// The block offset is outside the valid segment range.
    InvalidOffset(u32),
}

/// A typed handle to an allocated block in the matrix.
///
/// Since the matrix operates entirely on raw pointer addresses and internal
/// types, `Block<T>` is provided at the API level to wrap allocations into
/// a typed, ergonomic handle. The raw [`RelativePtr`] returned by the matrix
/// is reinterpreted as `T` and wrapped in `Block<T>` to maintain type
/// information at the surface layer. All pointer arithmetic is delegated to
/// the inner [`RelativePtr<T>`], referred to as **pointer**.
///
/// # Validity
///
/// A `Block<T>` is valid as long as:
/// - The originating [`MatrixHandler`] (and its mmap) is alive.
/// - The block has not been freed via `handler.free()`.
///
/// Blocks carry no lifetime parameter. The caller is responsible for not using
/// a block after freeing it or after the handler is dropped.
pub struct Block<T> {
    /// Payload offset from SHM base — points past the `BlockHeader`.
    pointer: RelativePtr<T>,
}

/// A lightweight reflection of the original handler that can be safely sent
/// across threads.
///
/// Produced by [`MatrixHandler::share()`]. Holds raw pointers into the SHM
/// segment. The originating [`MatrixHandler`] **must** outlive all
/// `SharedHandler` instances derived from it.
///
/// `SharedHandler` exposes the same allocation, I/O, lifecycle, and escape
/// hatch API as [`MatrixHandler`] via the [`HandlerFunctions`] trait —
/// it does not own the mmap.
pub struct SharedHandler {
    matrix_addr: usize,
    base_addr: usize,
    segment_size: u32,
    first_block_offset: u32,
}

/// The primary interface for interacting with an [`AtomicMatrix`].
///
/// Owns the SHM mapping. Cannot be cloned — use [`share()`] to produce a
/// [`SharedHandler`] for other threads.
///
/// See module documentation for the full abstraction layer diagram.
pub struct MatrixHandler {
    matrix: &'static mut AtomicMatrix,
    mmap: MmapMut,
    first_block_offset: u32,
}

impl<T> Block<T> {
    /// Constructs a `Block<T>` from a raw payload offset.
    ///
    /// The offset must point past the [`BlockHeader`] (i.e. `header_offset + 32`).
    /// Type `T` is introduced here — the matrix has no knowledge of it.
    pub(crate) fn from_offset(offset: u32) -> Self {
        Self { pointer: RelativePtr::new(offset) }
    }
}

impl MatrixHandler {
    /// Internal constructor. Called exclusively by [`AtomicMatrix::bootstrap`].
    pub(crate) fn new(
        matrix: &'static mut AtomicMatrix,
        mmap: MmapMut,
        first_block_offset: u32
    ) -> Self {
        Self { matrix, mmap, first_block_offset }
    }

    /// Produces a lightweight [`SharedHandler`] that can be sent to other threads.
    ///
    /// [`SharedHandler`] holds raw pointers into the SHM segment. This handler
    /// **must** outlive all shared handles derived from it — Rust cannot enforce
    /// this lifetime relationship automatically because `SharedHandler` uses raw
    /// pointers. Violating this contract is undefined behaviour.
    pub fn share(&self) -> SharedHandler {
        SharedHandler {
            matrix_addr: self.matrix as *const AtomicMatrix as usize,
            base_addr: self.base_ptr() as usize,
            segment_size: self.segment_size(),
            first_block_offset: self.first_block_offset,
        }
    }
}

impl HandlerFunctions for MatrixHandler {
    fn base_ptr(&self) -> *const u8 { self.mmap.as_ptr() }
    fn matrix(&self) -> &AtomicMatrix { self.matrix }
    fn first_block_offset(&self) -> u32 { self.first_block_offset }
    fn segment_size(&self) -> u32 { self.mmap.len() as u32 }
}

// Safety: AtomicMatrix uses only atomic operations internally.
// Caller guarantees the originating MatrixHandler outlives all SharedHandlers.
unsafe impl Send for SharedHandler {}
unsafe impl Sync for SharedHandler {}

impl HandlerFunctions for SharedHandler {
    fn base_ptr(&self) -> *const u8 { self.base_addr as *const u8 }
    fn matrix(&self) -> &AtomicMatrix {
        unsafe { &*(self.matrix_addr as *const AtomicMatrix) }
    }
    fn first_block_offset(&self) -> u32 { self.first_block_offset }
    fn segment_size(&self) -> u32 { self.segment_size }
}

/// Defines the core interaction surface for any matrix handle.
///
/// Implemented by both [`MatrixHandler`] and [`SharedHandler`]. All matrix
/// operations — allocation, I/O, lifecycle management, and escape hatches —
/// are provided through this trait so that framework code in `internals` can
/// operate generically over either handle type via `impl HandlerFunctions`.
///
/// # Implementing this trait
///
/// Implementors must provide four primitive accessors:
/// - [`base_ptr()`] — the SHM base pointer for this process's mapping
/// - [`matrix()`] — reference to the underlying [`AtomicMatrix`]
/// - [`first_block_offset()`] — offset of the first data block in the segment
/// - [`segment_size()`] — total segment size in bytes
///
/// All other methods have default implementations built on these four.
pub trait HandlerFunctions {
    /// Returns the SHM base pointer for this process's mapping.
    fn base_ptr(&self) -> *const u8;

    /// Returns a reference to the underlying [`AtomicMatrix`].
    fn matrix(&self) -> &AtomicMatrix;

    /// Returns the offset of the first data block in the segment.
    /// Used by `internals` iterators as the physical chain walk start point.
    fn first_block_offset(&self) -> u32;

    /// Returns the total segment size in bytes.
    fn segment_size(&self) -> u32;

    /// Allocates a block sized to hold `T`.
    ///
    /// Size is computed from `size_of::<T>()` and rounded up to the 16-byte
    /// minimum payload if necessary. The matrix remains typeless — type
    /// information exists only in the returned [`Block<T>`].
    ///
    /// # Errors
    /// Returns [`HandlerError::AllocationFailed`] if the matrix is out of
    /// memory or under contention after 512 retries.
    fn allocate<T>(&self) -> Result<Block<T>, HandlerError> {
        let size = (std::mem::size_of::<T>() as u32).max(16);
        self.matrix()
            .allocate(self.base_ptr(), size)
            .map(|ptr| Block::from_offset(ptr.offset()))
            .map_err(HandlerError::AllocationFailed)
    }

    /// Allocates a raw byte block of the given size.
    ///
    /// Returns a [`RelativePtr<u8>`] directly — use when the payload type is
    /// not known at allocation time, or when building `internals` framework
    /// primitives that operate on raw offsets. The caller is responsible for
    /// all casting and interpretation of the memory.
    ///
    /// # Errors
    /// Returns [`HandlerError::AllocationFailed`] if OOM or contention.
    fn allocate_raw(&self, size: u32) -> Result<RelativePtr<u8>, HandlerError> {
        self.matrix()
            .allocate(self.base_ptr(), size)
            .map_err(HandlerError::AllocationFailed)
    }

    /// Writes a value of type `T` into an allocated block.
    ///
    /// # Safety
    /// - `block` must be in `STATE_ALLOCATED`.
    /// - `block` must have been allocated with sufficient size to hold `T`.
    ///   This is guaranteed if the block was produced by [`allocate::<T>()`].
    /// - No other thread may be reading or writing this block concurrently.
    ///   The caller is responsible for all synchronization beyond the atomic
    ///   state transitions provided by [`set_state`] and [`transition_state`].
    unsafe fn write<T>(&self, block: &mut Block<T>, value: T) {
        unsafe { block.pointer.write(self.base_ptr(), value) }
    }

    /// Reads a shared reference to `T` from an allocated block.
    ///
    /// # Safety
    /// - `block` must be in `STATE_ALLOCATED`.
    /// - A value of type `T` must have been previously written via [`write`].
    /// - The returned reference is valid as long as the SHM mapping is alive
    ///   and the block has not been freed. It is **not** tied to the lifetime
    ///   of the [`Block<T>`] handle — the caller must ensure the block is not
    ///   freed while the reference is in use.
    /// - No other thread may be writing to this block concurrently.
    unsafe fn read<'a, T>(&self, block: &Block<T>) -> &'a T {
        unsafe { block.pointer.resolve(self.base_ptr()) }
    }

    /// Reads a mutable reference to `T` from an allocated block.
    ///
    /// # Safety
    /// - `block` must be in `STATE_ALLOCATED`.
    /// - A value of type `T` must have been previously written via [`write`].
    /// - The returned reference is valid as long as the SHM mapping is alive
    ///   and the block has not been freed. It is **not** tied to the lifetime
    ///   of the [`Block<T>`] handle — the caller must ensure the block is not
    ///   freed while the reference is in use.
    /// - No other thread may be reading or writing this block concurrently.
    ///   Two simultaneous `read_mut` calls on the same block is undefined behaviour.
    unsafe fn read_mut<'a, T>(&self, block: &Block<T>) -> &'a mut T {
        unsafe { block.pointer.resolve_mut(self.base_ptr()) }
    }

    /// Frees a typed block.
    ///
    /// Marks the block `STATE_ACKED` and immediately triggers coalescing.
    /// The block is invalid after this call — using it in any way is
    /// undefined behaviour.
    fn free<T>(&self, block: Block<T>) {
        let header_ptr = RelativePtr::<BlockHeader>::new(block.pointer.offset() - 32);
        self.matrix().ack(&header_ptr, self.base_ptr());
    }

    /// Frees a block by its header offset directly.
    ///
    /// Used by `internals` framework code that operates on raw offsets
    /// rather than typed [`Block<T>`] handles. `header_offset` must point
    /// to a valid [`BlockHeader`] within the segment.
    fn free_at(&self, header_offset: u32) {
        let header_ptr = RelativePtr::<BlockHeader>::new(header_offset);
        self.matrix().ack(&header_ptr, self.base_ptr());
    }

    /// Sets a user-defined lifecycle state on a block.
    ///
    /// The state must be >= [`USER_STATE_MIN`] (49). Attempting to set an
    /// internal state (0–48) returns [`HandlerError::ReservedStatus`].
    ///
    /// User states are invisible to the coalescing engine — a block in any
    /// user state will never be automatically reclaimed. Call [`free`]
    /// explicitly when the lifecycle is complete.
    ///
    /// # Errors
    /// Returns [`HandlerError::ReservedStatus`] if `state < USER_STATE_MIN`.
    fn set_state<T>(&self, block: &Block<T>, state: u32) -> Result<(), HandlerError> {
        if state < USER_STATE_MIN {
            return Err(HandlerError::ReservedStatus(state));
        }
        unsafe {
            block.pointer
                .resolve_header_mut(self.base_ptr())
                .state
                .store(state, Ordering::Release);
        }
        Ok(())
    }

    /// Returns the current state of a block.
    ///
    /// `order` controls the memory ordering of the atomic load. Use
    /// `Ordering::Acquire` for the general case. Use `Ordering::Relaxed`
    /// only if you do not need to synchronize with writes to the block's
    /// payload.
    fn get_state<T>(&self, block: &Block<T>, order: Ordering) -> u32 {
        unsafe {
            block.pointer
                .resolve_header(self.base_ptr())
                .state
                .load(order)
        }
    }

    /// Atomically transitions a block from one state to another.
    ///
    /// Succeeds only if the block is currently in `expected`. `next` must
    /// be >= [`USER_STATE_MIN`] — transitioning into an internal state is
    /// not permitted.
    ///
    /// `success_order` controls the memory ordering on success. Use
    /// `Ordering::AcqRel` for the general case. The failure ordering is
    /// always `Ordering::Relaxed`.
    ///
    /// Returns `Ok(expected)` on success — the value that was replaced.
    ///
    /// # Errors
    /// - [`HandlerError::ReservedStatus`] if `next < USER_STATE_MIN`.
    /// - [`HandlerError::TransitionFailed(actual)`] if the block was not
    ///   in `expected` — `actual` is the state that was observed instead.
    fn transition_state<T>(
        &self,
        block: &Block<T>,
        expected: u32,
        next: u32,
        success_order: Ordering
    ) -> Result<u32, HandlerError> {
        if next < USER_STATE_MIN {
            return Err(HandlerError::ReservedStatus(next));
        }
        unsafe {
            block.pointer
                .resolve_header_mut(self.base_ptr())
                .state
                .compare_exchange(expected, next, success_order, Ordering::Relaxed)
                .map_err(HandlerError::TransitionFailed)
        }
    }

    /// Returns a raw reference to the underlying [`AtomicMatrix`].
    ///
    /// For `internals` framework authors who need allocator primitives
    /// directly. Bypasses all handler abstractions — use with care.
    fn raw_matrix(&self) -> &AtomicMatrix {
        self.matrix()
    }

    /// Returns the raw SHM base pointer for this process's mapping.
    ///
    /// Use alongside [`raw_matrix()`] when building `internals` that need
    /// direct access to block memory beyond what the typed API provides.
    fn raw_base_ptr(&self) -> *const u8 {
        self.base_ptr()
    }
}