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

use crate::internals::error_collection::HandlerErrors;
use crate::prelude::*;
use memmap2::MmapMut;
use std::sync::atomic::Ordering;

/// Minimum state value available for user-defined lifecycles.
/// States 0–48 are reserved for internal matrix and future framework use.
/// Currently only 0–3 are assigned — the remaining range (4–48) is reserved
/// for future internal lifecycle states without breaking user code.
pub const USER_STATE_MIN: u32 = 49;

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
#[derive(Debug)]
pub struct Block<T> {
    /// Payload offset from SHM base — points past the `BlockHeader`.
    pub pointer: RelativePtr<T>,
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
///
/// **IF A DEDICATED MMAP IS NEEDED**, create a new MatrixHandler through
/// `AtomicMatrix::bootstrap()`.
#[derive(Clone, Copy)]
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
    id: String,
}

impl<T> Block<T> {
    /// Constructs a `Block<T>` from a raw payload offset.
    ///
    /// The offset must point past the [`BlockHeader`] (i.e. `header_offset + 32`).
    /// Type `T` is introduced here — the matrix has no knowledge of it.
    pub(crate) fn from_offset(offset: u32) -> Self {
        Self {
            pointer: RelativePtr::new(offset),
        }
    }
}

impl MatrixHandler {
    /// Internal constructor. Called exclusively by [`AtomicMatrix::bootstrap`].
    pub(crate) fn new(
        matrix: &'static mut AtomicMatrix,
        mmap: MmapMut,
        first_block_offset: u32,
        id: String,
    ) -> Self {
        Self {
            matrix,
            mmap,
            first_block_offset,
            id
        }
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

    /// Removes the SHM file from the system.
    ///
    /// Should be called before the application exits to prevent the SHM file
    /// from persisting in `/dev/shm/` across runs. This is an explicit, opt-
    /// in cleanup - if omitted, the file survives until the system reboots
    /// or it's manually removed.
    ///
    /// Existing mapping remains valid until the handlers are dropped; this
    /// only removes the filesystem entry, preventing new attachments.
    pub fn die(&self) -> Result<(), HandlerErrors> {
        let id = &self.id;
        let path = format!("/dev/shm/matrix-{}", id);

        match std::fs::remove_file(&path) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                return Err(HandlerErrors::DecomissionFailed { path, reason: e });
            }
        };

        Ok(())
    }
}

impl HandlerFunctions for MatrixHandler {
    fn base_ptr(&self) -> *const u8 {
        self.mmap.as_ptr()
    }
    fn matrix(&self) -> &AtomicMatrix {
        self.matrix
    }
    fn first_block_offset(&self) -> u32 {
        self.first_block_offset
    }
    fn segment_size(&self) -> u32 {
        self.mmap.len() as u32
    }
}

// Safety: AtomicMatrix uses only atomic operations internally.
// Caller guarantees the originating MatrixHandler outlives all SharedHandlers.
unsafe impl Send for SharedHandler {}
unsafe impl Sync for SharedHandler {}

impl HandlerFunctions for SharedHandler {
    fn base_ptr(&self) -> *const u8 {
        self.base_addr as *const u8
    }
    fn matrix(&self) -> &AtomicMatrix {
        unsafe { &*(self.matrix_addr as *const AtomicMatrix) }
    }
    fn first_block_offset(&self) -> u32 {
        self.first_block_offset
    }
    fn segment_size(&self) -> u32 {
        self.segment_size
    }
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
    fn allocate<T>(&self) -> Result<Block<T>, HandlerErrors> {
        let size = std::mem::size_of::<T>() as u32;
        match self.matrix()
            .allocate(self.base_ptr(), size)
            .map(|ptr| Block::from_offset(ptr.offset())) {
                Ok(v) => return Ok(v),
                Err(_) =>  return Err(HandlerErrors::AllocationFailed("Unable to allocate block".into())),
            };
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
    fn allocate_raw(&self, size: u32) -> Result<RelativePtr<u8>, HandlerErrors> {
        match self.matrix().allocate(self.base_ptr(), size) {
            Ok(v) => return Ok(v),
            Err(_) =>  return Err(HandlerErrors::AllocationFailed("Unable to allocate block".into())),
        };
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
        let header_ptr = RelativePtr::<BlockHeader>::new(block.pointer.offset() - HEADER_SPACE);
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
    fn set_state<T>(&self, block: &Block<T>, state: u32) -> Result<(), HandlerErrors> {
        if state < USER_STATE_MIN {
            return Err(HandlerErrors::ReservedState(state));
        }
        unsafe {
            block
                .pointer
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
            block
                .pointer
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
    /// - [`HandlerError::TransitionFailed`] if the block was not in the 
    ///   expected state.
    fn transition_state<T>(
        &self,
        block: &Block<T>,
        expected: u32,
        next: u32,
        success_order: Ordering,
    ) -> Result<u32, HandlerErrors> {
        if next < USER_STATE_MIN {
            return Err(HandlerErrors::ReservedState(next));
        }
        unsafe {
            block
                .pointer
                .resolve_header_mut(self.base_ptr())
                .state
                .compare_exchange(expected, next, success_order, Ordering::Relaxed)
                .map_err(HandlerErrors::TransitionFailed)
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

    /// Hashes a Matrix ID into a standard 16 bytes sized segment.
    ///
    /// ### Params:
    /// @id: The ID to be hashed.
    fn hash_id(&self, id: &str) -> [u8; 16] {
        let mut h1: u64 = 0x9368517673b203ef;
        let mut h2: u64 = 0x5851f42d4c957f2d;

        for &b in id.as_bytes() {
            h1 ^= b as u64;
            h1 = h1.wrapping_mul(0xff51afd7ed558ccd);
            h1 ^= h2.rotate_right(17); // cross-mix
            h2 ^= b as u64;
            h2 = h2.wrapping_mul(0xc4ceb9fe1a85ec53);
            h2 ^= h1.rotate_right(31); // cross-mix
        }

        h1 ^= id.len() as u64;
        h2 ^= id.len() as u64;
        h1 = h1.wrapping_add(h2);
        h2 = h2.wrapping_add(h1);

        let mut out = [0u8; 16];
        out[..8].copy_from_slice(&h1.to_le_bytes());
        out[8..].copy_from_slice(&h2.to_le_bytes());
        out
    }
}
