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
//! [ extensive_lib -> Matrix Internal Frameworks]  * fencer, guardian, looper ...
//!     * and
//! [ internals -> Matrix Internal Collections ]    + atomic data structs, errors ...
//!     * builds on
//! [ MatrixHandler ]                               + typed blocks, lifecycle, sharing
//!     * escape hatch
//! [ AtomicMatrix ]                                + raw offsets, sizes, bytes
//!     * physical bound layer
//! [ /dev/shm ]                                    * physical shared memory
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
//! Safe handler functions also enforce typing at runtime through Type Tagging ( check
//! the [`type_tag`] helper module), returning a [`HandlerErrors::TypeMismatchError`]
//! when the check fails.
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
pub const TAG_SIZE: u32 = std::mem::size_of::<u32>() as u32;

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
    base_ref: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlockMetadata {
    pub type_tag: u32,
    pub offset: u32,
    pub len: u32,
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
pub struct SharedHandler<'a> {
    matrix_addr: usize,
    base_addr: usize,
    segment_size: u32,
    first_block_offset: u32,
    blocks: &'a [BlockMetadata],
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
    blocks: Vec<BlockMetadata>,
}

impl<T> Block<T> {
    /// Constructs a `Block<T>` from a raw payload offset.
    ///
    /// The offset must point past the [`BlockHeader`] (i.e. `header_offset + 32`).
    /// Type `T` is introduced here — the matrix has no knowledge of it.
    pub fn from_offset(offset: u32, base_ref: usize) -> Self {
        Self {
            pointer: RelativePtr::new(offset),
            base_ref,
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
            id,
            blocks: Vec::<BlockMetadata>::new(),
        }
    }

    /// Produces a lightweight [`SharedHandler`] that can be sent to other threads.
    ///
    /// [`SharedHandler`] holds raw pointers into the SHM segment. This handler
    /// **must** outlive all shared handles derived from it — Rust cannot enforce
    /// this lifetime relationship automatically because `SharedHandler` uses raw
    /// pointers. Violating this contract is undefined behaviour.
    ///
    /// ### Returns:
    /// A new SharedHandler instance.
    pub fn share(&self) -> SharedHandler {
        SharedHandler {
            matrix_addr: self.matrix as *const AtomicMatrix as usize,
            base_addr: self.base_ptr() as usize,
            segment_size: self.segment_size(),
            first_block_offset: self.first_block_offset,
            blocks: &self.blocks,
        }
    }

    /// Removes the SHM file from the system.
    ///
    /// Should be called before the application exits to prevent the SHM file
    /// from persisting in `/dev/shm/` across runs. This is an explicit, opt-
    /// in cleanup. If omitted, the file survives until the system reboots
    /// or it's manually removed.
    ///
    /// ### Returns:
    /// A result containing either an empty Ok, or a HandlerErrors.
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
    fn block_list(&self) -> Vec<BlockMetadata> {
        self.blocks.clone()
    }
}

// Safety: AtomicMatrix uses only atomic operations internally.
// Caller guarantees the originating MatrixHandler outlives all SharedHandlers.
unsafe impl Send for SharedHandler<'_> {}
unsafe impl Sync for SharedHandler<'_> {}

impl HandlerFunctions for SharedHandler<'_> {
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
    fn block_list(&self) -> Vec<BlockMetadata> {
        self.blocks.to_vec()
    }
}

/// Defines the core interaction surface for any matrix handler.
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

    /// Returns the block list for the current handler.
    fn block_list(&self) -> Vec<BlockMetadata>;

    /// Allocates a block sized to hold `T`.
    ///
    /// Size is computed from `size_of::<T>()` and rounded up to the 16-byte
    /// minimum payload if necessary. The matrix remains typeless — type
    /// information exists only in the returned [`Block<T>`].
    ///
    /// # Returns:
    /// A result containing either the block of type T, or a HandlerErros.
    fn allocate<T>(&self) -> Result<Block<T>, HandlerErrors> {
        let tag = type_tag::make::<T>();
        let size = std::mem::size_of::<T>() as u32;

        let ptr = match self.matrix().allocate(self.base_ptr(), size + TAG_SIZE) {
            Ok(v) => v,
            Err(e) => {
                return Err(HandlerErrors::AllocationFailed {
                    reason: format!("{:?}", e),
                });
            }
        };

        unsafe { *(self.base_ptr().add(ptr.offset() as usize) as *mut u32) = tag };

        let block = Block::<T>::from_offset(ptr.offset() + TAG_SIZE, self.base_ptr() as usize);
        self.block_list().push(BlockMetadata {
            type_tag: tag,
            offset: block.pointer.offset(),
            len: size,
        });

        return Ok(block);
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
            Ok(v) => {
                self.block_list().push(BlockMetadata {
                    type_tag: 0,
                    offset: v.offset(),
                    len: size,
                });
                return Ok(v);
            }
            Err(e) => {
                return Err(HandlerErrors::AllocationFailed {
                    reason: format!("{:?}", e),
                });
            }
        }
    }

    /// Writes a value of type `T` into an allocated block.
    ///
    /// # Safety
    /// - No other thread may be reading or writing this block concurrently.
    ///   The caller is responsible for all synchronization beyond the atomic
    ///   state transitions provided by [`set_state`] and [`transition_state`].
    unsafe fn write<T>(&self, block: &mut Block<T>, value: T) -> Result<(), HandlerErrors> {
        let header = unsafe { block.pointer.resolve_header_mut(self.base_ptr()) };
        let state = &header.state;
        let total_size = header.size.load(Ordering::Acquire) - HEADER_SPACE;
        
        if !type_tag::compare::<T>(&block) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        if state
            .load_if_any(
                &[STATE_ACKED, STATE_COALESCING, STATE_FREE],
                Ordering::Relaxed,
            )
            .is_ok()
        {
            return Err(HandlerErrors::InvalidOffset {
                offset: block.pointer.offset(),
            });
        };

        unsafe {
            let payload = std::ptr::from_ref(&value);
            let size = std::mem::size_of_val(&value);
            let dst = self.base_ptr().add(block.pointer.offset() as usize) as *mut T;

            if size > total_size as usize {
                return Err(HandlerErrors::BlockOverflow { block_size: total_size as usize, payload_size: size })
            };

            std::ptr::copy_nonoverlapping(payload, dst, size);
        };

        header.last_edit.set_now();

        Ok(())
    }

    /// Writes a value of type `T` into an allocated block, if the block is in the provided state.
    ///
    /// If the target fails, it will not write the data into the block.
    fn write_if<T>(
        &self,
        block: &mut Block<T>,
        value: T,
        target: u32,
    ) -> Result<(), HandlerErrors> {
        let header = unsafe { block.pointer.resolve_header_mut(self.base_ptr()) };
        let state = &header.state;
        let total_size = header.size.load(Ordering::Acquire) - HEADER_SPACE;

        if !type_tag::compare::<T>(&block) {
            return Err(HandlerErrors::TypeMismatchError)
        };

        if state.load_if(target, Ordering::Acquire).is_err() {
            return Err(HandlerErrors::ReservedState { state: state.load(Ordering::Relaxed) })
        };

        unsafe {
            let payload = std::ptr::from_ref(&value);
            let size = std::mem::size_of_val(&value);
            let dst = self.base_ptr().add(block.pointer.offset() as usize) as *mut T;

            if size > total_size as usize {
                return Err(HandlerErrors::BlockOverflow { block_size: total_size as usize, payload_size: size});
            };

            std::ptr::copy_nonoverlapping(payload, dst, size);
        };
        
        header.last_edit.set_now();

        Ok(())
    }

    /// Writes a value of type `T` into an allocated block, after successfully transitioning its state
    /// into a targetted state.
    ///
    /// If the block is not into the targetted state, it will not be transitioned nor be written into.
    fn write_transition<T>(
        &self,
        block: &mut Block<T>,
        value: T,
        current: u32,
        next: u32,
        success_order: Ordering
    ) -> Result<(), HandlerErrors> {
        let header = unsafe { block.pointer.resolve_header(self.base_ptr()) };
        let state = &header.state;
        let total_size = header.size.load(Ordering::Acquire) - HEADER_SPACE;

        if !type_tag::compare::<T>(&block) {
            return Err(HandlerErrors::TypeMismatchError)
        };

        if state.compare_exchange(current, next, success_order, Ordering::Relaxed).is_err() {
            return Err(HandlerErrors::TransitionFailed { requested_state: current, current_state: state.load(Ordering::Relaxed) })
        };

        unsafe {
            let payload = std::ptr::from_ref(&value);
            let size = std::mem::size_of_val(&value);
            let dst = self.base_ptr().add(block.pointer.offset() as usize) as *mut T;

            if size > total_size as usize {
                return Err(HandlerErrors::BlockOverflow { block_size: total_size as usize, payload_size: size })
            };

            std::ptr::copy_nonoverlapping(payload, dst, size);
        };

        header.last_edit.set_now();

        Ok(())
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
    unsafe fn read<'a, T>(&self, block: &Block<T>) -> Result<&'a T, HandlerErrors> {
        if !type_tag::compare::<T>(&block) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        let data = unsafe { block.pointer.resolve(self.base_ptr()) };

        return Ok(data);
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
    unsafe fn read_mut<'a, T>(&self, block: &Block<T>) -> Result<&'a mut T, HandlerErrors> {
        if !type_tag::compare::<T>(&block) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        let mut_data = unsafe { block.pointer.resolve_mut(self.base_ptr()) };

        return Ok(mut_data);
    }

    /// Tries to query a block through a checked query.
    ///
    /// If the query fails, an error is return signaling why the query failed.
    ///
    /// ### Params:
    /// @offset: The offset of the block to be queried.
    ///
    /// ### Returns:
    /// An result containing either the requested block, or an InvalidOffset error containing the
    /// offset address.
    fn get_block<T>(&self, offset: u32) -> Result<Block<T>, HandlerErrors> {
        let v = match self.matrix().checked_query(self.base_ptr(), offset) {
            Ok(v) => v,
            Err(_) => return Err(HandlerErrors::InvalidOffset { offset: offset })
        };

        let block = Block::<T>::from_offset(v.offset(), self.base_ptr() as usize);

        if type_tag::compare::<T>(&block) {
            return Ok(block);
        } else {
            return Err(HandlerErrors::TypeMismatchError);
        }
    }

    /// Tries to query a list of blocks from the matrix through a checked query.
    ///
    /// A tuple containing a Vec with the successfull queried blocks and other with all failed
    /// offset addresses is returned after all queries are completed, so the caller can react to all
    /// failed offsets in a more adaptable way.
    ///
    /// ### Params:
    /// @offset_list: An array list containing all the offsets to be queried.
    ///
    /// ### Returns:
    /// A tuple containing a list of successfully queried blocks and a list of failed offsets.
    fn batch_get<T>(&self, offset_list: &[u32]) -> (Vec<Block<T>>, Vec<u32>) {
        let mut block_list = Vec::<Block<T>>::new();
        let mut failed_query = Vec::<u32>::new();

        for o in offset_list {
            match self.matrix().checked_query(self.base_ptr(), *o) {
                Ok(v) => {
                    let block = Block::<T>::from_offset(v.offset(), self.base_ptr() as usize);

                    if !type_tag::compare(&block) {
                        failed_query.push(*o);
                    } else {
                        block_list.push(block);
                    }
                }
                Err(_) => failed_query.push(*o),
            };
        }

        return (block_list, failed_query);
    }

    /// Frees a typed block.
    ///
    /// Marks the block `STATE_ACKED` and immediately triggers coalescing.
    /// The block is invalid after this call — using it in any way is
    /// undefined behaviour.
    fn free<T>(&self, block: Block<T>) {
        let header_ptr =
            RelativePtr::<BlockHeader>::new(block.pointer.offset() - HEADER_SPACE - TAG_SIZE);
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
            return Err(HandlerErrors::ReservedState { state: state });
        }

        let unpadded_block =
            Block::<T>::from_offset(block.pointer.offset() - TAG_SIZE, self.base_ptr() as usize);
        unsafe {
            unpadded_block
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
        let unpadded_block =
            Block::<T>::from_offset(block.pointer.offset() - TAG_SIZE, self.base_ptr() as usize);
        unsafe {
            unpadded_block
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
            return Err(HandlerErrors::ReservedState { state: next });
        }

        let unpadded_block =
            Block::<T>::from_offset(block.pointer.offset() - TAG_SIZE, self.base_ptr() as usize);
        let header = unsafe { unpadded_block.pointer.resolve_header_mut(self.base_ptr()) };
            match header
                .state
                .compare_exchange(expected, next, success_order, Ordering::Relaxed)
            {
                Err(_) => Err(HandlerErrors::TransitionFailed {
                    requested_state: expected,
                    current_state: header.state.load(Ordering::Relaxed)
                }),
                Ok(v) => Ok(v),
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
    /// ## DISCLAIMER
    ///
    /// This is not a fully fledged hasher, and its meant to compress ID Strings into 16 bytes.
    /// **DO NOT** use it for other things than its destined purpose.
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

impl Drop for MatrixHandler {
    fn drop(&mut self) {
        for block_md in self.blocks.iter() {
            self.free_at(block_md.offset - HEADER_SPACE);
        }
    }
}
