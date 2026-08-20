//! # Matrix High-Level API Handles
//!
//! This module encapsulates the matrix raw primitives into a more ergonomic API that abstracts a
//! lot of manual and repetitive work that has to be executed in order to correctly interact with
//! the matrix, as well as some safe pre-baked functions that add more extensibility over what can
//! be done generally.
//!
//! ## Abstraction Layers
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
//! ## Handler Scope
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
//! Safe handler functions also enforce typing at runtime through Type Tagging ( check the [`type_tag`]
//! helper module), returning a [`HandlerErrors::TypeMismatchError`] when the check fails. It also
//! provides checked queries to ensure that the block do exist inside the matrix
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
//! The matrix coalescing engine ignores any state beyond the ones described above, a block in state
//! 112 is never reclaimed automatically. Call `free()` explicitly when done.
//!
//! **Note:** States 4–48 are reserved for future internal state management implementations that have
//! not been planned yet. Better safe than sorry.
//!
//! ## Thread Sharing
//!
//! [`MatrixHandler`] owns the mmap and is not `Clone`. Use `share()` to produce a [`SharedHandler`]
//! that can be sent to other threads. The original handler must outlive all shared handles derived
//! from it.
//!
//! ## Safety
//!
//! All the operations assumes that each participant follows the safety contract proposed by the
//! high-level API. Adversarial behaviours that directly forge blocks, poison already existing
//! payloads, or just zero out the entire segment are undetectable at compile time.
//!
//! Although the API provides typed errors that can be treated locally if anything fails, more
//! safety surfaces must be implemented to ensure an attacker cannot reach the matrix SHM arena
//! directly (LSM protection, server hardening, infrastructure security, etc).

use crate::internals::error_collection::HandlerErrors;
use crate::helpers::type_guard::*;
use crate::prelude::*;
use crate::unwind;
use memmap2::MmapMut;
use std::sync::atomic::Ordering;

/// Minimum state value available for user-defined lifecycles.
/// States 0–48 are reserved for internal matrix and future framework use.
/// Currently only 0–3 are assigned — the remaining range (4–48) is reserved
/// for future internal lifecycle states without breaking user code.
pub const USER_STATE_MIN: u32 = 49;
pub const TAG_SIZE: u32 = std::mem::size_of::<u32>() as u32;
pub const STATE_LOCK: u32 = 4;

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
    pointer: RelativePtr<T>,
    base_ref: usize,
}

/// A lightweight reflection of the original handler that can be safely sent
/// across threads.
///
/// Produced by [`MatrixHandler::share()`]. Holds raw pointers into the SHM
/// segment. The originating [`MatrixHandler`] **must** outlive all
/// `SharedHandler` instances derived from it.
///
/// `SharedHandler` exposes the same allocation, I/O, lifecycle, and escape
/// hatch API as [`MatrixHandler`] via the [`HandlerFunctions`] trait.
/// It does not own the mmap.
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
/// Owns the SHM mapping. Cannot be cloned, use [`share()`] to produce a
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
    /// ### Safety:
    /// This function will construct a block from whatever offset you pass into it, unchecked. This
    /// behaviour must be taken into account when running code with it.
    ///
    /// ### Params:
    /// @offset: The u32 offset of the allocated block. \
    /// @base_ref: The base_ptr from MatrixHandler.
    ///
    /// ### Returns:
    /// An instance of self.
    pub(crate) fn from_offset(offset: u32, base_ref: usize) -> Self {
        Self {
            pointer: RelativePtr::new(offset),
            base_ref,
        }
    }

    /// Returns the underlying RelativePtr for this block.
    ///
    /// ### Safety:
    /// All operations inside the RelativePtr are raw pointer arithmetics, and considered unsafe by
    /// default. MatrixHandler abstracts these unsafe behaviours with a barrage of checks to ensure
    /// ops are valid at the surface of the call.
    ///
    /// ### Returns:
    /// The RelativePtr typed to T.
    pub fn pointer(&self) -> &RelativePtr<T> {
        return &self.pointer;
    }

    /// Returns the direct header offset for the block.
    ///
    /// This makes the pointer arithmetic for the header standard across all the codebase.
    ///
    /// ### Returns:
    /// The direct header offset as an u32 integer.
    pub fn header(&self) -> u32 {
        self.pointer.offset() - HEADER_SPACE - TAG_SIZE
    }

    /// Returns an offset to the block start, without the type_tag.
    ///
    /// This makes the pointer arithmetic for the clean block state standard across the codebase.
    ///
    /// ### Returns:
    /// A generic RelativePtr of the block, without the type_tag offset.
    pub fn tagless_ptr(&self) -> RelativePtr<u8> {
        RelativePtr::<u8>::new(self.pointer.offset() - TAG_SIZE)
    }
}

impl MatrixHandler {
    /// Internal constructor. Called exclusively by [`AtomicMatrix::bootstrap`].
    pub(crate) fn new(
        matrix: &'static mut AtomicMatrix,
        mmap: MmapMut,
        first_block_offset: u32,
        id: String
    ) -> Self {
        Self {
            matrix,
            mmap,
            first_block_offset,
            id,
        }
    }

    /// Produces a lightweight [`SharedHandler`] that can be sent to other threads.
    ///
    /// [`SharedHandler`] holds raw pointers into the SHM segment. This handler **must** outlive all
    /// shared handles derived from it, as Rust cannot enforce this lifetime relationship automatically
    /// because `SharedHandler` uses raw pointers. Violating this contract is undefined behaviour.
    ///
    /// ### Returns:
    /// A new SharedHandler instance.
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
    /// Should be called before the application exits to prevent the SHM file from persisting in
    /// `/dev/shm/` across runs. This is an explicit, opt-in cleanup. If omitted, the file survives
    /// until the system reboots, or it's manually removed.
    ///
    /// ### Safety:
    /// `die` deallocate the whole SHM arena, disregarding whatever other processes is hooked onto
    /// it at execution time. Therefore, it should be timed across all participants at the caller
    /// side.
    ///
    /// ### Returns:
    /// A result containing either an empty Ok, or a HandlerErrors.
    pub unsafe fn die(&self) -> Result<(), HandlerErrors> {
        let id = &self.id;
        let path = format!("/dev/shm/matrix-{}", id);

        match std::fs::remove_file(&path) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                return Err(HandlerErrors::DecomissionFailed { path, reason: e });
            }
        }

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

        unsafe {
            *(self.base_ptr().add(ptr.offset() as usize) as *mut u32) = tag;
        }

        let block = Block::<T>::from_offset(ptr.offset() + TAG_SIZE, self.base_ptr() as usize);

        return Ok(block);
    }

    /// Allocates a raw byte block of the given size.
    ///
    /// This returns a raw RelativePtr as oposed to a typed block from allocate. Therefore, the
    /// pointer has no safe type at compile time.
    ///
    /// ### Params:
    /// @size: The size of the block to be allocated.
    ///
    /// ### Returns
    /// A result containing either the RelativePtr, or a HandlerErrors.
    fn allocate_raw(&self, size: u32) -> Result<RelativePtr<u8>, HandlerErrors> {
        match self.matrix().allocate(self.base_ptr(), size) {
            Ok(v) => {
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
    /// ### Safety
    /// This function is a mostly unchecked write. The only safety provided by it is that the type
    /// of the block image and the segment in the matrix have the same type tag, and that the block is
    /// not in any of the forbidden states. All other safety checks are excluded.
    ///
    /// ### Params:
    /// @block: The block on which the value will be written to. \
    /// @value: The value to write into the block.
    ///
    /// ### Returns:
    /// A result containing an empty Ok, or a HandlerErrors
    unsafe fn write<T>(&self, block: &mut Block<T>, value: T) -> Result<(), HandlerErrors> {
        let header = unsafe { block.tagless_ptr().resolve_header_mut(self.base_ptr()) };
        let state = &header.state;
        let total_size = header.size.load(Ordering::Acquire) - HEADER_SPACE;

        if !type_tag::compare::<T>(&block, self.base_ptr()) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        if
            state
                .load_if_any(
                    &[STATE_ACKED, STATE_COALESCING, STATE_FREE, STATE_LOCK],
                    Ordering::Relaxed
                )
                .is_ok()
        {
            return Err(HandlerErrors::InvalidOffset {
                offset: block.pointer.offset(),
            });
        }

        unsafe {
            let payload = std::ptr::from_ref(&value);
            let size = std::mem::size_of_val(&value);
            let dst = self.base_ptr().add(block.pointer.offset() as usize) as *mut T;

            if size > (total_size as usize) {
                return Err(HandlerErrors::BlockOverflow {
                    block_size: total_size as usize,
                    payload_size: size,
                });
            }

            std::ptr::copy_nonoverlapping(payload, dst, size);
        }

        header.last_edit.set_now();

        Ok(())
    }

    /// Writes a value of type `T` into an allocated block, if the block is in the provided state.
    ///
    /// If the target fails, it will not write the data into the block.
    ///
    /// ### Params:
    /// @block: The block to write the value into. \
    /// @value: The value to write into the block. \
    /// @taget: The targetted state of the block.
    ///
    /// ### Returns:
    /// A result containing either an empty Ok, or a HandlerErrors.
    fn write_if<T>(
        &self,
        block: &mut Block<T>,
        value: T,
        target: u32
    ) -> Result<(), HandlerErrors> {
        if target < USER_STATE_MIN || target == STATE_LOCK {
            return Err(HandlerErrors::ReservedState { state: target });
        }
        self
            .matrix()
            .checked_query(self.base_ptr(), block.header())
            .map_err(|_| HandlerErrors::InvalidOffset { offset: block.pointer().offset() })?;

        let header = unsafe { block.tagless_ptr().resolve_header_mut(self.base_ptr()) };
        let state = &header.state;
        let total_size = header.size.load(Ordering::Acquire) - HEADER_SPACE;

        if !type_tag::compare::<T>(&block, self.base_ptr()) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        if state.load_if(target, Ordering::Acquire).is_err() {
            return Err(HandlerErrors::ReservedState {
                state: state.load(Ordering::Relaxed),
            });
        }

        unsafe {
            let payload = std::ptr::from_ref(&value);
            let size = std::mem::size_of_val(&value);
            let dst = self.base_ptr().add(block.pointer.offset() as usize) as *mut T;

            if size > (total_size as usize) {
                return Err(HandlerErrors::BlockOverflow {
                    block_size: total_size as usize,
                    payload_size: size,
                });
            }

            std::ptr::copy_nonoverlapping(payload, dst, size);
        }

        header.last_edit.set_now();

        Ok(())
    }

    /// Writes a value of type `T` into an allocated block, after successfully transitioning its state
    /// into a targetted state.
    ///
    /// If the block is not into the targetted state, it will not be transitioned nor be written into.
    ///
    /// ### Params:
    /// @block: The block to write the value into. \
    /// @value: The value to write into the block. \
    /// @current: The targetted state of the block. \
    /// @next: The state to transition the block into. \
    /// @sucess_order: Atomic ordering for a successful transition.
    ///
    /// ### Returns:
    /// A result containing either an empty Ok, or a HandlerErrors.
    fn write_transition<T>(
        &self,
        block: &mut Block<T>,
        value: T,
        current: u32,
        next: u32,
        success_order: Ordering
    ) -> Result<(), HandlerErrors> {
        if next < USER_STATE_MIN || next == STATE_LOCK {
            return Err(HandlerErrors::ReservedState { state: next });
        }
        self
            .matrix()
            .checked_query(self.base_ptr(), block.header())
            .map_err(|_| HandlerErrors::InvalidOffset { offset: block.pointer().offset() })?;

        let header = unsafe { block.tagless_ptr().resolve_header(self.base_ptr()) };
        let state = &header.state;
        let total_size = header.size.load(Ordering::Acquire) - HEADER_SPACE;

        if !type_tag::compare::<T>(&block, self.base_ptr()) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        if state.compare_exchange(current, next, success_order, Ordering::Relaxed).is_err() {
            return Err(HandlerErrors::TransitionFailed {
                requested_state: current,
                current_state: state.load(Ordering::Relaxed),
            });
        }

        unsafe {
            let payload = std::ptr::from_ref(&value);
            let size = std::mem::size_of_val(&value);
            let dst = self.base_ptr().add(block.pointer.offset() as usize) as *mut T;

            if size > (total_size as usize) {
                return Err(HandlerErrors::BlockOverflow {
                    block_size: total_size as usize,
                    payload_size: size,
                });
            }

            std::ptr::copy_nonoverlapping(payload, dst, size);
        }

        header.last_edit.set_now();

        Ok(())
    }

    /// Reads a Type Guarded shared reference to `T` from an allocated block.
    ///
    /// The value of T stays live inside the matrix as long as the segment exists.
    ///
    /// ### Safety:
    /// Deferencing this value checks if it still exists inside the matrix before returning the
    /// value. If the value has been deallocated, the call will panic. It also does not ensure
    /// exclusive access to the value.
    ///
    /// TypeGuard provides a safer method called `try_get()` that returns None if the value has been
    /// deallocated instead of panicking.
    ///
    /// ### Unwinding
    /// The macro [`unwind!`] shipped with the crate helps convert the panic into a treatable
    /// MatrixErrors, so raw deferencing doesn't crash everything into the ground.
    ///
    /// ### Params:
    /// @block: The block to get a reference from.
    ///
    /// ### Returns:
    /// A result containing either a TypeGuard<T>, or a HandlerErrors.
    fn read<'a, T>(&self, block: &Block<T>) -> Result<TypeGuard<T>, HandlerErrors> {
        match self.matrix().checked_query(self.base_ptr(), block.header()) {
            Ok(_) => {}
            Err(_) => {
                return Err(HandlerErrors::InvalidOffset {
                    offset: block.pointer.offset(),
                });
            }
        }

        if !type_tag::compare::<T>(&block, self.base_ptr()) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        let data = TypeGuard::<T>::new(
            block.pointer().offset(),
            block.header(),
            block.base_ref as *const u8
        );

        return Ok(data);
    }

    /// Reads a mutable reference to `T` from an allocated block.
    ///
    /// ### Safety:
    /// Deferencing this value checks if it still exists inside the matrix before returning the
    /// value. If the value has been deallocated, the call will panic. It also does not ensure
    /// exclusive access to the value.
    ///
    /// TypeGuard provides a safer method called `try_get()` that returns None if the value has been
    /// deallocated instead of panicking
    ///
    /// ### Unwinding
    /// The macro [`unwind!`] shipped with the crate helps convert the panic into a treatable
    /// MatrixErrors, so raw deferencing doesn't crash everything into the ground.
    ///
    /// ### Params:
    /// @block: The block to get a reference from.
    ///
    /// ### Returns:
    /// A result containing either a mutable reference to T, or a HandlerErrors.
    fn read_mut<'a, T>(&self, block: &Block<T>) -> Result<TypeGuardMut<T>, HandlerErrors> {
        match self.matrix().checked_query(self.base_ptr(), block.header()) {
            Ok(_) => {}
            Err(_) => {
                return Err(HandlerErrors::InvalidOffset {
                    offset: block.pointer.offset(),
                });
            }
        }

        if !type_tag::compare::<T>(&block, self.base_ptr()) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        let mut_data = TypeGuardMut::<T>::new(
            block.pointer().offset(),
            block.header(),
            block.base_ref as *const u8
        );

        return Ok(mut_data);
    }

    /// Reads a copy of `T` from an allocated block.
    ///
    /// This function extract a thread safe copy of the value that cannot be changed nor dropped by
    /// other participants, with the downside of being a stale copy that may not reflect the latest
    /// state of the value.
    ///
    /// ### Params:
    /// @block: The block to get a reference from.
    ///
    /// ### Returns:
    /// A result containing either a copy of T, or a HandlerErrors.
    fn read_copy<T>(&self, block: &Block<T>) -> Result<T, HandlerErrors> {
        match self.matrix().checked_query(self.base_ptr(), block.header()) {
            Ok(_) => {}
            Err(_) => {
                return Err(HandlerErrors::InvalidOffset {
                    offset: block.pointer.offset(),
                });
            }
        }

        if !type_tag::compare::<T>(&block, self.base_ptr()) {
            return Err(HandlerErrors::TypeMismatchError);
        }

        let data_cp = unsafe { std::ptr::read(block.pointer.resolve(self.base_ptr())) };

        Ok(data_cp)
    }

    /// Locks the block while inside a provided expression scope, giving a reference to the
    /// underlying value.
    ///
    /// This function ensures an exclusive read operation on the block data inside the MatrixHandler
    /// contract, as it transitions the block to STATE_LOCK, and every participant that adheres to
    /// the handler API respect the block present state.
    /// 
    /// If a call stumbles upon a STATE_LOCK block, it will retry to acquire exclusivity 512 times 
    /// before returning None
    ///
    /// ### Warning
    ///
    /// If a participant crashes holding this block inline ref, the segment will stale inside
    /// STATE_LOCK unless some action is taken. Callers are required to manage their own block
    /// recovery logic, as blocks can still have their state transitioned manually even when set 
    /// to LOCK
    ///
    /// ### Params:
    /// @block: The block to execute the expression on. \
    /// @expr: The inline closure to execute.
    ///
    /// ### Returns:
    /// An Option stating the success of the execution.
    fn inline<T, F>(&self, block: &Block<T>, expr: F) -> Option<()> where F: FnOnce(TypeGuard<T>) {
        let mut retry_count = 0;
        self.matrix().checked_query(self.base_ptr(), block.header()).ok()?;
            
        let header_ptr = RelativePtr::<BlockHeader>::new(block.header());
        let header = unsafe { header_ptr.resolve_mut(self.base_ptr()) };

        loop {
            let curr_state = self.get_state(block, Ordering::Acquire).ok()?;

            if curr_state == STATE_LOCK {
                retry_count += 1;
                if retry_count >= 512 { return None };
                continue;
            }

            match
                header.state.compare_exchange(
                    curr_state,
                    STATE_LOCK,
                    Ordering::Acquire,
                    Ordering::Relaxed
                )
            {
                Ok(_) => {
                    let exec_res = match self.read(block) {
                        Ok(rt_ref) => unwind!(expr(rt_ref)),
                        Err(e) => Err(e),
                    };
                    header.state.store(curr_state, Ordering::Release);
                    exec_res.ok()?;

                    break;
                },
                Err(_) => {
                    retry_count += 1;
                    if retry_count >= 512 {
                        return None
                    } else {
                        continue
                    }
                }
            }
        }

        return Some(());
    }

    /// Locks the block while inside a provided expression scope, giving a mutable reference to the
    /// underlying value.
    ///
    /// This function ensures an exclusive read/write operation on the block data inside the
    /// MatrixHandler contract, as it transitions the block to STATE_LOCK, and every participant that
    /// adheres to the handler API respect the block present state.
    /// 
    /// If a call stumbles upon a STATE_LOCK block, it will retry to acquire exclusivity 512 times 
    /// before returning None
    ///
    /// ### Warning
    ///
    /// If a participant crashes holding this block inline ref, the segment will stale inside
    /// STATE_LOCK unless some action is taken. Callers are required to manage their own block
    /// recovery logic, as blocks can still have their state transitioned manually even when set 
    /// to LOCK
    ///
    /// ### Params:
    /// @block: The block to execute the expression on. \
    /// @expr: The inline closure to execute.
    ///
    /// ### Returns:
    /// An Option stating the success of the execution.
    fn inline_mut<T, F>(&self, block: &Block<T>, expr: F) -> Option<()>
        where F: FnOnce(TypeGuardMut<T>)
    {
        let mut retry_count = 0;
        self.matrix().checked_query(self.base_ptr(), block.header()).ok()?;
            
        let header_ptr = RelativePtr::<BlockHeader>::new(block.header());
        let header = unsafe { header_ptr.resolve_mut(self.base_ptr()) };

        loop {
            let curr_state = header.state.load(Ordering::Acquire);

            if curr_state == STATE_LOCK {
                retry_count += 1;
                if retry_count >= 512 { return None };
                continue;
            }

            match
                header.state.compare_exchange(
                    curr_state,
                    STATE_LOCK,
                    Ordering::Acquire,
                    Ordering::Relaxed
                )
            {
                Ok(_) => {
                    let exec_res = match self.read_mut(block) {
                        Ok(rt_ref_mut) => unwind!(expr(rt_ref_mut)),
                        Err(e) => Err(e),
                    };
                    header.state.store(curr_state, Ordering::Release);
                    exec_res.ok()?;

                    break;
                },
                Err(_) => {
                    retry_count += 1;
                    if retry_count >= 512 { return None };
                    continue;
                }
            }        
        }

        return Some(());
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
            Err(e) => {
                return Err(HandlerErrors::InnerMatrixError(e));
            }
        };

        let block = Block::<T>::from_offset(v.offset() + TAG_SIZE, self.base_ptr() as usize);

        if type_tag::compare::<T>(&block, self.base_ptr()) {
            return Ok(block);
        } else {
            return Err(HandlerErrors::TypeMismatchError);
        }
    }

    /// Tries to query a list of blocks from the matrix through a checked query.
    ///
    /// A tuple containing a Vec with the successfull queried blocks and other with all failed
    /// offset addresses is returned after all queries are completed, so the caller can react to all
    /// failed offsets in a more dynamic way.
    ///
    /// The specific errors for each failed query are completelly abstracted from the call, giving
    /// place to the list of failed queries only.
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
                    let block = Block::<T>::from_offset(
                        v.offset() + TAG_SIZE,
                        self.base_ptr() as usize
                    );

                    if !type_tag::compare(&block, self.base_ptr()) {
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
    /// The block is invalid after this call.
    ///
    /// ### Params:
    /// @block: The block to be freed.
    ///
    /// ### Returns:
    /// A result containing an empty Ok, or a HandlerErrors
    fn free<T>(&self, block: Block<T>) -> Result<(), HandlerErrors> {
        self
            .matrix()
            .checked_query(self.base_ptr(), block.header())
            .map_err(|_| HandlerErrors::InvalidOffset { offset: block.pointer.offset() })?;

        let header = unsafe { block.tagless_ptr().resolve_header(self.base_ptr()) };
        if
            header.state
                .load_if_any(
                    &[STATE_ACKED, STATE_COALESCING, STATE_FREE, STATE_LOCK],
                    Ordering::Acquire
                )
                .is_ok()
        {
            return Err(HandlerErrors::InvalidOffset { offset: block.pointer.offset() });
        }

        let header_ptr = RelativePtr::<BlockHeader>::new(block.header());
        self.matrix().ack(&header_ptr, self.base_ptr());

        Ok(())
    }

    /// Sets a user-defined lifecycle state on a block.
    ///
    /// Any state provided that is bellow the defined min state (49) will fail the call.
    ///
    /// ### Params:
    /// @block: The block to apply the desired state. \
    /// @state: The custom state to apply into the block header.
    ///
    /// ### Returns:
    /// A result containing either an empty Ok, or a HandlerErrors.
    fn set_state<T>(&self, block: &Block<T>, state: u32) -> Result<(), HandlerErrors> {
        match self.matrix().checked_query(self.base_ptr(), block.header()) {
            Ok(_) => {}
            Err(_) => {
                return Err(HandlerErrors::InvalidOffset {
                    offset: block.pointer.offset(),
                });
            }
        }

        if state < USER_STATE_MIN {
            return Err(HandlerErrors::ReservedState { state: state });
        }

        unsafe {
            block
                .tagless_ptr()
                .resolve_header_mut(self.base_ptr())
                .state.store(state, Ordering::Release);
        }
        Ok(())
    }

    /// Returns the current state of a block.
    ///
    /// ### Params:
    /// @block: The block to apply the desired state. \
    /// @order: The atomic ordering for the operation.
    ///
    /// ### Returns:
    /// A result containing either the requested state, or a HandlerErrors.
    fn get_state<T>(&self, block: &Block<T>, order: Ordering) -> Result<u32, HandlerErrors> {
        match self.matrix().checked_query(self.base_ptr(), block.header()) {
            Ok(_) => {}
            Err(_) => {
                return Err(HandlerErrors::InvalidOffset {
                    offset: block.pointer.offset(),
                });
            }
        }

        unsafe { Ok(block.tagless_ptr().resolve_header(self.base_ptr()).state.load(order)) }
    }

    /// Atomically transitions a block from one state to another.
    ///
    /// Succeeds only if the block is currently in `expected`, and `next` must be >= [`USER_STATE_MIN`],
    /// as transitioning into an internal state is not permitted. Failed transition attempts are always
    /// executed as Ordering::Relaxed.
    ///
    /// ### Params:
    /// @block: The block on which to apply the state transition. \
    /// @expected: The current expected state of the block. \
    /// @next: The state to apply in case exchange succeeds. \
    /// @success_ordering: Atomic ordering to execute the transition if it succeeds.
    ///
    /// ### Returns:
    /// A result containing either the expected state, or a HandlerErrors.
    fn transition_state<T>(
        &self,
        block: &Block<T>,
        expected: u32,
        next: u32,
        success_order: Ordering
    ) -> Result<u32, HandlerErrors> {
        match self.matrix().checked_query(self.base_ptr(), block.header()) {
            Ok(_) => {}
            Err(_) => {
                return Err(HandlerErrors::InvalidOffset {
                    offset: block.pointer.offset(),
                });
            }
        }

        if next < USER_STATE_MIN {
            return Err(HandlerErrors::ReservedState { state: next });
        }

        let header = unsafe { block.tagless_ptr().resolve_header_mut(self.base_ptr()) };
        match header.state.compare_exchange(expected, next, success_order, Ordering::Relaxed) {
            Err(_) =>
                Err(HandlerErrors::TransitionFailed {
                    requested_state: expected,
                    current_state: header.state.load(Ordering::Relaxed),
                }),
            Ok(v) => Ok(v),
        }
    }

    /// Returns a raw reference to the underlying [`AtomicMatrix`].
    fn raw_matrix(&self) -> &AtomicMatrix {
        self.matrix()
    }

    /// Returns the raw SHM base pointer for this process's mapping.
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
    ///
    /// ### Returns:
    /// The 16 byte array derived from the provided ID.
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

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use crate::core::matrix::AtomicMatrix;
    use super::*;

    const SIZE: usize = memory_scale::two::KB;

    #[test]
    fn test_typed_block_allocation() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();

        let val = handler.read_copy(&block).unwrap();

        assert_eq!(std::any::type_name_of_val(&val), std::any::type_name::<u32>());
        assert_eq!(val, 0); // Unwritten blocks should always have a value of 0, regardless of type.

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_typed_write() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let mut block = handler.allocate::<u32>().unwrap();
        println!("{}", std::mem::align_of::<BlockHeader>());

        let res = unsafe { handler.write(&mut block, 400) };

        assert_eq!(res.is_ok(), true);

        let read_res = handler.read_copy(&block);

        assert_eq!(read_res.is_ok(), true);
        assert_eq!(read_res.unwrap(), 400);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_write_transition() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let mut block = handler.allocate::<u32>().unwrap();

        handler
            .write_transition(&mut block, 5000, STATE_ALLOCATED, 5000, Ordering::Release)
            .unwrap();
        let header = unsafe { block.tagless_ptr().resolve_header(handler.base_ptr()) };
        let val = handler.read_copy(&block).unwrap();

        assert_eq!(header.state.load(Ordering::Relaxed), 5000);
        assert_eq!(val, 5000);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_write_transition_fail() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let mut block = handler.allocate::<u32>().unwrap();

        let res = handler.write_transition(&mut block, 5000, 9999, 5000, Ordering::Release);
        let header = unsafe { block.tagless_ptr().resolve_header(handler.base_ptr()) };
        let val = handler.read_copy(&block).unwrap();

        assert_eq!(res.is_ok(), false);
        assert_eq!(header.state.load(Ordering::Relaxed), STATE_ALLOCATED);
        assert_eq!(val, 0);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_write_if() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let mut block = handler.allocate::<u32>().unwrap();

        handler.set_state(&mut block, 5000).unwrap();
        let res = handler.write_if(&mut block, 1, 5000);
        let val = handler.read_copy(&block).unwrap();

        assert_eq!(res.is_ok(), true);
        assert_eq!(val, 1);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_write_if_fail() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let mut block = handler.allocate::<u32>().unwrap();

        let res = handler.write_if(&mut block, 1, 5000);
        let val = handler.read_copy(&block).unwrap();

        assert_eq!(res.is_ok(), false);
        assert_eq!(val, 0);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_type_mismatch_detection() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();

        let mismatched_block = handler.get_block::<&str>(block.header());

        assert_eq!(mismatched_block.is_ok(), false);
        assert!(matches!(mismatched_block, Err(HandlerErrors::TypeMismatchError)));

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_use_after_free() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();
        let mut cloned_block = handler.get_block::<u32>(block.header()).unwrap();
        let _err: Result<(), HandlerErrors> = Err(HandlerErrors::InvalidOffset {
            offset: cloned_block.pointer().offset(),
        });

        handler.free(block).unwrap();
        std::thread::sleep(std::time::Duration::from_nanos(100));

        let res = handler.write_if(&mut cloned_block, 10, 1);

        assert_eq!(res.is_ok(), false);
        assert!(matches!(res, _err));

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_double_free() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();
        let cloned_block = handler.get_block::<u32>(block.header()).unwrap();
        let _err: Result<(), HandlerErrors> = Err(HandlerErrors::InvalidOffset {
            offset: cloned_block.pointer().offset(),
        });

        handler.free(block).unwrap();
        std::thread::sleep(std::time::Duration::from_nanos(100));
        let res = handler.free(cloned_block);

        assert_eq!(res.is_ok(), false);
        assert!(matches!(res, _err));

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_successful_state_transition() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();

        let res = handler.transition_state(&block, STATE_ALLOCATED, 100, Ordering::Release);
        let state = handler.get_state(&block, Ordering::Acquire).unwrap();

        assert_eq!(res.is_ok(), true);
        assert_eq!(state, 100);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_fail_transition_to_reserved_states() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();

        let res = handler.transition_state(&block, STATE_ALLOCATED, 6, Ordering::Release);

        assert_eq!(res.is_ok(), false);
        assert!(matches!(res, Err(HandlerErrors::ReservedState { state: 6 })));

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_successful_state_setting() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();

        let res = handler.set_state(&block, 100);
        let state = handler.get_state(&block, Ordering::Acquire).unwrap();

        assert_eq!(res.is_ok(), true);
        assert_eq!(state, 100);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_fail_setting_reserved_state() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();

        let res = handler.set_state(&block, 10);

        assert_eq!(res.is_ok(), false);
        assert!(matches!(res, Err(HandlerErrors::ReservedState { state: 10 })));

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_mutref_usability() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let mut block = handler.allocate::<u32>().unwrap();

        unsafe {
            handler.write(&mut block, 50).unwrap();
        }

        let real_time_ref = handler.read(&block).unwrap();
        let mut mutable_ref = handler.read_mut(&block).unwrap();

        assert_eq!(*real_time_ref, 50);

        *mutable_ref += 50;

        assert_eq!(*real_time_ref, 100);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_type_guard_panic() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let block = handler.allocate::<u32>().unwrap();

        let real_time_ref = handler.read(&block).unwrap();
        let mut mutable_ref = handler.read_mut(&block).unwrap();

        let success = unwind!({
            *mutable_ref += 2000;
        });

        assert!(success.is_ok());
        assert!(*real_time_ref == 2000);

        handler.free(block).unwrap();

        let fail = unwind!({
            *mutable_ref += 2000;
        });

        assert!(fail.is_err());
        assert!(matches!(fail, Err(HandlerErrors::PanicRecovery)));

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_inline_mut_multi_producer() {
        use std::thread;

        let increment = 10_000;

        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let mut block = handler.allocate::<u32>().unwrap();
        unsafe {
            handler.write(&mut block, 0).unwrap();
        }

        thread::scope(|s| {
            let coord = block.header();
            let s_handler = handler.share();

            s.spawn(move || {
                let b = s_handler.get_block::<u32>(coord).unwrap();
                let local_ref = s_handler.read(&b).unwrap();
                loop {
                    if
                        s_handler
                            .inline_mut(&b, |mut v| {
                                if *v < increment {
                                    *v += 2;
                                    println!("{}", *v);
                                }
                            })
                            .is_none()
                    {
                        continue;
                    }

                    if *local_ref >= increment {
                        break;
                    }
                }
            });
            s.spawn(move || {
                let b = s_handler.get_block::<u32>(coord).unwrap();
                let local_ref = s_handler.read(&b).unwrap();
                loop {
                    if
                        s_handler
                            .inline_mut(&b, |mut v| {
                                if *v < increment {
                                    *v += 4;
                                    println!("{}", *v);
                                }
                            })
                            .is_none()
                    {
                        continue;
                    }

                    if *local_ref >= increment {
                        break;
                    }
                }
            });
            s.spawn(move || {
                let b = s_handler.get_block::<u32>(coord).unwrap();
                let local_ref = s_handler.read(&b).unwrap();
                loop {
                    if
                        s_handler
                            .inline_mut(&b, |mut v| {
                                if *v < increment {
                                    *v += 8;
                                    println!("{}", *v);
                                }
                            })
                            .is_none()
                    {
                        continue;
                    }

                    if *local_ref >= increment {
                        break;
                    }
                }
            });
            s.spawn(move || {
                let b = s_handler.get_block::<u32>(coord).unwrap();
                let local_ref = s_handler.read(&b).unwrap();
                loop {
                    if
                        s_handler
                            .inline_mut(&b, |mut v| {
                                if *v < increment {
                                    *v += 64;
                                    println!("{}", *v);
                                }

                                thread::sleep(Duration::from_secs(1));
                            })
                            .is_none()
                    {
                        continue;
                    }
                    if *local_ref >= increment {
                        break;
                    }
                }
            });
        });

        let val_ref = handler.read(&block).unwrap();

        assert!(*val_ref >= increment);

        unsafe { handler.die().unwrap() }
    }
}
