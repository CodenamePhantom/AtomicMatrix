//! # Atomic Matrix Core
//!
//! This module implements a high-velocity, lock-free memory arena designed for
//! ultra-low latency IPC (Inter-Process Communication).
//!
//! # Theory of Operation
//! Unlike traditional allocators that use centralized mutexes or complex
//! background garbage collection, the **AtomicMatrix** treats memory
//! fragmentation as a fluid dynamics problem.
//!
//! 1. **Kinetic Healing:** Freeing a block triggers a "Ripple" ('coalesce')
//! that propagates through the sector.
//!
//! 2. **Monotonicity:** Ripples only move backward (towards the sector origin)
//! to prevent circular atomic dependencies and deadlocks.
//!
//! 3. **Permissive Concurrency:** If a thread encounters contention, it skips the
//! block rather than blocking, relying on the high frequency of future operations
//! to complete the healing.
//!
//! # Memory Topography
//! The matrix is laid out linearly in a shared memory segment:
//! ```text
//! [ Init Guard (16b) ] [ AtomicMatrix Struct ] [ Padding ] [ Data Sector ]
//! ```
//!
//! # Safety & Atomicity
//! All state transitions follow a strict 'STATE_FREE -> STATE_ALLOCATED ->
//! STATE_ACKED -> STATE_COALESCING' lifecycle. Hardware-level memory fences
//! (std::sync::atomic::fence) are utilized to ensure visibility across all CPU
//! cores without locking.

use memmap2::MmapMut;
use std::fs::OpenOptions;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicU32, Ordering, fence};

pub mod helpers {
    use super::*;

    /// System initialization flags
    pub const SYS_UNINITIALIZED: u32 = 0;
    pub const SYS_FORMATTING: u32 = 1;
    pub const SYS_READY: u32 = 2;

    /// Header lifecycle flags
    pub const STATE_FREE: u32 = 0;
    pub const STATE_ALLOCATED: u32 = 1;
    pub const STATE_ACKED: u32 = 2;
    pub const STATE_COALESCING: u32 = 3;

    pub const HEADER_SPACE: u32 = std::mem::size_of::<BlockHeader>() as u32;
    pub const METADATA_SPACE: u32 = (16 + std::mem::size_of::<AtomicMatrix>() as u32 + 15) & !15;

    pub const HEADER_SIGNATURE: u32 = 0xCADAFACE;

    /// Maps a block size to its corresponding (First-Level, Second-Level) indices.
    ///
    /// This function implements a two-level mapping strategy used for O(1) free-block
    /// lookup, optimized for both high-velocity small allocations and logarithmic
    /// scaling of large blocks.
    ///
    /// ### Mapping Logic:
    ///
    /// - **Linear (Small):** For sizes < 128, it uses a fixed FL (0) and 16-byte SL
    ///   subdivisions. This minimizes fragmentation for tiny objects.
    /// - **Logarithmic (Large):** For sizes >= 128, FL is the power of 2 (determined via
    ///   `leading_zeros`), and SL is a 3-bit subdivider of the range between 2^n and 2^(n+1).
    ///
    /// ### Mathematical Transformation:
    /// - `FL = log2(size)`
    /// - `SL = (size - 2^FL) / (2^(FL - 3))`
    ///
    /// ### Bounds:
    /// Indices are clamped to `(31, 7)` to prevent overflow in the matrix bitmask.
    ///
    /// # Arguments
    /// @size: The total byte size of the memory block.
    ///
    /// # Returns
    /// A tuple of `(fl, sl)` indices.
    pub fn find_indices(size: u32) -> (u32, u32) {
        if size < 128 {
            (0, (size / 16).min(7))
        } else {
            let fl = 31 - size.leading_zeros();
            let sl = ((size >> (fl - 3)) & 0x7).min(7);
            (fl.min(31), sl)
        }
    }
}

use crate::internals::collections::atomic_timestamp::AtomicTimestamp;
use crate::internals::collections::uid_lite::generate_uuid;
use crate::internals::error_collection::MatrixErrors;
use crate::prelude::*;

/// Header structure that is written at the beginning of each block/sector
///
/// The block is made entirely of atomic primitives to ensure safe reading
/// and manipulation across participant modules in the matrix.
#[derive(Debug)]
#[repr(C, align(16))]
pub struct BlockHeader {
    pub(crate) signature: u32,
    pub size: AtomicU32,
    pub state: AtomicU32,
    pub prev_phys: AtomicU32,
    pub next_free: AtomicU32,
    pub created_at: AtomicTimestamp,
    pub last_edit: AtomicTimestamp,
}

/// The structural core of the matrix.
///
/// Its the non-blocking, SHM-backed memory arena, utilizing a segmented **TLSF
/// (Two-Level segregated fit)** inspired mapping for O(1) allocation, paired with
/// a custom **Kinetic Coalescing** logic.
///
/// # Memory Layout
///
/// The matrix is designed to be mapped directly into '/dev/shm". It starts with
/// a 16-byte 'init_guard' followed by the struct itself, and then the sectorized
/// raw memory blocks.
#[repr(C, align(16))]
pub struct AtomicMatrix {
    pub fl_bitmap: AtomicU32,
    pub sl_bitmaps: [AtomicU32; 32],
    pub matrix: [[AtomicU32; 8]; 32],
    pub sector_boundaries: AtomicU32,
    pub total_size: u32,
}

/// A Relative Pointer to the block memory address, relative to the start of the
/// matrix inside the process memory scope.
///
/// This RelativePointer is used to calculate the accurate address of the block its
/// related to. Providing a way for independent process to localize the data inside
/// their own mappings of the SHM segment.
///
/// It also receives a PhantomData to inform the compiler we safely own whatever
/// generic type the caller has passed to this pointer.
#[derive(Debug)]
pub struct RelativePtr<T> {
    offset: u32,
    _marker: PhantomData<T>,
}

impl AtomicMatrix {
    /// Initialized the matrix struct and returns it.
    ///
    /// This function will initialize both TLSF level flags, the matrix map for free
    /// blocks, assign all the require metadata and return the ready to use object
    ///
    /// ### Params
    /// @ptr: The pointer to the beginning of the matrix segment \
    /// @size: The total size of the SHM allocation
    ///
    /// ### Returns
    /// A static, lifetime specified, reference to the matrix struct.
    pub fn init(ptr: *mut AtomicMatrix, size: u32) -> &'static mut Self {
        let matrix = unsafe { &mut *ptr };
        matrix.fl_bitmap.store(0, Ordering::Release);
        for i in 0..32 {
            matrix.sl_bitmaps[i].store(0, Ordering::Release);
            for j in 0..8 {
                matrix.matrix[i][j].store(0, Ordering::Release);
            }
        }
        matrix.total_size = size;
        matrix
    }

    /// The entry point of the matrix struct.
    ///
    /// It initializes SHM segment, bind to it, executes the initial formatting,
    /// prepares both the matrix and handler structs and return the High-Level API
    /// to the caller.
    ///
    /// ### Params:
    /// @id: The ID of a new or existing matrix. If one is provided, will skip
    /// formatting and just bind to it \
    /// @size: The SHM allocation size
    ///
    /// ### Returns
    /// The matrix handler api, or a [`MatrixErrors`]
    pub fn bootstrap(id: Option<String>, size: usize) -> Result<MatrixHandler, MatrixErrors> {
        let mut retry_count = 0;

        let path_id = id.unwrap_or_else(generate_uuid);
        let path = format!("/dev/shm/matrix-{}", path_id);
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(&path)
            .map_err(|e| MatrixErrors::MatrixInitializationError {
                reason: format!("Failed to create SHM file: {}", e),
            })?;

        let total_size = size + helpers::METADATA_SPACE as usize;

        file.set_len(total_size as u64).map_err(|e| MatrixErrors::MatrixInitializationError {
            reason: format!("Failed to set matrix physical length: {}", e),
        })?;

        let mut mmap = unsafe {
            MmapMut::map_mut(&file).map_err(|e| MatrixErrors::MatrixInitializationError {
                reason: format!("Failed to create vmem map from the SHM file: {}", e),
            })?
        };

        let base_ptr = mmap.as_mut_ptr();

        let init_guard = unsafe { &*(base_ptr as *const AtomicU32) };
        let matrix_ptr = unsafe { base_ptr.add(16) as *mut AtomicMatrix };
        let current_offset: u32 = helpers::METADATA_SPACE;

        if init_guard
            .compare_exchange(
                helpers::SYS_UNINITIALIZED,
                helpers::SYS_FORMATTING,
                Ordering::SeqCst,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            let matrix = AtomicMatrix::init(matrix_ptr, total_size as u32);

            let remaining_size = total_size - (current_offset as usize);

            let (fl, sl) = helpers::find_indices(remaining_size as u32);

            let header: &mut BlockHeader;
            unsafe {
                header = &mut *(base_ptr.add(current_offset as usize) as *mut BlockHeader);
            }

            header.signature = helpers::HEADER_SIGNATURE;
            header.size.store(remaining_size as u32, Ordering::Release);
            header.state.store(helpers::STATE_FREE, Ordering::Release);
            header.prev_phys.store(0, Ordering::Release);
            header.next_free.store(0, Ordering::Release);
            header.created_at.set_now();

            matrix.insert_free_block(base_ptr, current_offset, fl, sl);
            matrix
                .sector_boundaries
                .store(size as u32, Ordering::Release);

            init_guard.store(helpers::SYS_READY, Ordering::SeqCst);
        } else {
            while init_guard.load(Ordering::Acquire) != helpers::SYS_READY {
                if retry_count >= 512 {
                    return Err(MatrixErrors::MatrixAttachingError);
                } else {
                    retry_count += 1;
                    std::hint::spin_loop();
                }
            }
        }

        Ok(MatrixHandler::new(
            unsafe { &mut *matrix_ptr },
            mmap,
            current_offset,
            path_id,
        ))
    }

    /// Allocates a block in the matrix for the caller
    ///
    /// It acts as a greed allocator, ensuring each call will either get a block allocated
    /// in the matrix, or it throws a OOM Contention flag. It achieves this by politely
    /// trying to claim a block for itself. In case the CAS loop fails, it will simply jump
    /// to the next free block on the chain, granting a lock-free allocation paradigm.
    ///
    /// Each allocation is allowed to retry itself 512 times to confirm the matrix is
    /// indeed out of memory before killing the execution of the function.
    ///
    /// ### Params:
    /// @base_ptr: The starting offset of the SHM mapping. \
    /// @size: The allocation size of the block
    ///
    /// ### Returns:
    /// Either the relative pointer to the allocated block, or a [`MatrixErrors`].
    pub fn allocate(
        &self,
        base_ptr: *const u8,
        size: u32,
    ) -> Result<RelativePtr<u8>, MatrixErrors> {
        let size = (size + 15) & !15;
        let size = size.max(32);
        let (fl, sl) = helpers::find_indices(size);

        for _ in 0..512 {
            if let Some((f_fl, f_sl)) = self.find_suitable_block(fl, sl) {
                if let Ok(offset) = self.remove_free_block(base_ptr, f_fl, f_sl) {
                    let header =
                        unsafe { &mut *(base_ptr.add(offset as usize) as *mut BlockHeader) };
                    header
                        .state
                        .store(helpers::STATE_ALLOCATED, Ordering::Release);

                    let total_size = header.size.load(Ordering::Acquire);

                    if total_size >= size + helpers::HEADER_SPACE {
                        let next_off = offset + size + helpers::HEADER_SPACE;
                        let eos = self.sector_end_offset(next_off);

                        let rem_size = total_size - (size + helpers::HEADER_SPACE);

                        if next_off < eos {
                            let next_h = unsafe {
                                &mut *(base_ptr.add(next_off as usize) as *mut BlockHeader)
                            };

                            next_h.signature = helpers::HEADER_SIGNATURE;
                            next_h.size.store(rem_size, Ordering::Release);
                            next_h.state.store(helpers::STATE_FREE, Ordering::Release);
                            next_h.prev_phys.store(offset, Ordering::Release);
                            next_h.next_free.store(0, Ordering::Release);
                            next_h.created_at.set_now();
                        }

                        header
                            .size
                            .store(size + helpers::HEADER_SPACE, Ordering::Release);
                        fence(Ordering::SeqCst);

                        let (r_fl, r_sl) = helpers::find_indices(rem_size);
                        self.insert_free_block(base_ptr, next_off, r_fl, r_sl);
                    }

                    return Ok(RelativePtr::new(offset + helpers::HEADER_SPACE));
                }
            }
            std::hint::spin_loop();
        }
        Err(MatrixErrors::OutOfMemory)
    }

    /// Acknowledges the freedom of a block and pushes it to be coalesced.
    ///
    /// ### Params:
    /// @ptr: The relative pointer of the block to acknowledge \
    /// @base_ptr: The offset from the start of the SHM segment.
    ///
    /// TODO: In this current state, ack allows multiple processes to double-free the same block. It
    /// is ok, since the kinetic coalescing handles double-free gracefully without blowing
    /// everything up, but it needs to be fixed to ensure a correct ownership contract on higher
    /// levels.
    pub fn ack(&self, ptr: &RelativePtr<BlockHeader>, base_ptr: *const u8) {
        let header = unsafe { ptr.resolve_mut(base_ptr) };

        header.state.store(helpers::STATE_ACKED, Ordering::Release);

        self.coalesce(ptr, base_ptr);
    }

    /// Acknowledges the freedom of a block without triggering the coalescing procedure.
    ///
    /// ### SAFETY
    /// This function leaves a fragment in the matrix in hope someone will coleasce this
    /// futher down the operation tree. Therefore, it is unsafe and should be used with
    /// responsibility.
    ///
    /// ### Params:
    /// @ptr: The relative pointer of the block to acknowledge \
    /// @base_ptr: The offset from the start of the SHM segment.
    pub unsafe fn fast_ack(&self, ptr: &RelativePtr<BlockHeader>, base_ptr: *const u8) {
        let header = unsafe { ptr.resolve_mut(base_ptr) };

        header.state.store(helpers::STATE_ACKED, Ordering::Release);
    }

    /// Tries to merge neighbouring blocks to the left until the end of the matrix is
    /// reached or the neighbour block is not ACKED/FREE.
    ///
    /// This is the elegant implementation of the Kinetic Coalescence processes. It
    /// receives the initial block that will start the ripple, and traverse the matrix
    /// to the left (monotonicity guard). If any race conditions are met in the middle
    /// (another coalescing just start, or a module just claimed this block), it will
    /// stop the coalescing and move on (permissive healing).
    ///
    /// Then it tries to update the next neighbour previous physical offset metadata to
    /// the start of the new free block. If this exchange fails due to end of sector, or
    /// just claimed blocks, it will skip this marking in hopes that when this block is
    /// eventually coalesced, it will passivelly merge backwards with the ripple and fix
    /// the marking on its header by himself (horizon boundary).
    ///
    /// This three core implementations together composes the Propagation Principle of
    /// Atomic Coalescence and enables the matrix to have such high throughput speeds.
    ///
    /// ### Params:
    /// @ptr: The relative pointer of the block to coalesce. \
    /// @base_ptr: The offset from the start of the SHM segment.
    ///
    /// ### Throws:
    /// TidalRippleContentionError: Two coalescing ripples executing simultaneously on
    /// the same blocks.
    pub fn coalesce(&self, ptr: &RelativePtr<BlockHeader>, base_ptr: *const u8) {
        let current_offset = ptr.offset();
        let mut current_header = unsafe { ptr.resolve_mut(base_ptr) };
        let mut total_size = current_header.size.swap(0, Ordering::AcqRel);
        if total_size < HEADER_SPACE {
            return;
        }

        let mut final_offset = current_offset;

        while current_offset > helpers::METADATA_SPACE as u32 {
            let prev_phys_offset = current_header.prev_phys.load(Ordering::Acquire);

            if prev_phys_offset == 0 {
                break;
            }

            let prev_header_ptr =
                unsafe { base_ptr.add(prev_phys_offset as usize) as *mut BlockHeader };
            let prev_header = unsafe { &mut *prev_header_ptr };

            let claimed = prev_header
                .state
                .swap_if_any(
                    &[STATE_ACKED, STATE_FREE],
                    STATE_COALESCING,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                )
                .is_ok();

            if claimed {
                let size_to_add = prev_header.size.swap(0, Ordering::Acquire);
                if size_to_add == 0 || prev_phys_offset >= current_offset {
                    break;
                }

                if size_to_add > self.total_size {
                    break;
                }

                let (fl, sl) = helpers::find_indices(size_to_add);

                total_size = total_size
                    .checked_add(size_to_add)
                    .expect("TidalRippleCoalescingError");
                final_offset = prev_phys_offset;
                current_header = prev_header;
                let _ = self.remove_free_block(base_ptr, fl, sl).is_ok();
            } else {
                break;
            }
        }

        let sector_limit = self.sector_end_offset(final_offset);
        if let Some(next_h_offset) = final_offset.checked_add(total_size) {
            if next_h_offset < sector_limit {
                let next_h =
                    unsafe { &*(base_ptr.add(next_h_offset as usize) as *const BlockHeader) };
                next_h.prev_phys.store(final_offset, Ordering::Release);
            }
        }

        current_header.size.store(total_size, Ordering::Release);
        current_header
            .state
            .store(helpers::STATE_FREE, Ordering::Release);
        current_header.created_at.set_now();

        let (fl, sl) = helpers::find_indices(total_size);
        self.insert_free_block(base_ptr, final_offset, fl, sl);
    }

    /// Queries a block offset inside of the matrix.
    ///
    /// This function does not provide any boundary checks, nor validate if the offset provided is,
    /// in fact, a block. Whatever you pass in, we return a [`RelativePtr`] for it.
    ///
    /// ### Params:
    /// @offset: The offset of the block to be queried
    ///
    /// ### Returns:
    /// The Relative Pointer to the queried block
    pub unsafe fn query(&self, offset: u32) -> RelativePtr<u8> {
        RelativePtr::new(offset + helpers::HEADER_SPACE)
    }

    /// Tries to query a block offset inside the matrix with safety checks.
    ///
    /// It first checks if the requested offset is within the matrix dataplane bounds (before the
    /// last offset, and after the matrix metadata section). Then it checks if the offset actually
    /// has a header and if that header isn't in any of the controlled life cycle states. If all
    /// the checks pass, the raw [`RelativePtr`] to the section is returned.
    ///
    /// ### Params:
    /// @base_ptr: The offset to the beginning of the SHM segment. \
    /// @offset: The offset of the block to be queried.
    ///
    /// ### Returns:
    /// A result containing either the [`RelativePtr`], or a [`MatrixErrors`].
    pub fn checked_query(
        &self,
        base_ptr: *const u8,
        offset: u32,
    ) -> Result<RelativePtr<u8>, MatrixErrors> {
        if offset % 16 != 0 {
            return Err(MatrixErrors::MisalignedHeader);
        }

        if offset >= self.sector_boundaries.load(Ordering::Relaxed) || offset < METADATA_SPACE {
            return Err(MatrixErrors::OutOfBounds);
        }

        let ptr = RelativePtr::<u8>::new(offset + HEADER_SPACE);
        let header = unsafe { ptr.resolve_header(base_ptr) };

        if header
            .state
            .load_if_not_any(&[STATE_COALESCING, STATE_ACKED, STATE_FREE], Ordering::Acquire)
            .is_err()
            || header.signature != helpers::HEADER_SIGNATURE
        {
            return Err(MatrixErrors::InvalidBlock);
        }

        Ok(ptr)
    }

    /// Queries the TLSF bitmaps in search of a block.
    ///
    /// It acquires the first most suitable index flag (according to the find
    /// _indices function) and does a bitwise operation to check if it possesses an
    /// available block. If it matches, return the coordinates of the FL and the
    /// CTZ result from the SL. If it doesn't match, performs CTZ on the first level
    /// to return the first available coordinate.
    ///
    /// ### Params:
    /// @fl: Calculated first level coordinate \
    /// @sl: Calculated second level coordinate
    ///
    /// ### Returns:
    /// A tuple containing the FL/SL coordinates or nothing if there is no space
    /// available in the matrix.
    pub fn find_suitable_block(&self, fl: u32, sl: u32) -> Option<(u32, u32)> {
        let sl_map = self.sl_bitmaps[fl as usize].load(Ordering::Acquire);
        let m_sl = sl_map & (!0u32 << sl);
        if m_sl != 0 {
            return Some((fl, m_sl.trailing_zeros()));
        }

        let fl_map = self.fl_bitmap.load(Ordering::Acquire);
        let m_fl = fl_map & (!0u32 << (fl + 1));
        if m_fl != 0 {
            let f_fl = m_fl.trailing_zeros();
            if f_fl < 32 {
                let s_map = self.sl_bitmaps[f_fl as usize].load(Ordering::Acquire);
                if s_map != 0 {
                    return Some((f_fl, s_map.trailing_zeros()));
                }
            }
        }
        None
    }

    /// Pops a free block from the TLSF bitmap.
    ///
    /// It tries atomically claims ownership over the header inside the map. If
    /// successful, swap the current head to next free head in the chain, or 0 if
    /// there is none. If it fails, it automatically assumes someone claimed the
    /// buffer first and calls a hint::spin loop instruction to retry claiming a
    /// head. If, in one of the interactions, the bucket returs 0, it breaks the
    /// function with an error.
    ///
    /// ### Params:
    /// @base_ptr: The offset from the start of the SHM segment \
    /// @fl: First level coordinates of the bucket \
    /// @sl: Second level coordinates of the head.
    ///
    /// ### Returns
    /// A result containing either the head of the newly acquired block, or an
    /// EmptyBitmapError
    pub fn remove_free_block(
        &self,
        base_ptr: *const u8,
        fl: u32,
        sl: u32,
    ) -> Result<u32, MatrixErrors> {
        let head = &self.matrix[fl as usize][sl as usize];
        loop {
            let off = head.load(Ordering::Acquire);
            if off == 0 {
                return Err(MatrixErrors::EmptyBitmapError);
            }
            let next = unsafe {
                (*(base_ptr.add(off as usize) as *const BlockHeader))
                    .next_free
                    .load(Ordering::Acquire)
            };
            if head
                .compare_exchange(off, next, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                if next == 0 {
                    self.sl_bitmaps[fl as usize].fetch_and(!(1 << sl), Ordering::Release);
                    if self.sl_bitmaps[fl as usize].load(Ordering::Acquire) == 0 {
                        self.fl_bitmap.fetch_and(!(1 << fl), Ordering::Release);
                    }
                }
                return Ok(off);
            }
            std::hint::spin_loop();
        }
    }

    /// Stores a new header inside a bucket
    ///
    /// It does the exact oposite of the remove_free_block basically.
    ///
    /// ### Params:
    /// @base_ptr: The offset from the beginning of the SHM segment \
    /// @offset: The header offset to be inserted into the bucket \
    /// @fl: The first level insertion coordinates \
    /// @sl: The second level insertion coordinates
    pub fn insert_free_block(&self, base_ptr: *const u8, offset: u32, fl: u32, sl: u32) {
        let head = &self.matrix[fl as usize][sl as usize];
        let h = unsafe { &mut *(base_ptr.add(offset as usize) as *mut BlockHeader) };
        loop {
            let old = head.load(Ordering::Acquire);
            h.next_free.store(old, Ordering::Release);
            if head
                .compare_exchange(old, offset, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                break;
            }
            std::hint::spin_loop();
        }
        self.fl_bitmap.fetch_or(1 << fl, Ordering::Release);
        self.sl_bitmaps[fl as usize].fetch_or(1 << sl, Ordering::Release);
    }

    /// Returns the boundary of the current sector
    ///
    /// It queries the boundaries from the metadata and check wheter the block fits or
    /// not inside this sector.
    ///
    /// ### Params:
    /// @current_offset: The offset to check against the sector.
    ///
    /// ### Returns:
    /// Either the boundary value of the current sector, or the end of the segment.
    pub fn sector_end_offset(&self, current_offset: u32) -> u32 {
        let boundary = self.sector_boundaries.load(Ordering::Acquire);
        if current_offset < boundary {
            return boundary;
        }

        self.total_size as u32
    }
}

/// # SAFETY
/// most functions inside of the Relative Pointer relies on math assuming that any
/// provided address is valid and functional inside the matrix. Therefore, the
/// caller must ensure that any operations executed are checked and valid before
/// proceeding.
impl<T> RelativePtr<T> {
    /// Creates a new relative pointer based on the provided offset
    ///
    /// This initializes the pointer with the PhantomData ownership over the type
    /// passed in the parameter
    ///
    /// ### Params:
    /// @offset: The offset value to be wrapped in the pointer.
    ///
    /// ### Returns:
    /// An instance of Self.
    pub fn new(offset: u32) -> Self {
        Self {
            offset,
            _marker: PhantomData,
        }
    }

    /// Returns the offset value in the pointer
    pub fn offset(&self) -> u32 {
        self.offset
    }

    /// Resolves the header based on the base_ptr of the current caller process.
    ///
    /// This ensures that the pointer returned is actually mapped to the process local
    /// memory scope
    ///
    /// ### Params:
    /// @base_ptr: The offset from the start of the SHM segment.
    ///
    /// ### Returns:
    /// A life time speficied reference to the header of this block
    pub unsafe fn resolve_header<'a>(&self, base_ptr: *const u8) -> &'a BlockHeader {
        unsafe {
            &*(base_ptr.add((self.offset as usize) - helpers::HEADER_SPACE as usize)
                as *mut BlockHeader)
        }
    }

    /// Resolves the header based on the base_ptr of the current caller process.
    ///
    /// This ensures that the pointer returned is actually mapped to the process local
    /// memory scope
    ///
    /// ### Params:
    /// @base_ptr: The offset from the start of the SHM segment.
    ///
    /// ### Returns:
    /// A life time speficied mutable reference to the header of this block
    pub unsafe fn resolve_header_mut<'a>(&self, base_ptr: *const u8) -> &'a mut BlockHeader {
        unsafe {
            &mut *(base_ptr.add((self.offset as usize) - helpers::HEADER_SPACE as usize)
                as *mut BlockHeader)
        }
    }

    /// Resolves the block scope based on the base_ptr of the current caller process.
    ///
    /// This ensures that the pointer returned is actually mapped to the process local
    /// memory scope
    ///
    /// ### Params:
    /// @base_ptr: The offset from the start of the SHM segment.
    ///
    /// ### Returns:
    /// A life time specified reference to the block scope.
    pub unsafe fn resolve<'a>(&self, base_ptr: *const u8) -> &'a T {
        unsafe { &*(base_ptr.add(self.offset as usize) as *mut T) }
    }

    /// Resolves the block scope based on the base_ptr of the current caller process.
    ///
    /// This ensures that the pointer returned is actually mapped to the process local
    /// memory scope
    ///
    /// ### Params:
    /// @base_ptr: The offset from the start of the SHM segment.
    ///
    /// ### Returns:
    /// A life time specified mutable reference to the block scope.
    pub unsafe fn resolve_mut<'a>(&self, base_ptr: *const u8) -> &'a mut T {
        unsafe { &mut *(base_ptr.add(self.offset as usize) as *mut T) }
    }

    /// Writes bytes to the pointer data section.
    ///
    /// ### Params:
    /// @base_ptr: The offset from the start of the SHM segment.
    /// @value: The value to be written into the pointer.
    pub unsafe fn write(&self, base_ptr: *const u8, value: T) {
        unsafe { std::ptr::write(self.resolve_mut(base_ptr), value) }
    }
}

#[cfg(test)]
mod tests {
    use crate::core::handlers::HandlerFunctions;
    use crate::core::matrix::{BlockHeader, RelativePtr};
    use crate::internals::collections::memory_scale;

    use super::*;

    /// Test if the mapping function can return the correct indexes.
    #[test]
    fn test_mapping() {
        assert_eq!(helpers::find_indices(16), (0, 1));
        assert_eq!(helpers::find_indices(64), (0, 4));
        assert_eq!(helpers::find_indices(128), (7, 0));

        let (fl, sl) = helpers::find_indices(1024);
        assert_eq!(fl, 10);
        assert_eq!(sl, 0);
    }

    /// Test if the bootstrap function actually initializes the matrix, and allocates the
    /// blocks on the correct bitmaps.
    #[test]
    fn test_initial_bootstrap() {
        let size = memory_scale::two::MB;
        let handler = AtomicMatrix::bootstrap(Some(String::from("bootstrap_test")), size).unwrap();

        let bitmap = handler.matrix().fl_bitmap.load(Ordering::Acquire);

        assert!(bitmap != 0, "FL bitmap should not be zero after bootstrap");
    }

    /// Test allocation and sppliting logic by comparing the size of our buffer.
    #[test]
    fn test_allocation_and_spliting() {
        let size = memory_scale::two::MB;
        let handler = AtomicMatrix::bootstrap(Some(String::from("a_s_test")), size).unwrap();
        let base_ptr = handler.base_ptr();
        let matrix = handler.matrix();

        unsafe {
            let rel_ptr = matrix.allocate(base_ptr, 64).unwrap();

            let header = rel_ptr.resolve_header(base_ptr);

            assert_eq!(
                header.size.load(Ordering::Acquire),
                64 + helpers::HEADER_SPACE
            );
            assert_eq!(
                header.state.load(Ordering::Acquire),
                helpers::STATE_ALLOCATED
            );
        }

        unsafe { handler.die().unwrap() };
    }

    /// Test coalesce logic to see if blocks will merge correctly.
    #[test]
    fn test_ack_and_coalesce() {
        let size = memory_scale::two::MB;
        let handler = AtomicMatrix::bootstrap(Some(String::from("a_c_test")), size).unwrap();
        let base_ptr = handler.base_ptr();
        let matrix = handler.matrix();

        unsafe {
            let ptr_a = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_b = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_c = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_d = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_e = matrix.allocate(base_ptr, 64).unwrap();

            let h_b = ptr_b.resolve_header(base_ptr);
            let rel_c = RelativePtr::<BlockHeader>::new(ptr_c.offset() - helpers::HEADER_SPACE);
            let rel_d = RelativePtr::<BlockHeader>::new(ptr_d.offset() - helpers::HEADER_SPACE);

            h_b.state.store(helpers::STATE_FREE, Ordering::Release);
            matrix.ack(&rel_c, base_ptr);
            matrix.ack(&rel_d, base_ptr);

            matrix.coalesce(&rel_d, base_ptr);

            let h_a = ptr_a.resolve_header(base_ptr);
            assert_eq!(h_a.state.load(Ordering::Acquire), helpers::STATE_ALLOCATED);

            let h_merged = ptr_b.resolve_header(base_ptr);
            assert_eq!(h_merged.state.load(Ordering::Acquire), helpers::STATE_FREE);
            assert_eq!(
                h_merged.size.load(Ordering::Acquire),
                (64 * 3) + (3 * helpers::HEADER_SPACE)
            );

            let h_e = ptr_e.resolve_header(base_ptr);
            assert_eq!(h_e.state.load(Ordering::Acquire), helpers::STATE_ALLOCATED);
        };

        unsafe { handler.die().unwrap() };
    }

    #[test]
    fn test_allocation_size_plus_md_section() {
        let size = memory_scale::two::KB;
        let handler = AtomicMatrix::bootstrap(None, size).unwrap();
        let init_block = handler.matrix()
            .checked_query(handler.base_ptr() as *const u8, helpers::METADATA_SPACE).unwrap();

        let header = unsafe { init_block.resolve_header(handler.base_ptr()) };

        assert_eq!(header.size.load(Ordering::Relaxed), size as u32);

        unsafe { handler.die().unwrap() }
    }

    /// ----------------------------------------------------------------------------
    /// STRESS TESTS
    ///
    /// These are all ignored from the correctness suite so github doesn't get mad at
    /// me. Before shipping, running these are explicitly required.
    /// ----------------------------------------------------------------------------

    /// Run 800.000 allocations in parallel (100.000 each) to test if the matrix
    /// can hold without race conditions.
    #[test]
    #[ignore]
    fn test_multithreaded_stress() {
        // Quick author note:
        //
        // May God help us all at this moment.

        use std::collections::HashSet;
        use std::sync::{Arc, Barrier};
        use std::thread;

        // We use 62MB matrix to allocate all the buffers
        let size = memory_scale::custom::mb::<100>();
        let handler = AtomicMatrix::bootstrap(None, size).unwrap();

        let thread_count = 8;
        let allocs_per_second = 100_000;
        let barrier = Arc::new(Barrier::new(thread_count));

        // Track the failed allocs
        let fail_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));

        let mut handles = vec![];
        static USED_SIZE: AtomicU32 = AtomicU32::new(0);

        // Fuck the matrix! GO GO GO
        for _ in 0..thread_count {
            let b = Arc::clone(&barrier);
            let us_ref = &USED_SIZE;
            let s_handler = handler.share();
            let fail_count_clone = Arc::clone(&fail_count);

            handles.push(thread::spawn(move || {
                let mut my_offsets = Vec::new();

                b.wait();

                for _ in 0..allocs_per_second {
                    for _ in 0..10 {
                        if let Ok(rel_ptr) = s_handler.matrix().allocate(s_handler.base_ptr(), 64) {
                            let h = unsafe { rel_ptr.resolve_header(s_handler.base_ptr()) };
                            us_ref.fetch_add(h.size.load(Ordering::Relaxed), Ordering::Release);

                            my_offsets.push(rel_ptr.offset());
                            break;
                        }
                        fail_count_clone.fetch_add(1, Ordering::Relaxed);
                        std::hint::spin_loop();
                    }
                }

                my_offsets
            }));
        }

        // Collect everything we did and check
        let mut all_offsets = Vec::new();
        for h in handles {
            all_offsets.extend(h.join().unwrap());
        }

        let total_obtained = all_offsets.len();
        let unique_offsets: HashSet<_> = all_offsets.into_iter().collect();

        // We allow for a 0.5% failure marging, as this stress test does not account for deallocations.
        let success_percentage = ((thread_count * allocs_per_second) as f64) * 0.995;
        let used_size = USED_SIZE.load(Ordering::Relaxed) / 1024 / 1024;

        // Assert we can obtain at least 99.5% of the expected allocations without collisions, which would
        // indicate a potential race condition.
        println!("{} MB used", used_size);
        println!("Obtained: {}", total_obtained);
        println!("Threshold: {}", success_percentage);
        assert!(
            total_obtained >= (success_percentage as usize),
            "Total allocations should match expected count"
        );
        assert_eq!(
            unique_offsets.len(),
            total_obtained,
            "RACE CONDITION DETECTED: Duplicate offsets found"
        );
        println!(
            "Successfully allocated {} unique blocks across {} threads without collisions! {} allocations failed",
            total_obtained,
            thread_count,
            fail_count.load(Ordering::Relaxed)
        );

        unsafe { handler.die().unwrap() };
    }

    /// Test if the matrix can hold 10 minutes of 8 threads executing random alloc
    /// and dealloc operations to ensure the Propagation Principle of Atomic
    /// Coalescence works.
    #[test]
    #[ignore]
    fn test_long_term_fragmentation_healing() {
        // Quick author note:
        //
        // May god help your device at this moment

        use std::sync::{Arc, Barrier};
        use std::thread;
        use std::time::{Duration, Instant};

        const DURATION: u32 = 600;
        const THREADS: u32 = 8;

        let size = memory_scale::custom::mb::<20>();
        let handler = AtomicMatrix::bootstrap(Some(String::from("f_h_test")), size).unwrap();
        let handler_arc = Arc::new(handler);
        let barrier = Arc::new(Barrier::new(THREADS as usize));
        let start_time = Instant::now();
        let duration = Duration::from_secs(DURATION as u64);

        let mut handles = vec![];

        for t_id in 0..THREADS {
            let h = Arc::clone(&handler_arc);
            let b = Arc::clone(&barrier);

            handles.push(thread::spawn(move || {
                let base_ptr = h.base_ptr() as *mut u8;
                let matrix = &h.matrix();
                let mut my_blocks = Vec::new();
                let mut rng = t_id + 1;
                let mut total_ops = 0u64;

                b.wait();

                while start_time.elapsed() < duration {
                    rng = rng.wrapping_mul(1103515245).wrapping_add(12345);

                    if rng % 10 < 7 && my_blocks.len() < 200 {
                        let alloc_size = rng % 512;
                        if let Ok(ptr) = matrix.allocate(base_ptr, alloc_size as u32) {
                            my_blocks.push(ptr);
                        }
                    } else if !my_blocks.is_empty() {
                        let idx = (rng as usize) % my_blocks.len();
                        let ptr = my_blocks.swap_remove(idx);

                        let header_ptr =
                            RelativePtr::<BlockHeader>::new(ptr.offset() - helpers::HEADER_SPACE);
                        matrix.ack(&header_ptr, base_ptr);

                        if total_ops % 5 == 0 {
                            matrix.coalesce(&header_ptr, base_ptr);
                        }
                    }
                    total_ops += 1;
                }
                (my_blocks, total_ops)
            }));
        }

        let mut total_work = 0u64;
        for h in handles {
            let (remaining, thread_ops) = h.join().unwrap();
            total_work += thread_ops;
            let base_ptr = handler_arc.base_ptr() as *mut u8;

            for ptr in remaining {
                let header_ptr =
                    RelativePtr::<BlockHeader>::new(ptr.offset() - helpers::HEADER_SPACE);
                handler_arc.matrix().ack(&header_ptr, base_ptr);
                handler_arc.matrix().coalesce(&header_ptr, base_ptr);
            }
        }

        let mut free_count = 0;
        for fl in 0..32 {
            for sl in 0..8 {
                if handler_arc.matrix().matrix[fl][sl].load(Ordering::Acquire) != 0 {
                    free_count += 1;
                }
            }
        }

        let entrophy_percentage = ((free_count as f64) / (total_work as f64)) * 100.0;
        let mops = (total_work as f64) / (DURATION as f64) / 1_000_000.0;

        println!("Endurance Results (Duration of {} seconds)", DURATION);
        println!("Num of threads: {}", THREADS);
        println!("Total operations: {}", total_work);
        println!(
            "Throughput: {:.2} Mop/s",
            (total_work as f64) / (DURATION as f64) / 1_000_000.0
        );
        println!("Final free fragments: {}", free_count);
        println!("Entrophy percentage: {}%", entrophy_percentage);

        assert!(entrophy_percentage < 0.001, "Excessive Fragmentation");
        assert!(mops > 1.0, "Throughput regression: {:.2} Mop/s", mops);

        unsafe { handler_arc.die().unwrap() };
    }
}
