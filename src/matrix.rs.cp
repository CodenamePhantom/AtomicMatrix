use std::sync::atomic::{ AtomicU32, AtomicU64, Ordering, fence };
use std::marker::PhantomData;
use std::fs::OpenOptions;
use memmap2::MmapMut;

use crate::handlers::matrix_handler::MatrixHandler;

const SYS_UNINITIALIZED: u32 = 0;
const SYS_FORMATTING: u32 = 1;
const SYS_READY: u32 = 2;

const STATE_FREE: u32 = 0;
const STATE_ALLOCATED: u32 = 1;
const _STATE_READY: u32 = 2;
const STATE_ACKED: u32 = 3;
const STATE_COALESCING: u32 = 4;
const _STATE_CACHED: u32 = 6;

pub mod core {
    use super::*;
    use uuid::Uuid;

    /// Header prepended to every memory block within the SHM pool.
    ///
    /// The 'BlockHeader' is the core of the TLSF (Two-Level Segregated Fit) mechanism.
    /// It maintains the metadata required to manage a block's lifecycle and its position
    /// in the segregated free lists
    #[repr(C, align(16))]
    pub struct BlockHeader {
        /// The total size of the block (including this header) in bytes.
        pub size: AtomicU32,
        /// The state of this current buffer (FREE, ALLOCATED, READY, ACKED, COALESING).
        /// Used as an atomic guard to prevent race conditions between modules.
        pub state: AtomicU32,
        /// Offset to the start of the physically adjacent preceding block in memory.
        /// Essential for O(1) coalescing of adjacent free blocks
        pub prev_phys: AtomicU32,
        /// Offset to the next block in the current TLSF segregated free list.
        pub next_free: AtomicU32,
        /// Offset to the previous block in the current TLSF segregated free list.
        pub prev_free: AtomicU32,
    }

    #[repr(C, align(16))]
    pub struct StreamHeader {
        pub size: AtomicU32,
        pub state: AtomicU32,
        pub stream_key: AtomicU64,
    }

    /// The Shared Memory Orchestrator.
    ///
    /// This structure resides at the fixed offset (16 bytes) of the SHM segment.
    /// It manages the bitmapped indices used to find the free memory blocks in
    /// O(1) time.
    #[repr(C)]
    pub struct AtomicMatrix {
        /// Public identifier for the SHM segment, stored as a UUID. This allows modules to
        /// reference the same segment across different processes by using the same identifier.
        pub id: Uuid,
        /// First-Level Bitmap: Each bit represents a power-of-two size class
        /// (e.g., bit 10 = 1KB range).
        pub fl_bitmap: AtomicU32,
        /// Second-Level Bitmaps: Each entry sub-divides the FL power-of-two range
        /// into 8 linear bins.
        pub sl_bitmaps: [AtomicU32; 32],
        /// The head pointers for the free lists. Each entry is an offset to a
        /// 'BlockHeader'. Accessed via 'matrix[fl][sl]'.
        pub matrix: [[AtomicU32; 8]; 32],
    }

    /// A process-agnostic pointer.
    ///
    /// Since Shared Memory can be mapped to different virtual addresses in different
    /// processes, we cannot store raw pointer (*mut T). Instead, we store a 'u32'
    /// offset from the base of the SHM segment.
    pub struct RelativePtr<T> {
        /// Byte offset from the SHM base segment.
        offset: u32,
        /// Tells Rust that this struct logically "owns" or carries a T, ensuring type
        /// safety during resolution without storing a real pointer.
        _marker: PhantomData<T>,
    }

    /// Utility for calculating TLSF coordinates.
    pub struct Mapping;

    impl AtomicMatrix {
        /// Initiates the matrix metadata in a raw memory location.
        ///
        /// This should only be called once per SHM lifecycle (usually by the Master Thread).
        /// It resets all bitmaps and free-list heads to zero (empty).
        pub fn init(ptr: *mut AtomicMatrix, id: Uuid) -> &'static mut Self {
            unsafe {
                let matrix = &mut *ptr;
                matrix.fl_bitmap.store(0, Ordering::Relaxed);

                for i in 0..32 {
                    matrix.sl_bitmaps[i].store(0, Ordering::Relaxed);

                    for j in 0..8 {
                        matrix.matrix[i][j].store(0, Ordering::Relaxed);
                    }
                }

                matrix.id = id;

                matrix
            }
        }

        /// The primary entry point for a module to attach to the SHM segment.
        ///
        /// This function handles:
        /// 1. Opening/Creating the file in '/dev/shm/'.
        /// 2. Sizing the segment.
        /// 3. Mapping the memory into the current process's address space.
        /// 4. Synchronized initialization via an atomic 'init_guard'.
        ///
        /// Returns a 'MatrixHandler' which acts as the High-Level API for the module.
        pub fn bootstrap(id: Option<Uuid>, size: usize) -> Result<MatrixHandler, String> {
            let path_id = id.unwrap_or_else(Uuid::new_v4);
            let path = format!("/dev/shm/{}", path_id);
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .open(&path)
                .map_err(|e| e.to_string())?;

            file.set_len(size as u64).map_err(|e| e.to_string())?;

            let mut mmap = unsafe { MmapMut::map_mut(&file).map_err(|e| e.to_string())? };
            let base_ptr = mmap.as_mut_ptr();

            let init_guard = unsafe { &*(base_ptr as *const AtomicU32) };
            let matrix_ptr = unsafe { base_ptr.add(16) as *mut AtomicMatrix };

            if
                init_guard
                    .compare_exchange(
                        SYS_UNINITIALIZED,
                        SYS_FORMATTING,
                        Ordering::SeqCst,
                        Ordering::Relaxed
                    )
                    .is_ok()
            {
                let matrix = AtomicMatrix::init(matrix_ptr, path_id);

                matrix.sectorize(base_ptr, size, 100, 0).unwrap();
                init_guard.store(SYS_READY, Ordering::SeqCst);
            } else {
                while init_guard.load(Ordering::Acquire) != SYS_READY {
                    std::hint::spin_loop();
                }
            }

            let matrix_ref = unsafe { &mut *matrix_ptr };

            Ok(MatrixHandler {
                matrix: matrix_ref,
                mmap,
            })
        }

        /// A bootstrap utility that segrates the SHM segment into three classes:
        ///
        /// 1. **Small Volume (e.g. 1MB):** For small messages, using a single FL=0 with 16-byte SL subdivisions.
        /// 2. **Medium Volume (e.g. 10MB):** For medium messages, using FL=1 to FL=10 with 8 SL subdivisions each.
        /// 3. **Large Volume (e.g. 100MB):** For large messages, using FL=11 to FL=31 with 8 SL subdivisions each.
        ///
        /// This function is a convenience for typical use cases, but the 'bootstrap' method needs to stablish the
        /// total size of the SHM segment on which we will calculate the appropriate sizes for each class.
        ///
        /// Returns a Result indicating success or failure of the sectorization process.
        pub fn sectorize(
            &self,
            base_ptr: *mut u8,
            total_file_size: usize,
            mut small_percent: u8,
            mut medium_percent: u8
        ) -> Result<(), String> {
            let matrix_size = std::mem::size_of::<AtomicMatrix>();
            let mut current_offset = 16 + (matrix_size as u32);
            current_offset = (current_offset + 15) & !15;

            let usable_space = (total_file_size as u32).saturating_sub(current_offset);

            if usable_space < 256 * 1024 {
                let (fl, sl) = Mapping::find_indices(usable_space);
                self.create_and_insert_sector(base_ptr, current_offset, usable_space, 0, fl, sl);
                return Ok(());
            }

            if total_file_size < 5 * 1024 * 1024 {
                small_percent = 30;
                medium_percent = 70;
            }

            let small_size = (((usable_space as u64) * (small_percent as u64)) / 100) as u32;
            let small_size = (small_size + 15) & !15;

            let medium_size = (((usable_space as u64) * (medium_percent as u64)) / 100) as u32;
            let medium_size = (medium_size + 15) & !15;

            let large_size = usable_space.saturating_sub(small_size).saturating_sub(medium_size);

            let mut prev_phys_offset = 0u32;

            if small_size > 0 {
                let (fl, sl) = Mapping::find_indices(small_size);
                self.create_and_insert_sector(
                    base_ptr,
                    current_offset,
                    small_size,
                    prev_phys_offset,
                    fl,
                    sl
                );
                prev_phys_offset = current_offset;
                current_offset += small_size;
            }

            if medium_size > 0 {
                let (fl, sl) = Mapping::find_indices(medium_size);
                self.create_and_insert_sector(
                    base_ptr,
                    current_offset,
                    medium_size,
                    prev_phys_offset,
                    fl,
                    sl
                );
                prev_phys_offset = current_offset;
                current_offset += medium_size;
            }

            if large_size > 16 {
                let (fl, sl) = Mapping::find_indices(large_size);
                self.create_and_insert_sector(
                    base_ptr,
                    current_offset,
                    large_size,
                    prev_phys_offset,
                    fl,
                    sl
                );
            }

            Ok(())
        }

        pub fn allocate(&self, base_ptr: *mut u8, size: u32) -> Result<RelativePtr<u8>, String> {
            // 1. Align size to 16 bytes and enforce minimum block size (Header + small payload)
            let size = (size + 15) & !15;
            let size = size.max(32);
            let (fl, sl) = Mapping::find_indices(size);

            // 2. High-Contention Retry Loop
            // In lock-free systems, a bitmap might say "Ready" while the list-head swap
            // is still in flight. We spin briefly to resolve these nanosecond races.
            for _ in 0..128 {
                if let Some((found_fl, found_sl)) = self.find_suitable_block(fl, sl) {
                    match self.remove_free_block(base_ptr, found_fl, found_sl) {
                        Ok(block_offset) => {
                            // --- SUCCESS: We claimed a block ---
                            unsafe {
                                let header = &mut *(
                                    base_ptr.add(block_offset as usize) as *mut BlockHeader
                                );
                                let total_size = header.size.load(Ordering::Acquire);

                                // 3. O(1) Splitting logic
                                if total_size >= size + 32 {
                                    let remaining_size = total_size - size;
                                    let next_offset = block_offset + size;

                                    // Initialize the remainder block header
                                    let next_header = &mut *(
                                        base_ptr.add(next_offset as usize) as *mut BlockHeader
                                    );
                                    next_header.size.store(remaining_size, Ordering::Release);
                                    next_header.state.store(STATE_FREE, Ordering::Release);
                                    next_header.prev_phys.store(block_offset, Ordering::Release);
                                    next_header.next_free.store(0, Ordering::Release);
                                    next_header.prev_free.store(0, Ordering::Release);

                                    // Update current block size
                                    header.size.store(size, Ordering::Release);

                                    // Ensure header writes are visible to all cores before insertion
                                    std::sync::atomic::fence(Ordering::SeqCst);

                                    let (rem_fl, rem_sl) = Mapping::find_indices(remaining_size);
                                    self.insert_free_block(base_ptr, next_offset, rem_fl, rem_sl);
                                }

                                header.state.store(STATE_ALLOCATED, Ordering::Release);
                                return Ok(RelativePtr::new(block_offset + 32));
                            }
                        }
                        Err(_) => {
                            // Bit was set but list was momentarily empty (Contention)
                            std::hint::spin_loop();
                            continue;
                        }
                    }
                } else {
                    // Truly exhausted the pool (no bits set in bitmap)
                    break;
                }
            }

            Err("Out of memory: High contention or exhausted pool".to_string())
        }

        /// Marks a block as ACKED, indicating that the consumer has processed the message and the block can be safely
        /// freed or reused. This typically involver:
        ///
        /// 1. Atomically changing the block's state to ACKED.
        /// 2. Add it to the to-be-freed list corresponding to its FL/SL class.
        ///
        /// Blocks in the ACKED stated are not immediately freed to avoid high fragmentation of memory segments inside
        /// the matrix. Instead, they are added to a "to-be-freed" list and only coalesced and returned to the free
        /// pool when the master/delegate thread performs a cleanup cycle (based on its on internal criteria on what and
        /// when).
        pub fn ack(&self, ptr: RelativePtr<BlockHeader>, base_ptr: *mut u8) {
            unsafe {
                let header = ptr.resolve_mut(base_ptr);

                header.state.store(STATE_ACKED, Ordering::Release);
            }
        }

        /// Coalesces adjacent free blocks in the same FL/SL class to reduce fragmentation and improve allocation success
        /// rates. This function is typically called during the cleanup cycle of the master/delegate thread and
        /// involves:
        ///
        /// 1. Iterating through the "to-be-freed" list for each FL/SL class.
        /// 2. For each block, checking its physical neighbors using the 'prev_phys' offset and the size of the
        /// current block to find the next physical block.
        /// 3. If the neighboring block is also free, atomically change its state to COALESCING to prevent other
        /// threads from modifying it during the coalescing process.
        /// 4. Jump the chain until no more adjacent free blocks are found, calculating the total size of the
        /// coalesced block.
        /// 5. Update the bitmaps and free list to reflect the new coalesced block.
        ///
        /// This O(1) coalescing mechanism is a key advantage of using relative pointers and the 'prev_phys' offset,
        /// as it allows us to quickly navigate through adjacent blocks without needing to traverse the free list or
        /// perform complex pointer arithmetic.
        pub fn coalesce(&self, ptr: RelativePtr<BlockHeader>, base_ptr: *mut u8) {
            unsafe {
                let current_offset = ptr.offset();
                let mut current_header = ptr.resolve_mut(base_ptr);
                let mut total_size = current_header.size.load(Ordering::Acquire);

                let mut final_offset = current_offset;

                while current_offset > 16 {
                    let prev_phys_offset = current_header.prev_phys.load(Ordering::Acquire);

                    if prev_phys_offset == 0 {
                        break;
                    }

                    let prev_header_ptr = base_ptr.add(
                        prev_phys_offset as usize
                    ) as *mut BlockHeader;
                    let prev_header = &mut *prev_header_ptr;

                    let res = prev_header.state.compare_exchange(
                        STATE_FREE,
                        STATE_COALESCING,
                        Ordering::Acquire,
                        Ordering::Relaxed
                    );

                    let claimed = if res.is_ok() {
                        true
                    } else {
                        prev_header.state
                            .compare_exchange(
                                STATE_ACKED,
                                STATE_COALESCING,
                                Ordering::Acquire,
                                Ordering::Relaxed
                            )
                            .is_ok()
                    };

                    if claimed {
                        total_size += prev_header.size.load(Ordering::Acquire);
                        final_offset = prev_phys_offset;
                        current_header = prev_header;
                    } else {
                        break;
                    }
                }

                std::ptr::write_bytes(
                    base_ptr.add((final_offset as usize) + 32),
                    0,
                    (total_size - 32) as usize
                );

                current_header.size.store(total_size, Ordering::Release);
                current_header.state.store(STATE_FREE, Ordering::Release);

                let (fl, sl) = Mapping::find_indices(total_size);
                self.insert_free_block(base_ptr, final_offset, fl, sl);
            }
        }

        /// Creates a new free sector in the SHM segment and inserts it into the appropriate FL/SL free list.
        /// This function is used during the initial sectorization of the segment to allocate the initial categories
        /// of blocks.
        ///
        /// The process involvers:
        /// 1. Writing a 'BlockHeader' at the specified offset with the given size and previous physical offset.
        /// 2. Setting the block's state to FREE and initializing its free list pointers to zero.
        /// 3. Updating the FL and SL bitmaps to indicate the presence of a free block in the corresponding class.
        /// 4. Inserting the new block into the head of the appropriate free list in the matrix.
        fn create_and_insert_sector(
            &self,
            base_ptr: *mut u8,
            offset: u32,
            size: u32,
            prev_phys: u32,
            fl: u32,
            sl: u32
        ) {
            let header_ptr = unsafe { base_ptr.add(offset as usize) as *mut BlockHeader };
            let header = unsafe { &mut *header_ptr };

            header.size.store(size, Ordering::Release);
            header.state.store(STATE_FREE, Ordering::Release);
            header.prev_phys.store(prev_phys, Ordering::Release);

            header.next_free.store(0, Ordering::Release);
            header.prev_free.store(0, Ordering::Release);

            self.fl_bitmap.fetch_or(1 << fl, Ordering::Release);
            self.sl_bitmaps[fl as usize].fetch_or(1 << sl, Ordering::Release);

            self.matrix[fl as usize][sl as usize].store(offset, Ordering::Release);
        }

        /// Traverses the FL and SL bitmaps to find a suitable free block for allocation.
        ///
        /// The search process involves:
        /// 1. Checking the SL bitmap for the requested FL to find a suitable block in the same size class.
        /// 2. If no block is found in the same SL, check the FL bitmap to find the next available FL with
        /// free blocks, then check its SL bitmap to find a suitable block.
        /// 3. If a block is found, returns its FL and SL coordinates. If no block is found, returns None.
        fn find_suitable_block(&self, fl: u32, sl: u32) -> Option<(u32, u32)> {
            let sl_map = self.sl_bitmaps[fl as usize].load(Ordering::Acquire);
            let masked_sl = sl_map & (!0u32 << sl);

            if masked_sl != 0 {
                let found_sl = masked_sl.trailing_zeros();
                if found_sl < 8 {
                    return Some((fl, found_sl));
                }
            }

            let fl_map = self.fl_bitmap.load(Ordering::Acquire);
            let masked_fl = fl_map & (!0u32 << (fl + 1));

            if masked_fl != 0 {
                let found_fl = masked_fl.trailing_zeros();
                if found_fl < 32 {
                    let sl_map = self.sl_bitmaps[found_fl as usize].load(Ordering::Acquire);
                    if sl_map != 0 {
                        let found_sl = sl_map.trailing_zeros();
                        return Some((found_fl, found_sl));
                    }
                }
            }

            None
        }

        /// Atomically removes a free block from the specified FL/SL free list and updates the bitmaps accordingly.
        ///
        /// The removal process involves:
        /// 1. Atomically loading the head of the free list for the specified FL/SL.
        /// 2. If the list is empty (offset == 0), return an error indicating that no block is available.
        /// 3. If a block is found, atomically compare-and-swap the head of the list to the next block in the free
        /// list. This ensures that the block is removed from the free list without race conditions.
        /// 4. After successfully removing the block, update the SL bitmap to clear the bit for the SL index.
        /// If the SL bitmap for that FL becomes zero, also clear the corresponding bit in the FL bitmap.
        /// 5. Return the offset of the removed block for allocation.
        fn remove_free_block(&self, base_ptr: *mut u8, fl: u32, sl: u32) -> Result<u32, String> {
            let list_head = &self.matrix[fl as usize][sl as usize];

            loop {
                let offset = list_head.load(Ordering::Acquire);
                if offset == 0 {
                    return Err("Race condition: free list became empty".to_string());
                }

                unsafe {
                    let header = &*(base_ptr.add(offset as usize) as *const BlockHeader);
                    let next_offset = header.next_free.load(Ordering::Acquire);

                    if
                        list_head
                            .compare_exchange(
                                offset,
                                next_offset,
                                Ordering::SeqCst,
                                Ordering::Relaxed
                            )
                            .is_ok()
                    {
                        // ONLY clear the bitmap if the list is now truly empty
                        if next_offset == 0 {
                            self.sl_bitmaps[fl as usize].fetch_and(!(1 << sl), Ordering::Release);

                            // Double check if the whole FL level is now empty
                            if self.sl_bitmaps[fl as usize].load(Ordering::Acquire) == 0 {
                                self.fl_bitmap.fetch_and(!(1 << fl), Ordering::Release);
                            }
                        }
                        return Ok(offset);
                    }
                }
            }
        }

        /// Atomically inserts a free block into the specified FL/SL free list and updates the bitmaps accordingly.
        ///
        /// The insertion process involves:
        /// 1. Writing the block's 'BlockHeader' at the specified offset with the given size and previous physical offset.
        /// 2. Setting the block's state to FREE and initializing its free list pointers to zero.
        /// 3. Atomically inserting the block at the head of the appropriate free list in the matrix. This involves:
        ///   - Loading the current head of the free list for the specified FL/SL.
        ///  - Setting the new block's 'next_free' pointer to the current head.
        /// - If the current head is not zero, updating the old head's 'prev_free' pointer to point back to the new block.
        /// 4. Updating the FL and SL bitmaps to indicate the presence of a free block in the corresponding class.
        fn insert_free_block(&self, base_ptr: *mut u8, offset: u32, fl: u32, sl: u32) {
            let list_head = &self.matrix[fl as usize][sl as usize];

            unsafe {
                let header = &mut *(base_ptr.add(offset as usize) as *mut BlockHeader);

                loop {
                    let old_head = list_head.load(Ordering::Acquire);
                    header.next_free.store(old_head, Ordering::Release);

                    // Only point the list head to us if it hasn't changed since we loaded old_head
                    if
                        list_head
                            .compare_exchange(old_head, offset, Ordering::SeqCst, Ordering::Relaxed)
                            .is_ok()
                    {
                        break;
                    }
                    // If it failed, someone else inserted a block. Retry the link.
                }
            }

            // Turn on the lights
            self.fl_bitmap.fetch_or(1 << fl, Ordering::Release);
            self.sl_bitmaps[fl as usize].fetch_or(1 << sl, Ordering::Release);
        }
    }

    impl<T> RelativePtr<T> {
        /// Wraps a raw SHM offset into a Typed Relative Pointer.
        pub fn new(offset: u32) -> Self {
            Self { offset, _marker: PhantomData }
        }

        /// Converts the relative offset into a real memory reference.
        ///
        /// # Safety
        /// The 'base_ptr' must be the same base address used to map the shm in the
        /// current process.
        pub unsafe fn resolve<'a>(&self, base_ptr: *const u8) -> &'a T {
            unsafe {
                let ptr = base_ptr.add(self.offset as usize) as *const T;
                &*ptr
            }
        }

        /// Converts the relative offset into a mutable memory reference.
        ///
        /// # Safety
        /// The caller must ensure that no other process is currently writing to this
        /// memory block (typically enforced by the 'BlockHeader' state).
        pub unsafe fn resolve_mut<'a>(&self, base_ptr: *const u8) -> &'a mut T {
            unsafe {
                let ptr = base_ptr.add(self.offset as usize) as *mut T;
                &mut *ptr
            }
        }

        pub unsafe fn resolve_header<'a>(&self, base_ptr: *const u8) -> &'a BlockHeader {
            unsafe {
                let ptr = base_ptr.add((self.offset as usize) - 32) as *const BlockHeader;
                &*ptr
            }
        }

        /// Returns the raw underlying offset
        pub fn offset(&self) -> u32 {
            self.offset
        }
    }

    impl Mapping {
        /// Maps a requested buffer size to its corresponding coordinates in the TLSF matrix.
        ///
        /// This function performs a constant-time $O(1)$ calculation to determine which
        /// "Bucket" (First Level and Second Level) should hold a block of the given size.
        ///
        /// ### Algorithm Logic:
        /// 1. **Small Block Optimization (< 128B):** To prevent massive fragmentation in
        /// small messages, sizes are mapped linearly into 'fl=0' with a 16-byte granularity.
        ///
        /// 2. **Large Block Logarithmic Mapping (>= 128B):**
        ///    - **FL (First Level):** Represents the power of 2 ($2^{fl} \le size < 2^{fl+1}$).
        ///    Calculated using the CPU's 'CLZ' (Count Leading Zeros) instruction.
        ///    - **SL (Second Level):** Divides the range between $2^{fl}$ and $2^{fl+1}$ into
        ///    8 equal subdivisions (represented by 3 bits).
        ///
        /// ### Arguments
        /// * 'size': The requested allocation size in bytes (must be aligned to 16-bytes).
        ///
        /// ### Returns
        /// A tuple of '(fl, sl)' indices for the 'AtomicMatrix.matrix' array.
        pub fn find_indices(size: u32) -> (u32, u32) {
            if size < 128 {
                (0, (size / 16).min(7))
            } else {
                let fl = 31 - size.leading_zeros();
                let fl = fl.min(31);
                let sl = (size >> (fl - 3)) & 0x7;
                (fl, sl)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mapping() {
        assert_eq!(core::Mapping::find_indices(16), (0, 1));
        assert_eq!(core::Mapping::find_indices(64), (0, 4));
        assert_eq!(core::Mapping::find_indices(128), (7, 0));

        let (fl, sl) = core::Mapping::find_indices(1024);
        assert_eq!(fl, 10);
        assert_eq!(sl, 0);
    }

    #[test]
    fn test_initial_bootstrap() {
        let size = 1024 * 1024;
        let handler = core::AtomicMatrix::bootstrap(Some(uuid::Uuid::new_v4()), size).unwrap();

        let bitmap = handler.matrix.fl_bitmap.load(Ordering::Acquire);

        assert!(bitmap != 0, "FL bitmap should not be zero after sectorization");
        assert!(
            (bitmap & ((1 << 19) | (1 << 18))) != 0,
            "FL bitmap should have bits set for the expected sectors (19 and 18 for 512KB and 256KB"
        );
    }

    #[test]
    fn test_initialization_integrity() {
        let mut fake_shm = vec![0u8; 1024 * 1024];
        let base_ptr = fake_shm.as_mut_ptr();

        unsafe {
            let matrix_ptr = base_ptr.add(16) as *mut core::AtomicMatrix;
            let matrix = core::AtomicMatrix::init(matrix_ptr, uuid::Uuid::new_v4());

            matrix.sectorize(base_ptr, 1024 * 1024, 10, 20).unwrap();

            assert!(matrix.fl_bitmap.load(Ordering::Relaxed) != 0);
        }
    }

    #[test]
    fn test_allocation_and_spliting() {
        let size = 1024 * 1024;
        let mut handler = core::AtomicMatrix::bootstrap(Some(uuid::Uuid::new_v4()), size).unwrap();
        let base_ptr = handler.mmap.as_mut_ptr();
        let matrix = &mut *handler.matrix;

        unsafe {
            let rel_ptr = matrix.allocate(base_ptr, 64).unwrap();

            let header = rel_ptr.resolve_header(base_ptr);

            assert_eq!(header.size.load(Ordering::Acquire), 64);
            assert_eq!(header.state.load(Ordering::Acquire), STATE_ALLOCATED);
        }
    }

    #[test]
    fn test_multithreaded_stress() {
        // Quick author note:
        //
        // May God help us all at this moment.

        use std::sync::{ Arc, Barrier };
        use std::thread;
        use std::collections::HashSet;

        let size = 10 * 1024 * 1024;
        let handler = core::AtomicMatrix::bootstrap(Some(uuid::Uuid::new_v4()), size).unwrap();

        let thread_count = 8;
        let allocs_per_second = 100;
        let barrier = Arc::new(Barrier::new(thread_count));

        let mut handles = vec![];

        for _ in 0..thread_count {
            let b = Arc::clone(&barrier);
            let base_addr = handler.mmap.as_ptr() as usize;
            let matrix_addr = handler.matrix as *const core::AtomicMatrix as usize;

            handles.push(
                thread::spawn(move || {
                    let base_ptr = base_addr as *mut u8;
                    let matrix = unsafe { &*(matrix_addr as *const core::AtomicMatrix) };
                    let mut my_offsets = Vec::new();

                    b.wait();

                    for _ in 0..allocs_per_second {
                        for _ in 0..10 {
                            if let Ok(rel_ptr) = matrix.allocate(base_ptr, 64) {
                                my_offsets.push(rel_ptr.offset());
                                break;
                            }
                            std::hint::spin_loop();
                        }
                    }

                    my_offsets
                })
            );
        }

        let mut all_offsets = Vec::new();
        for h in handles {
            all_offsets.extend(h.join().unwrap());
        }

        let total_obtained = all_offsets.len();
        let unique_offsets: HashSet<_> = all_offsets.into_iter().collect();

        assert_eq!(
            total_obtained,
            thread_count * allocs_per_second,
            "Total allocations should match expected count"
        );
        assert_eq!(
            unique_offsets.len(),
            total_obtained,
            "RACE CONDITION DETECTED: Duplicate offsets found"
        );
        println!(
            "Successfully allocated {} unique blocks across {} threads without collisions!",
            total_obtained,
            thread_count
        );
    }
}
