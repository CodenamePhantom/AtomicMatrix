use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::marker::PhantomData;
use std::fs::OpenOptions;
use memmap2::MmapMut;

use crate::handlers::matrix_handler::MatrixHandler;

const SYS_UNINITIALIZED: u32 = 0;
const SYS_FORMATTING: u32 = 1;
const SYS_READY: u32 = 2;

const STATE_FREE: u32 = 0;
const STATE_ALLOCATED: u32 = 1;
const STATE_READY: u32 = 2;
const STATE_ACKED: u32 = 3;
const STATE_COALESCING: u32 = 4;
const STATE_CACHED: u32 = 6;


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
        /// First-Level Bitmap: Each bit represents a power-of-two size class 
        /// (e.g., bit 10 = 1KB range).
        pub fl_bitmap: AtomicU32,
        /// Second-Level Bitmaps: Each entry sub-divides the FL power-of-two range 
        /// into 8 linear bins.
        pub sl_bitmaps: [AtomicU32; 32],
        /// The head pointers for the free lists. Each entry is an offset to a 
        /// 'BlockHeader'. Accessed via 'matrix[fl][sl]'.
        pub matrix: [[AtomicU32; 8]; 32]
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
        pub fn init(ptr: *mut AtomicMatrix) -> &'static mut Self {
            unsafe {
                let matrix = &mut *ptr;
                matrix.fl_bitmap.store(0, Ordering::Relaxed);

                for i in 0..32 {
                    matrix.sl_bitmaps[i].store(0, Ordering::Relaxed);

                    for j in 0..8 {
                        matrix.matrix[i][j].store(0, Ordering::Relaxed);
                    }
                }

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
        pub fn bootstrap(size: usize) -> Result<MatrixHandler, String> {
            let id = Uuid::new_v4();
            let path = format!("/dev/shm/{}", id);
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

            if init_guard.compare_exchange(
                SYS_UNINITIALIZED, 
                SYS_FORMATTING, 
                Ordering::SeqCst, 
                Ordering::Relaxed
            ).is_ok() {
                let matrix = AtomicMatrix::init(matrix_ptr);

                matrix.sectorize(base_ptr, 1024, 100, 0).unwrap();
                init_guard.store(SYS_READY, Ordering::SeqCst);
            } else {
                while init_guard.load(Ordering::Acquire) != SYS_READY {
                    std::hint::spin_loop();
                }
            }

            let matrix_ref = unsafe { &mut * matrix_ptr };

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
        pub fn sectorize(
            &self,
            base_ptr: *mut u8,
            total_file_size: usize,
            mut small_percent: u8,
            mut medium_percent: u8,
        ) -> Result<(), String> {
            let matrix_size = std::mem::size_of::<AtomicMatrix>();
            let mut current_offset = 16 + matrix_size as u32;
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

            let small_size = ((usable_space as u64 * small_percent as u64) / 100) as u32;
            let small_size = (small_size + 15) & !15;

            let medium_size = ((usable_space as u64 * medium_percent as u64) / 100) as u32;
            let medium_size = (medium_size + 15) & !15;

            let large_size = usable_space.saturating_sub(small_size).saturating_sub(medium_size);

            let mut prev_phys_offset = 0u32;

            if small_size > 0 {
                let (fl, sl) = Mapping::find_indices(small_size);
                self.create_and_insert_sector(base_ptr, current_offset, small_size, prev_phys_offset, fl, sl);
                prev_phys_offset = current_offset;
                current_offset += small_size;
            }

            if medium_size > 0 {
                let (fl, sl) = Mapping::find_indices(medium_size);
                self.create_and_insert_sector(base_ptr, current_offset, medium_size, prev_phys_offset, fl, sl);
                prev_phys_offset = current_offset;
                current_offset += medium_size;
            }

            if large_size > 16 {
                let (fl, sl) = Mapping::find_indices(large_size);
                self.create_and_insert_sector(base_ptr, current_offset, large_size, prev_phys_offset, fl, sl);
            }

            Ok(())
        }

        /// Allocates a block of memory of the requested size and returns a 'RelativePtr' to it.
        /// The allocation process involves:
        /// 1. Calculating the appropriate FL and SL indices using 'Mapping::find_indices
        /// 2. Checking the corresponding bitmaps to find a suitable free block.
        /// 3. If a block is found, atomically changing its state to ALLOCATED and update the bitmaps and free list
        /// 4. If no block is found in this current FL/SL, do a CLZ search in the FL bitmap to find the next available
        /// block
        /// 5. If a block is found in a higher FL, perform O(1) splitting to create a block of the requested size.
        /// 6. Return a 'RelativePtr' to the allocated block's data area.
        pub fn allocate(
            &self,
            base_ptr: *mut u8,
            size: u32
        ) -> Result<RelativePtr<u8>, String> {
            let size = (size + 15) & !15;
            let size = size.max(32);

            let (fl, sl) = Mapping::find_indices(size);

            let (found_fl, found_sl) = match self.find_suitable_block(fl, sl) {
                Some(coords) => coords,
                None => return Err("Out of memory: No suitable block found".to_string()),
            };

            let block_offset = self.remove_free_block(found_fl, found_sl)?;
            
            unsafe {
                let header_ptr = base_ptr.add(block_offset as usize) as *mut BlockHeader;
                let header = &mut *header_ptr;
                let total_size = header.size.load(Ordering::Acquire);

                if total_size >= size + 32 {
                    header.size.store(size, Ordering::Release);

                    let remaining_size = total_size - size;
                    let next_offset = block_offset + size;

                    let next_header = &mut *(base_ptr.add(next_offset as usize) as *mut BlockHeader);
                    next_header.size.store(remaining_size, Ordering::Release);
                    next_header.state.store(STATE_FREE, Ordering::Release);
                    next_header.prev_phys.store(block_offset, Ordering::Release);

                    let next_phys_offset = next_offset + remaining_size;

                    let (rem_fl, rem_sl) = Mapping::find_indices(remaining_size);
                    self.insert_free_block(next_offset, rem_fl, rem_sl);
                }

                header.state.store(STATE_ALLOCATED, Ordering::Release);

                Ok(RelativePtr::new(block_offset + 32))
            }
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
        pub fn ack(
            &self,
            ptr: RelativePtr<BlockHeader>,
            base_ptr: *mut u8,
        ) {
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
        pub fn coalesce(
            &self,
            ptr: RelativePtr<BlockHeader>,
            base_ptr: *mut u8,
        ) {
            unsafe {
                let header = ptr.resolve_mut(base_ptr);

                let mut total_size = header.size.load(Ordering::Acquire);

                // For ease, we will coalesce only with previous neigbours. The loop will jump the chain of prev
                // until no more free blocks are found, or it reaches the beginning of the segment (prev_phys = 0).
                let mut current_offset = ptr.offset();
                let mut prev_header: *mut BlockHeader = std::ptr::null_mut();

                while current_offset > 16 {
                    let prev_phys_offset = header.prev_phys.load(Ordering::Acquire);
                    if prev_phys_offset == 0 {
                        break;
                    }

                    prev_header = base_ptr.add(prev_phys_offset as usize) as *mut BlockHeader;
                    let prev_header_ref = &mut *prev_header;
                    if prev_header_ref.state.load(Ordering::Acquire) != STATE_FREE {
                        break;
                    }

                    // Someone swapped the state before we could coalesce with this block, so we stop iterating.
                    if prev_header_ref.state.compare_exchange(
                        STATE_FREE, 
                        STATE_COALESCING, 
                        Ordering::Acquire, 
                        Ordering::Relaxed
                    ).is_ok() {
                        total_size += prev_header_ref.size.load(Ordering::Acquire);
                        current_offset = prev_phys_offset;
                    } else {
                        break;
                    }
                }

                // Here we start coalescing with the found range. We update the header of the first block in the chain
                // with the total size, and we add it to the free list corresponding to its new size.
                if !prev_header.is_null() {
                    std::ptr::write_bytes(prev_header, 0, (total_size - 32) as usize);

                    (*prev_header).size.store(total_size, Ordering::Release);
                    (*prev_header).state.store(STATE_FREE, Ordering::Release);
                    let (fl, sl) = Mapping::find_indices(total_size);
                    self.insert_free_block(current_offset, fl, sl);
                }
            }
            
        }

        fn create_and_insert_sector(
            &self,
            base_ptr: *mut u8,
            offset: u32,
            size: u32,
            prev_phys: u32,
            fl: u32,
            sl: u32
        ) {
            let header_ptr = unsafe{ base_ptr.add(offset as usize) as *mut BlockHeader };
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

        fn find_suitable_block(&self, fl: u32, sl: u32) -> Option<(u32, u32)> {
            let sl_map = self.sl_bitmaps[fl as usize].load(Ordering::Acquire);
            let masked_sl = sl_map & (!0u32 << sl);

            if masked_sl != 0 {
                let found_sl = masked_sl.trailing_zeros();
                return Some((fl, found_sl));
            }

            let fl_map = self.fl_bitmap.load(Ordering::Acquire);
            let masked_fl = fl_map & (!0u32 << (fl + 1));

            if masked_fl != 0 {
                let found_fl = masked_fl.trailing_zeros();
                let sl_map = self.sl_bitmaps[found_fl as usize].load(Ordering::Acquire);
                let found_sl = sl_map.trailing_zeros();
                return Some((found_fl, found_sl));
            }

            None
        }

        fn remove_free_block(&self, fl: u32, sl: u32) -> Result<u32, String> {
            let list_head = &self.matrix[fl as usize][sl as usize];

            loop {
                let offset = list_head.load(Ordering::Acquire);
                if offset == 0 {
                    return Err("Race condition: free list became empty".to_string());
                }

                if list_head.compare_exchange(
                    offset, 
                    0, 
                    Ordering::SeqCst, 
                    Ordering::Relaxed
                ).is_ok() {
                    self.sl_bitmaps[fl as usize].fetch_and(!(1 << sl), Ordering::Release);
                    if self.sl_bitmaps[fl as usize].load(Ordering::Acquire) == 0 {
                        self.fl_bitmap.fetch_and(!(1 << fl), Ordering::Release);
                    }

                    return Ok(offset);
                }
            }
        }

        fn insert_free_block(&self, offset: u32, fl: u32, sl: u32) {
            let list_head = &self.matrix[fl as usize][sl as usize];

            let old_head = list_head.swap(offset, Ordering::SeqCst);

            if old_head != 0 {
                let old_header_ptr = list_head.load(Ordering::Acquire) as *mut BlockHeader;
                let old_header = unsafe { &mut *old_header_ptr };
                old_header.prev_free.store(offset, Ordering::Release);
                let new_header_ptr = unsafe { list_head.load(Ordering::Acquire) as *mut BlockHeader };
                let new_header = unsafe { &mut *new_header_ptr };
                new_header.next_free.store(old_head, Ordering::Release);
            }

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

        /// Returns the raw underlying offset
        pub fn offset(&self) -> u32 { self.offset }
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
                (0, size / 16)
            } else {
                let fl = 31 - size.leading_zeros();

                let sl = (size >> (fl - 3)) & 0x7;

                (fl, sl)
            }
        }
    }
}