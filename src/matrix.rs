use std::sync::atomic::{ AtomicU32, Ordering, fence };
use std::marker::PhantomData;
use std::fs::OpenOptions;
use memmap2::MmapMut;
use uuid::Uuid;

const SYS_UNINITIALIZED: u32 = 0;
const SYS_FORMATTING: u32 = 1;
const SYS_READY: u32 = 2;

const STATE_FREE: u32 = 0;
const STATE_ALLOCATED: u32 = 1;
const STATE_ACKED: u32 = 3;
const STATE_COALESCING: u32 = 4;

pub mod core {
    use super::*;

    #[derive(Debug)]
    #[repr(C, align(16))]
    pub struct BlockHeader {
        pub size: AtomicU32,
        pub state: AtomicU32,
        pub prev_phys: AtomicU32,
        pub next_free: AtomicU32,
        pub prev_free: AtomicU32, // Reserved for future use/alignment
    }

    #[repr(C)]
    pub struct AtomicMatrix {
        pub id: Uuid,
        pub fl_bitmap: AtomicU32,
        pub sl_bitmaps: [AtomicU32; 32],
        pub matrix: [[AtomicU32; 8]; 32],
        pub mmap: MmapMut,
        pub sector_boundaries: [AtomicU32; 4],
        pub total_size: u32,
        pub to_be_freed: [AtomicU32; 1024],
        pub to_be_freed_count: AtomicU32,
    }

    pub struct RelativePtr<T> {
        offset: u32,
        _marker: PhantomData<T>,
    }

    pub struct Mapping;

    impl AtomicMatrix {
        pub fn init(ptr: *mut AtomicMatrix, id: Uuid, size: u32) -> &'static mut Self {
            unsafe {
                let matrix = &mut *ptr;
                matrix.fl_bitmap.store(0, Ordering::Release);
                for i in 0..32 {
                    matrix.sl_bitmaps[i].store(0, Ordering::Release);
                    for j in 0..8 {
                        matrix.matrix[i][j].store(0, Ordering::Release);
                    }
                }
                matrix.id = id;
                matrix.total_size = size;
                matrix
            }
        }

        pub fn bootstrap(
            id: Option<Uuid>,
            size: usize
        ) -> Result<crate::handlers::matrix_handler::MatrixHandler, String> {
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
                let matrix = AtomicMatrix::init(matrix_ptr, path_id, size as u32);
                matrix.sectorize(base_ptr, size, 100, 0).unwrap();
                init_guard.store(SYS_READY, Ordering::SeqCst);
            } else {
                while init_guard.load(Ordering::Acquire) != SYS_READY {
                    std::hint::spin_loop();
                }
            }

            Ok(crate::handlers::matrix_handler::MatrixHandler {
                matrix: unsafe {
                    &mut *matrix_ptr
                },
                mmap,
            })
        }

        pub fn sectorize(
            &self,
            base_ptr: *mut u8,
            total_file_size: usize,
            mut small_percent: u8,
            mut medium_percent: u8
        ) -> Result<(), String> {
            let matrix_size = std::mem::size_of::<AtomicMatrix>();
            let mut current_offset = (16 + (matrix_size as u32) + 15) & !15;
            let usable_space = (total_file_size as u32).saturating_sub(current_offset);

            if total_file_size < 5 * 1024 * 1024 {
                small_percent = 30;
                medium_percent = 70;
            }

            let small_size =
                ((((usable_space as u64) * (small_percent as u64)) / 100) as u32) & !15;
            let medium_size =
                ((((usable_space as u64) * (medium_percent as u64)) / 100) as u32) & !15;
            let large_size = usable_space.saturating_sub(small_size).saturating_sub(medium_size);

            let mut prev_phys = 0u32;
            let mut sizes = vec![small_size, medium_size, large_size];
            sizes.retain(|&s| s > 64);

            for (i, size) in sizes.iter().enumerate() {
                let (fl, sl) = Mapping::find_indices(*size);
                self.create_and_insert_sector(base_ptr, current_offset, *size, prev_phys, fl, sl);
                prev_phys = current_offset;
                current_offset += size;

                if i < 4 {
                    self.sector_boundaries[i].store(current_offset, Ordering::Release);
                }
            }
            Ok(())
        }

        pub fn allocate(&self, base_ptr: *mut u8, size: u32) -> Result<RelativePtr<u8>, String> {
            let size = (size + 15) & !15;
            let size = size.max(32);
            let (fl, sl) = Mapping::find_indices(size);

            for _ in 0..512 {
                if let Some((f_fl, f_sl)) = self.find_suitable_block(fl, sl) {
                    if let Ok(offset) = self.remove_free_block(base_ptr, f_fl, f_sl) {
                        unsafe {
                            let header = &mut *(base_ptr.add(offset as usize) as *mut BlockHeader);
                            let total_size = header.size.load(Ordering::Acquire);

                            if total_size >= size + 32 {
                                let rem_size = total_size - size;
                                let next_off = offset + size;
                                let next_h = &mut *(
                                    base_ptr.add(next_off as usize) as *mut BlockHeader
                                );

                                next_h.size.store(rem_size, Ordering::Release);
                                next_h.state.store(STATE_FREE, Ordering::Release);
                                next_h.prev_phys.store(offset, Ordering::Release);
                                next_h.next_free.store(0, Ordering::Release);

                                header.size.store(size, Ordering::Release);
                                fence(Ordering::SeqCst);

                                let (r_fl, r_sl) = Mapping::find_indices(rem_size);
                                self.insert_free_block(base_ptr, next_off, r_fl, r_sl);
                            }
                            header.state.store(STATE_ALLOCATED, Ordering::Release);
                            return Ok(RelativePtr::new(offset + 32));
                        }
                    }
                }
                std::hint::spin_loop();
            }
            Err("OOM: Contention".into())
        }

        pub fn ack(&self, ptr: &RelativePtr<BlockHeader>, base_ptr: *mut u8) {
            unsafe {
                let header = ptr.resolve_mut(base_ptr);

                header.state.store(STATE_ACKED, Ordering::Release);

                let idx = self.to_be_freed_count.fetch_add(1, Ordering::Relaxed);

                if idx < (self.to_be_freed.len() as u32) {
                    self.to_be_freed[idx as usize].store(ptr.offset(), Ordering::Release);
                } else {
                    self.drain_and_coalesce(base_ptr);
                    self.to_be_freed_count.store(1, Ordering::Release);
                    self.to_be_freed[0].store(ptr.offset(), Ordering::Release);
                }
            }
        }

        pub fn coalesce(&self, ptr: &RelativePtr<BlockHeader>, base_ptr: *mut u8) {
            unsafe {
                let current_offset = ptr.offset();
                let mut current_header = ptr.resolve_mut(base_ptr);
                let mut total_size = current_header.size.load(Ordering::Acquire);
                if total_size < 32 {
                    return;
                }

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
                        let size_to_add = prev_header.size.swap(0, Ordering::Acquire);
                        if size_to_add == 0  || prev_phys_offset >= current_offset {
                            break;
                        }

                        if size_to_add > self.total_size {
                            break;
                        }

                        let (fl, sl) = Mapping::find_indices(size_to_add);

                        total_size = total_size.checked_add(size_to_add).expect("Tsunami detected.");
                        final_offset = prev_phys_offset;
                        current_header = prev_header;
                        self.remove_free_block(base_ptr, fl, sl).ok();
                    } else {
                        break;
                    }
                }

                let sector_limit = self.sector_end_offset(final_offset);
                if let Some(next_h_offset) = final_offset.checked_add(total_size) {
                    if next_h_offset < sector_limit {
                        let next_h = &*(base_ptr.add(next_h_offset as usize) as *const BlockHeader);
                        next_h.prev_phys.store(final_offset, Ordering::Release);
                    }
                }

                current_header.size.store(total_size, Ordering::Release);
                current_header.state.store(STATE_FREE, Ordering::Release);

                let (fl, sl) = Mapping::find_indices(total_size);
                self.insert_free_block(base_ptr, final_offset, fl, sl);
            }
        }

        pub fn query(&self, offset: u32) -> RelativePtr<u8> {
            RelativePtr::new(offset + 32)
        }

        fn drain_and_coalesce(&self, base_ptr: *mut u8) {
            for slot in self.to_be_freed.iter() {
                let offset = slot.swap(0, Ordering::Acquire);

                if offset != 0 {
                    let rel_ptr = RelativePtr::<BlockHeader>::new(offset);
                    self.coalesce(&rel_ptr, base_ptr);
                }
            }
        }

        fn find_suitable_block(&self, fl: u32, sl: u32) -> Option<(u32, u32)> {
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

        fn remove_free_block(&self, base_ptr: *mut u8, fl: u32, sl: u32) -> Result<u32, String> {
            let head = &self.matrix[fl as usize][sl as usize];
            loop {
                let off = head.load(Ordering::Acquire);
                if off == 0 {
                    return Err("Empty".into());
                }
                let next = unsafe {
                    (*(base_ptr.add(off as usize) as *const BlockHeader)).next_free.load(
                        Ordering::Acquire
                    )
                };
                if head.compare_exchange(off, next, Ordering::SeqCst, Ordering::Relaxed).is_ok() {
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

        fn insert_free_block(&self, base_ptr: *mut u8, offset: u32, fl: u32, sl: u32) {
            let head = &self.matrix[fl as usize][sl as usize];
            unsafe {
                let h = &mut *(base_ptr.add(offset as usize) as *mut BlockHeader);
                loop {
                    let old = head.load(Ordering::Acquire);
                    h.next_free.store(old, Ordering::Release);
                    if
                        head
                            .compare_exchange(old, offset, Ordering::SeqCst, Ordering::Relaxed)
                            .is_ok()
                    {
                        break;
                    }
                    std::hint::spin_loop();
                }
            }
            self.fl_bitmap.fetch_or(1 << fl, Ordering::Release);
            self.sl_bitmaps[fl as usize].fetch_or(1 << sl, Ordering::Release);
        }

        fn create_and_insert_sector(
            &self,
            base_ptr: *mut u8,
            offset: u32,
            size: u32,
            prev: u32,
            fl: u32,
            sl: u32
        ) {
            unsafe {
                let h = &mut *(base_ptr.add(offset as usize) as *mut BlockHeader);
                h.size.store(size, Ordering::Release);
                h.state.store(STATE_FREE, Ordering::Release);
                h.prev_phys.store(prev, Ordering::Release);
                h.next_free.store(0, Ordering::Release);
                self.insert_free_block(base_ptr, offset, fl, sl);
            }
        }

        fn sector_end_offset(&self, current_offset: u32) -> u32 {
            for i in 0..4 {
                let boundary = self.sector_boundaries[i].load(Ordering::Acquire);
                if boundary == 0 {
                    break;
                }
                if current_offset < boundary {
                    return boundary;
                }
            }

            self.mmap.len() as u32
        }
    }

    impl<T> RelativePtr<T> {
        pub fn new(offset: u32) -> Self {
            Self { offset, _marker: PhantomData }
        }
        pub fn offset(&self) -> u32 {
            self.offset
        }
        pub unsafe fn resolve_header<'a>(&self, base_ptr: *const u8) -> &'a BlockHeader {
            unsafe { &*(base_ptr.add((self.offset as usize) - 32) as *const BlockHeader) }
        }
        pub unsafe fn resolve_mut<'a>(&self, base_ptr: *const u8) -> &'a mut T {
            unsafe { &mut *(base_ptr.add(self.offset as usize) as *mut T) }
        }
    }

    impl Mapping {
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
}

#[cfg(test)]
mod tests {
    use crate::matrix::core::{ BlockHeader, RelativePtr };

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
            let matrix = core::AtomicMatrix::init(matrix_ptr, uuid::Uuid::new_v4(), fake_shm.len() as u32);

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

        // We use 500MB matrix to allocate all the buffers
        let size = 500 * 1024 * 1024;
        let handler = core::AtomicMatrix::bootstrap(Some(uuid::Uuid::new_v4()), size).unwrap();

        let thread_count = 8;
        let allocs_per_second = 1000000;
        let barrier = Arc::new(Barrier::new(thread_count));

        // Track the failed allocs
        let fail_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));

        let mut handles = vec![];

        // Fuck the matrix! GO GO GO
        for _ in 0..thread_count {
            let b = Arc::clone(&barrier);
            let base_addr = handler.mmap.as_ptr() as usize;
            let matrix_addr = handler.matrix as *const core::AtomicMatrix as usize;
            let fail_count_clone = Arc::clone(&fail_count);

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
                            fail_count_clone.fetch_add(1, Ordering::Relaxed);
                            std::hint::spin_loop();
                        }
                    }

                    my_offsets
                })
            );
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

        // Assert we can obtain at least 99.5% of the expected allocations without collisions, which would
        // indicate a potential race condition.
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
    }

    #[test]
    fn test_ack_and_coalesce() {
        let size = 1024 * 1024;
        let mut handler = core::AtomicMatrix::bootstrap(Some(uuid::Uuid::new_v4()), size).unwrap();
        let base_ptr = handler.mmap.as_mut_ptr();
        let matrix = &mut *handler.matrix;
        let block_ext = matrix.query(318288);

        println!("{:?}", matrix.matrix);

        unsafe {
            let h_ext = block_ext.resolve_header(base_ptr);
            println!("{:?}", h_ext);

            let ptr_a = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_b = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_c = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_d = matrix.allocate(base_ptr, 64).unwrap();
            let ptr_e = matrix.allocate(base_ptr, 64).unwrap();

            let h_b = ptr_b.resolve_header(base_ptr);
            let rel_c = RelativePtr::<BlockHeader>::new(ptr_c.offset() - 32);
            let rel_d = RelativePtr::<BlockHeader>::new(ptr_d.offset() - 32);

            h_b.state.store(STATE_FREE, Ordering::Release);
            matrix.ack(&rel_c, base_ptr);
            matrix.ack(&rel_d, base_ptr);

            matrix.coalesce(&rel_d, base_ptr);

            println!("Matrix: {:?}", matrix.matrix);
            println!("Header B: {:?}", ptr_b.offset() - 32);

            let h_a = ptr_a.resolve_header(base_ptr);
            println!("Header A: {:?}", ptr_a.offset() - 32);
            assert_eq!(h_a.state.load(Ordering::Acquire), STATE_ALLOCATED);

            let h_merged = ptr_b.resolve_header(base_ptr);
            assert_eq!(h_merged.state.load(Ordering::Acquire), STATE_FREE);
            assert_eq!(h_merged.size.load(Ordering::Acquire), 192);

            let h_e = ptr_e.resolve_header(base_ptr);
            println!("Header E: {:?}", ptr_e.offset() - 32);
            println!("Header E prev: {:?}", h_e.prev_phys);
            assert_eq!(h_e.state.load(Ordering::Acquire), STATE_ALLOCATED);

            let block_ext_1 = matrix.query(5632);
            let block_ext_2 = matrix.query(318288);

            let h_ext_1 = block_ext_1.resolve_header(base_ptr);
            let h_ext_2 = block_ext_2.resolve_header(base_ptr);

            println!("Extra blocks: \n {:?} \n {:?}", h_ext_1, h_ext_2);
            println!("Extra blocks offsets: {:?} {:?}", block_ext_1.offset(), block_ext_2.offset())
        }
    }

    #[test]
    fn test_long_term_fragmentation_healing() {
        use std::sync::{ Arc, Barrier };
        use std::thread;
        use std::time::{ Instant, Duration };

        const DURATION: u32 = 600;
        const THREADS: u32 = 16;

        let size = 50 * 1024 * 1024;
        let handler = core::AtomicMatrix::bootstrap(Some(uuid::Uuid::new_v4()), size).unwrap();
        let handler_arc = Arc::new(handler);
        let barrier = Arc::new(Barrier::new(THREADS as usize));
        let start_time = Instant::now();
        let duration = Duration::from_secs(DURATION as u64);

        let mut handles = vec![];

        for t_id in 0..THREADS {
            let h = Arc::clone(&handler_arc);
            let b = Arc::clone(&barrier);

            handles.push(
                thread::spawn(move || {
                    let base_ptr = h.mmap.as_ptr() as *mut u8;
                    let matrix = &h.matrix;
                    let mut my_blocks = Vec::new();
                    let mut rng = t_id + 1;
                    let mut total_ops = 0u64;

                    b.wait();

                    while start_time.elapsed() < duration {
                        rng = rng.wrapping_mul(1103515245).wrapping_add(12345);

                        if rng % 10 < 7 && my_blocks.len() < 200 {
                            let alloc_size = (rng % 512) + 32;
                            if let Ok(ptr) = matrix.allocate(base_ptr, alloc_size as u32) {
                                my_blocks.push(ptr);
                            }
                        } else if !my_blocks.is_empty() {
                            let idx = (rng as usize) % my_blocks.len();
                            let ptr = my_blocks.swap_remove(idx);

                            let header_ptr = RelativePtr::<BlockHeader>::new(ptr.offset() - 32);
                            matrix.ack(&header_ptr, base_ptr);

                            if total_ops % 5 == 0 {
                                matrix.coalesce(&header_ptr, base_ptr);
                            }
                        }
                        total_ops += 1;
                    }
                    (my_blocks, total_ops)
                })
            );
        }

        let mut total_work = 0u64;
        for h in handles {
            let (remaining, thread_ops) = h.join().unwrap();
            total_work += thread_ops;
            let base_ptr = handler_arc.mmap.as_ptr() as *mut u8;

            for ptr in remaining {
                let header_ptr = RelativePtr::<BlockHeader>::new(ptr.offset() - 32);
                handler_arc.matrix.ack(&header_ptr, base_ptr);
                handler_arc.matrix.coalesce(&header_ptr, base_ptr);
            }
        }

        let mut free_count = 0;
        for fl in 0..32 {
            for sl in 0..8 {
                if handler_arc.matrix.matrix[fl][sl].load(Ordering::Acquire) != 0 {
                    free_count += 1;
                }
            }
        }

        let entrophy_percentage = ((free_count / total_work) as f64) * 100.0;

        println!("Endurance Results (Duration of {} seconds)", DURATION);
        println!("Num of threads: {}", THREADS);
        println!("Total operations: {}", total_work);
        println!("Throughput: {:.2} Mop/s", (total_work as f64) / (DURATION as f64) / 1_000_000.0);
        println!("Final free fragments: {}", free_count);
        println!("Entrophy percentage: {}%", entrophy_percentage);

        assert!(entrophy_percentage < 0.001, "Excessive Fragmentation")
    }
}
