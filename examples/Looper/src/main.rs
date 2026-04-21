// ATOMIC MATRIX BITCH
use atomic_matrix::handlers::HandlerFunctions;
use atomic_matrix::internals::looper;
use atomic_matrix::matrix::{
    core::*, 
    helpers,
};

// std imports
use std::time::Instant;

const PASSES: u32 = 1_000_000;

#[derive(Debug)]
struct Blocks<'a, T> {
    pub _header: &'a BlockHeader,
    pub _data_raw: &'a [u8],
    pub _data_cast: T,
    pub _offset: u32,
}

// Test iteration
fn main() {
    let matrix_size = 100 * 1024 * 1024; // We start with 100MB to fit a 1M u16 blocks
    let handler = AtomicMatrix::bootstrap(None, matrix_size).unwrap();
    let looper = looper::Looper::new(&handler);
    let mut my_blocks = Vec::new();
    let mut total_size = 0;

    let start = Instant::now();

    for _i in 0..PASSES {
        let mut block = handler.allocate::<u16>().unwrap();
        unsafe { handler.write(&mut block, 397) };
    }

    let alloc_time = start.elapsed();

    for v in looper {
        let h = v.view_header();
        let d_r = v.view_data_raw();
        let d_c = v.view_data_as::<u16>();

        if h.state.load(std::sync::atomic::Ordering::Acquire) == helpers::STATE_FREE {
            println!("Done")
        } else {
            total_size = total_size + h.size.load(std::sync::atomic::Ordering::Acquire);
            my_blocks.push(Blocks {
                _header: h,
                _data_raw: d_r,
                _data_cast: d_c,
                _offset: v.view_offset(),
            });
        }
    }

    let iter_time = start.elapsed();

    println!("Total blocks: {}", PASSES);
    println!(
        "Iter in {} ms. \n Alloc in {} ms",
        iter_time.as_millis(),
        alloc_time.as_millis()
    );
    println!("Total size used: {}MB", (total_size / 1024) / 1024);

    handler.die().unwrap();
}
