pub mod internals;
pub mod matrix;
pub mod extensive_lib;
pub mod helpers;

pub mod prelude {
    pub use crate::internals::{
        handlers::{
            Block, HandlerFunctions, MatrixHandler, SharedHandler,
        },
        collections::{
            atomic_ringbuffer::AtomicRingBuffer,
            atomic_array::AtomicArray,
        },
    };
    pub use crate::matrix::{core::*, helpers::*};
}
