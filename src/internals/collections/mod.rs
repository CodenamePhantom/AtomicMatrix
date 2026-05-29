//! This crate is used to house organized data structures that can be used on
//! the matrix.
//!
//! All data structures present on this module inherit the lock-free phylosophy
//! of the project.

pub mod atomic_array;
pub mod atomic_ringbuffer;
pub mod atomic_hashmap;
pub mod atomic_vector;
pub mod uid_lite;
pub mod memory_scale;
pub mod atomic_timestamp;
