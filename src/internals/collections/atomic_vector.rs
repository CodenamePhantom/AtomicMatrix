//! AtomicVectors works as a dynamic sized, matrix safe vector structure that can be safely shared
//! across all participants.
//! 
//! They are composed of statically sized matrix blocks linked to each other, that contains the 
//! array of values corresponding to this block index range. Each block contains a fixed index range
//! from the whole array that is used to calculate the number of link jumps that queries have to do
//! in order to reach this specified value.
//! 
//! ### Architecture
//! 
//! ```text
//! [Matrix Block]
//!     |
//!     |-> [AtomicVector (Fixed array of T)(u32 pointer)]
//!             |
//!         [Matrix Block]
//!             |
//!             |-> [Overflow vector (Fixed array of T)(u32 pointer)]
//!                     |
//!                     ... Up to N overflowed blocks.
//! ```
//! 
//! The head of the AtomicVector is static and its never deallocated. It works as the anchor to
//! traverse the vector chain.
//! 
//! ### Runtime mutability
//! 
//! The only internal operation from the vector that actually requires a lock is popping a value
//! that exists before the last value in the list, as all subsequent values needs to be moved
//! backwards. All other read/write operations happens fully atomically at the data layer.