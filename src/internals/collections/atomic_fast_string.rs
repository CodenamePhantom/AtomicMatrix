//! AtomicFastString works as a counterpart for the AtomicString, but faster (duh).
//! 
//! AtomicFastString works with the raw bytes from strings into a single block architecture (all
//! the string content is registered into a single block) that is linked into the main static
//! AtomicFastString struct. These bytes can then be iterated over, overwrited, shadow writed,
//! shadow copied, and many more fun methods that makes the API surface insuferable to work
//! with... But hey, at least its fast as fuck!
//! 
//! ### Architecture diagram
//! 
//! [Matrix Block]
//!     |
//!     |-> [AtomicFastString (metadata)(bytebuf pointer)]
//!             |
//!         [Matrix Block]
//!             |
//!             |-> [Bytebuf (string contents)]
//! 
//! The AtomicFastString block works as the enclosure of the string segment and it's never 
//! deallocated.
//! 
//! ### Runtime mutability
//! 
//! New strings that are written into the AtomicFastString will try to reutilize the already existing
//! content segment if the string has the same size or if its smaller than the current segment lenght.
//! Strings that surpasses the current segment will allocate a new block to write itself into and
//! swap the pointer at the metadata level, deallocating the previous string segment in the process.
//! 
//! Strings that are smaller than the current segment will invariantly waste the block spare space
//! in exchange for writting speed.
//! 
//! ### Corruption
//! 
//! Different from traditional (that is not traditional at all) AtomicString, writting procedures
//! utilizes a shadow write approach, which is not commited unless the whole writting process has
//! been completed successfully. So partial writes and corruption flags are not necessary for
//! this struct.
//! 
//! ### Rolling back.
//! 
//! Failed operations implement a basic Defer rollback procedure that unlocks the string from its
//! current state back to IDLE. However, the primitives for constructing your own rollback are
//! provided for more complex use cases
//! 
//! ### Safety
//! 
//! Like AtomicString, AtomicFastString also implements the RwLock philosophy, where all threads
//! can read simultaneosly, but only one thread can write to it at any given time. With the
//! difference that no Quorum Idling is provided by default, and if your use case requires this
//! behaviour, it is recommended to use AtomicString, or implement your own quorum agreement.