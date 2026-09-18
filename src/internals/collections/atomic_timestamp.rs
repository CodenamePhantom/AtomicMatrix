/// AtomicTimestamps are monotonic representations of the latest commited SystemTime, but wrapped
/// into an atomic value to be safelly manipulated and accessed by multiple threads/process 
/// simultaneously.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// The AtomicTimestamp struct wraps a simple AtomicU64 value to provide it with temporal functions
#[repr(C)]
#[derive(Debug)]
pub struct AtomicTimestamp {
    value: AtomicU64,
}

impl AtomicTimestamp {
    /// Generates a new AtomicTimestamp using the current SystemTime
    /// 
    /// ### Returns
    /// An instance of self
    pub fn now() -> Self {
        Self {
            value: AtomicU64::new(Self::unix_now()),
        }
    }

    /// Generates a new uninitialized AtomicTimestamp.
    /// 
    /// The UTC value will remain zero until the timestamp is set.
    /// 
    /// ### Returns
    /// An instance of self
    pub fn uninitialized() -> Self {
        Self {
            value: AtomicU64::new(0),
        }
    }

    /// Updates the current value of the AtomicTimestamp to the current SystemTime
    pub fn set_now(&self) {
        self.value.store(Self::unix_now(), Ordering::Release);
    }

    /// Queries and returns the current AtomicTimestamp value set
    /// 
    /// ### Returns
    /// The timestamp stored in the struct.
    pub fn get(&self) -> u64 {
        self.value.load(Ordering::Acquire)
    }

    /// Gets the current SystemTime from the OS and returns it as a u64 integer.
    /// 
    /// ### Returns
    /// An u64 integer representing the current SystemTime.
    fn unix_now() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }
}
