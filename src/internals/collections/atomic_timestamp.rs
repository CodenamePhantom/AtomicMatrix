use std::sync::atomic::{ AtomicU64, Ordering };
use std::time::{ SystemTime, UNIX_EPOCH };

#[repr(C)]
#[derive(Debug)]
pub struct AtomicTimestamp {
    value: AtomicU64,
}

impl AtomicTimestamp {
    pub fn now() -> Self {
        Self { value: AtomicU64::new(Self::unix_now()) }
    }

    pub fn uninitialized() -> Self {
        Self { value: AtomicU64::new(0) }
    }

    pub fn set_now(&self) {
        self.value.store(Self::unix_now(), Ordering::Release);
    }

    pub fn get(&self) -> u64 {
        self.value.load(Ordering::Acquire)
    }

    fn unix_now() -> u64 {
        SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as u64
    }
}
