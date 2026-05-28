//! A simple memory layout facilitator.
//!
//! It includes 4 base-two scales (2, 4, 8, and 16) baked in, as well as a way
//! to easely declare custom sizes within memory scales.

enum MemoryScale {
    B = 1,
    KiB = 1024,
    MiB = 1048576,
    GiB = 1073741824,
}

const fn custom(amount: usize, scale: MemoryScale) -> usize {
    amount * (scale as usize)
}

/// Size Two scaling for memory layout.
pub mod two {
    use super::{MemoryScale, custom};

    pub const B: usize = custom(2, MemoryScale::B);
    pub const KB: usize = custom(2, MemoryScale::KiB);
    pub const MB: usize = custom(2, MemoryScale::MiB);
    pub const GB: usize = custom(2, MemoryScale::GiB);
}

/// Size Four scaling for memory layout.
pub mod four {
    use super::{MemoryScale, custom};

    pub const B: usize = custom(4, MemoryScale::B);
    pub const KB: usize = custom(4, MemoryScale::KiB);
    pub const MB: usize = custom(4, MemoryScale::MiB);
    pub const GB: usize = custom(4, MemoryScale::GiB);
}

/// Size Eight scaling for memory layout.
pub mod eight {
    use super::{MemoryScale, custom};

    pub const B: usize = custom(8, MemoryScale::B);
    pub const KB: usize = custom(8, MemoryScale::KiB);
    pub const MB: usize = custom(8, MemoryScale::MiB);
    pub const GB: usize = custom(8, MemoryScale::GiB);
}

/// Size Sixteen scaling for memory layout.
pub mod sixteen {
    use super::{MemoryScale, custom};

    pub const B: usize = custom(16, MemoryScale::B);
    pub const KB: usize = custom(16, MemoryScale::KiB);
    pub const MB: usize = custom(16, MemoryScale::MiB);
    pub const GB: usize = custom(16, MemoryScale::GiB);
}

/// Custom memory layout declaration.
///
/// # Usage
///
/// let size = memory_scale::custom::mb::<353>();
/// let handler = AtomicMatrix::bootstrap(None, size).unwrap();
pub mod custom {
    use super::{MemoryScale, custom};

    pub const fn b<const VAL: usize>() -> usize {
        custom(VAL, MemoryScale::B)
    }
    pub const fn kb<const VAL: usize>() -> usize {
        custom(VAL, MemoryScale::KiB)
    }
    pub const fn mb<const VAL: usize>() -> usize {
        custom(VAL, MemoryScale::MiB)
    }
    pub const fn gb<const VAL: usize>() -> usize {
        custom(VAL, MemoryScale::GiB)
    }
}

