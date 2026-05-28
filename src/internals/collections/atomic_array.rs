use crate::internals::error_collection::AtomicArrayErrors;
use crate::helpers::atomic_ext::AtomicExtensions;
use crate::prelude::*;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU8, Ordering};

pub const STATE_ARRAY: u32 = 10_001;

const S_OPEN: u8 = 0;
const S_CLOSED: u8 = 1;
const S_WRITING: u8 = 2;
const S_READING: u8 = 3;

#[repr(C)]
pub struct Slot<T> {
    flag: AtomicU8,
    data: UnsafeCell<Option<T>>,
}

pub struct StrictRefMut<'a, T> {
    data: &'a mut T,
    slot: &'a Slot<T>,
}

#[repr(C, align(64))]
pub struct AtomicArray<T, const N: usize> {
    slots: [Slot<T>; N],
}

impl<T, const N: usize> AtomicArray<T, N> {
    pub fn new(handler: &MatrixHandler) -> &Self {
        let size = std::mem::size_of::<Self>();

        let block = handler.allocate_raw(size as u32).unwrap();

        let atomic_array =
            unsafe { *handler.base_ptr().add(block.offset() as usize) as *mut AtomicArray<T, N> };

        let slots: [Slot<T>; N] = std::array::from_fn(|_| Slot {
            flag: AtomicU8::new(S_OPEN),
            data: UnsafeCell::new(None),
        });

        unsafe {
            std::ptr::write(atomic_array, AtomicArray { slots });

            &*atomic_array
        }
    }

    pub fn new_from_map<F>(handler: &MatrixHandler, f: F) -> &Self
    where
        F: Fn() -> Slot<T>,
    {
        let size = std::mem::size_of::<Self>();

        let block = handler.allocate_raw(size as u32).unwrap();

        let atomic_array =
            unsafe { *handler.base_ptr().add(block.offset() as usize) as *mut AtomicArray<T, N> };

        let slots: [Slot<T>; N] = std::array::from_fn(|_| f());

        unsafe {
            std::ptr::write(atomic_array, AtomicArray { slots });

            &*atomic_array
        }
    }

    pub unsafe fn from(handler: &MatrixHandler, src: u32) -> &Self {
        let block = Block::<AtomicArray<T, N>>::from_offset(src);
        let atomic_array = unsafe { block.pointer.resolve(handler.base_ptr()) };

        atomic_array
    }

    /// Get a stale copy of the value within the informed index without locking.
    pub fn get(&self, idx: u32) -> Result<T, AtomicArrayErrors>
    where
        T: Copy,
    {
        let slot = &self[idx];

        match unsafe { *slot.data.get() } {
            Some(v) => Ok(v),
            None => Err(AtomicArrayErrors::EmptyIndexError),
        }
    }

    /// Get a live unlocked reference to the value inside the AtomicArray.
    ///
    /// This allows processes to observe the value while its being updated by
    /// other producers on the fly.
    pub fn ref_get(&self, idx: u32) -> Result<&T, AtomicArrayErrors> {
        let slot = &self[idx];

        match unsafe { &*slot.data.get() } {
            Some(v) => Ok(v),
            None => Err(AtomicArrayErrors::EmptyIndexError),
        }
    }

    /// Lock a slot and returns a mutable reference to its internal value.
    ///
    /// This can ideally be used to isolate a single slot inside a closed loop,
    /// since sharing the reference allows other threads to manipulate it even
    /// when locked.
    ///
    /// TODO: make it return a new struct with a drop trait to unlock the slot
    /// when the value is explicitly droped or goes out of scope to avoid ADHD
    /// slugs like me forgetting to unlocking the data and crashing the whole
    /// data structure in the process.
    pub fn strict_get<'a>(&'a self, idx: u32) -> Result<StrictRefMut<'a, T>, AtomicArrayErrors>
    where
        T: Copy,
    {
        let slot: &'a Slot<T> = &self[idx];

        if slot
            .flag
            .compare_exchange(S_OPEN, S_READING, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
        {
            let data: &'a mut Option<T> = unsafe { &mut *slot.data.get() };

            match data {
                Some(v) => Ok(StrictRefMut { data: v, slot }),
                None => {
                    slot.flag.store(S_OPEN, Ordering::Release);
                    Err(AtomicArrayErrors::EmptyIndexError)
                }
            }
        } else {
            Err(AtomicArrayErrors::BlockedSlotError(
                "The slot could not be read-locked because its already locked by another process"
                    .into(),
            ))
        }
    }

    /// Tries to set a slot on the fly
    ///
    /// Fails if it doesn't manage to set it to WRITTING
    pub fn set(&self, idx: u32, value: T) -> Result<(), AtomicArrayErrors> {
        let slot = &self[idx];
        let f_list: [u8; 2] = [S_CLOSED, S_WRITING];

        if slot.flag.store_if_not_any(
            &f_list,
            S_WRITING,
            Ordering::Release,
            Ordering::Relaxed
        ) {
            let data = unsafe { &mut *slot.data.get() };
            data.replace(value);

            slot.flag.store(S_OPEN, Ordering::Release);

            return Ok(());
        } else {
            return Err(AtomicArrayErrors::AtomicWriteFailed);
        }
    }

    /// Strictly waits to set a slot. This blocks the caller execution until either it manages
    /// to successfully set the slot, or it acheives the maximum defined number of tries.
    ///
    /// Fails at N given retries
    pub fn strict_set(&self, idx: u32, value: T, retries: u32) -> Result<u32, AtomicArrayErrors> {
        unimplemented!()
    }

    /// Tries to set a slot opportunistically.
    ///
    /// It either set the first empty slot, or returns a ArrayFull error.
    pub fn push() {
        unimplemented!()
    }

    /// Tries to set a slot opportunistically and keep it locked.
    pub fn strict_push() {
        unimplemented!()
    }

    /// Set all values inside the array to [`None`]
    ///
    /// This function completelly ignores the state machine and resets the
    /// whole array
    pub fn clear(&self, idx: u32) {
        unimplemented!()
    }

    /// removes the last value in the array and return it
    pub fn pop(&self) -> &T {
        unimplemented!()
    }

    /// removes a value from the array and moves all subsequent slots
    /// backwards.
    pub fn delete(&self, idx: u32) {
        unimplemented!()
    }

    /// Removes a value from the array without reorganizing slots.
    pub fn clean(&self, idx: u32) {
        unimplemented!()
    }

    /// unlock a slot without clearing its value
    pub fn unlock(&self, idx: u32) {
        unimplemented!()
    }

    /// Swap the place of two values
    pub fn swap(&self, idx1: u32, idx2: u32) {
        unimplemented!()
    }
}

impl<'a, T> Drop for StrictRefMut<'a, T> {
    fn drop(&mut self) {
        self.slot.flag.store(S_OPEN, Ordering::Release);
    }
}

impl<'a, T> std::ops::Deref for StrictRefMut<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.data
    }
}

impl<'a, T> std::ops::DerefMut for StrictRefMut<'a, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.data
    }
}

impl<T, const N: usize> std::ops::Index<u32> for AtomicArray<T, N> {
    type Output = Slot<T>;

    fn index(&self, idx: u32) -> &Self::Output {
        debug_assert!((idx as usize) < N, "Index out of bounds: {} >= {}", idx, N);
        unsafe { self.slots.get_unchecked(idx as usize) }
    }
}
