use crate::internals::error_collection::AtomicArrayErrors;
use crate::prelude::*;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU8, Ordering};

pub const STATE_ARRAY: u32 = 10_001;

const S_OPEN: u8 = 0;
const S_CLOSED: u8 = 1;
const S_WRITING: u8 = 2;
const S_READING: u8 = 3;
const S_FREE: u8 = 4;

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
    pub fn new(handler: &MatrixHandler) -> Option<&Self> {
        let block = handler.allocate::<AtomicArray<T, N>>().ok()?;

        handler.set_state(&block, STATE_ARRAY).ok()?;

        let slots: [Slot<T>; N] = std::array::from_fn(|_| Slot {
            flag: AtomicU8::new(S_FREE),
            data: UnsafeCell::new(None),
        });

        match handler.inline_mut(&block, |mut arr| {
            *arr = AtomicArray { slots };
        }) {
            Some(_) => {},
            None => return None,
        };

        let arr_guard = handler.read(&block).unwrap();
        let arr: &Self = unsafe { &*(&*arr_guard as *const Self) };

        return Some(arr)
    }

    pub fn new_from_map<F>(handler: &MatrixHandler, f: F) -> Option<&Self>
    where
        F: Fn() -> Slot<T>,
    {
        let block = handler.allocate::<AtomicArray<T, N>>().ok()?;

        handler.set_state(&block, STATE_ARRAY).ok()?;

        let slots: [Slot<T>; N] = std::array::from_fn(|_| f());

        match handler.inline_mut(&block, |mut arr| {
            *arr = AtomicArray { slots };
        }) {
            Some(_) => {},
            None => return None,
        };

        let arr_guard = handler.read(&block).ok()?;
        let arr: &Self = unsafe { &*(&*arr_guard as *const Self) };

        return Some(arr)
    }

    pub fn from(handler: &MatrixHandler, src: u32) -> Option<&Self> {
        let block = handler.get_block::<AtomicArray<T, N>>(src).ok()?;

        let arr_guard = handler.read(&block).ok()?;
        let atomic_array: &Self = unsafe { &*(&*arr_guard as *const Self) };

        Some(atomic_array)
    }

    /// Get a stale copy of the value within the informed index without locking.
    pub fn get(&self, idx: u32) -> Result<T, AtomicArrayErrors>
    where
        T: Copy,
    {
        let slot = &self[idx];

        match unsafe { *slot.data.get() } {
            Some(v) => Ok(v),
            None => Err(AtomicArrayErrors::EmptyIndexError { slot_idx: idx }),
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
            None => Err(AtomicArrayErrors::EmptyIndexError { slot_idx: idx }),
        }
    }

    /// Lock a slot and returns a mutable reference to its internal value.
    ///
    /// This can ideally be used to isolate a single slot inside a closed loop,
    /// since sharing the reference allows other threads to manipulate it even
    /// when locked.
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
                    Err(AtomicArrayErrors::EmptyIndexError { slot_idx: idx })
                }
            }
        } else {
            Err(AtomicArrayErrors::BlockedSlotError { slot_idx: idx })
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
            return Err(AtomicArrayErrors::AtomicWriteFailed { slot_idx: idx});
        }
    }

    /// Strictly waits to set a slot. This blocks the caller execution until either it manages
    /// to successfully set the slot, or it acheives the maximum defined number of tries.
    ///
    /// Fails at N given retries
    pub fn strict_set(&self, idx: u32, value: T, retries: u32) -> Result<(), AtomicArrayErrors> {
        let slot = &self[idx];
        let f_list = [S_CLOSED, S_WRITING];
        let mut retry_counter = 0;

        loop {
            if slot.flag.store_if_not_any(
                &f_list, 
                S_WRITING, 
                Ordering::Release, 
                Ordering::Relaxed
            ) {
                let data = unsafe {&mut *slot.data.get() };
                data.replace(value);

                slot.flag.store(S_OPEN, Ordering::Release);

                return Ok(())
            } else {
                retry_counter += 1;

                if retry_counter > retries {
                    return Err(AtomicArrayErrors::AtomicWriteFailed { slot_idx: idx});
                }
            }
        }
    }

    /// Tries to set a slot opportunistically.
    ///
    /// It either set the first empty slot, or returns a ArrayFull error.
    pub fn push(&self, value: T) -> Result<u32, AtomicArrayErrors> {
        for idx in 0..N {
            let slot = &self[idx as u32];
            
            if slot.flag.load(Ordering::Acquire) == S_FREE {
                slot.flag.store(S_WRITING, Ordering::Release);

                let data = unsafe { &mut *slot.data.get() };
                data.replace(value);

                slot.flag.store(S_OPEN, Ordering::Release);

                return Ok(idx as u32);
            }
        }

        return Err(AtomicArrayErrors::ArrayFull);
    }

    /// Tries to set a slot opportunistically and keep it locked.
    pub fn strict_push<'a>(&'a self, value: T) -> Result<StrictRefMut<'a, T>, AtomicArrayErrors> {
        for idx in 0..N {
            let slot: &'a Slot<T> = &self[idx as u32];
            
            if slot.flag.load(Ordering::Acquire) == S_FREE {
                slot.flag.store(S_WRITING, Ordering::Release);

                let data: &'a mut Option<T> = unsafe { &mut *slot.data.get() };
                data.replace(value);


                match data {
                    Some(v) => {
                        slot.flag.store(S_CLOSED, Ordering::Release);
                        return Ok(StrictRefMut { data: v, slot });
                    },
                    None => {
                        slot.flag.store(S_FREE, Ordering::Release);
                        return Err(AtomicArrayErrors::EmptyIndexError { slot_idx: idx as u32 });
                    }
                }
            }
        }

        return Err(AtomicArrayErrors::ArrayFull);
    }

    /// Set all values inside the array to [`None`]
    ///
    /// This function completelly ignores the state machine and resets the
    /// whole array
    pub fn clear(&self) {
        for idx in 0..N {
            let slot = &self[idx as u32];

            slot.flag.store(S_FREE, Ordering::Release);

            let data = unsafe { &mut *slot.data.get() };
            let _ = data.take();
        }
    }

    /// removes the last value in the array and return it
    pub fn pop(&self) -> Result<T, AtomicArrayErrors> {
        for idx in (0..N).rev() {
            let slot = &self[idx as u32];

            if slot.flag.load(Ordering::Acquire) == S_FREE {
                continue;
            }

            if slot.flag.swap_if(S_OPEN, S_CLOSED, Ordering::Acquire, Ordering::Relaxed).is_ok() {
                let data = unsafe { &mut *slot.data.get() };
                let val = data.take();
                
                slot.flag.store(S_FREE, Ordering::Release);

                if let Some(return_data) = val {
                    return Ok(return_data)
                } else {
                    return Err(AtomicArrayErrors::EmptyIndexError { slot_idx: idx as u32 })
                }
            } else {
                return Err(AtomicArrayErrors::BlockedSlotError { slot_idx: idx as u32 })
            }
        }

        return Err(AtomicArrayErrors::ArrayEmpty);
    }

    /// Removes a value from the array without reorganizing slots.
    pub fn clean(&self, idx: u32) {
        let slot = &self[idx];

        slot.flag.store(S_FREE, Ordering::Release);

        let data = unsafe { &mut *slot.data.get() };
        let _ = data.take();
    }

    /// unlock a slot without clearing its value
    pub fn unlock(&self, idx: u32) -> Option<()> {
        let slot = &self[idx];

        if slot.flag.compare_exchange(
            S_CLOSED, 
            S_OPEN, 
            Ordering::Release, 
            Ordering::Acquire
        ).is_ok() {
            return Some(());
        } else {
            return None
        }
    }

    /// Swap the place of two values
    pub fn swap(&self, idx1: u32, idx2: u32) -> Option<()> {
        // you seriously gonna try that?
        if idx1 == idx2 {
            return Some(())
        }

        let smallest = std::cmp::min(idx1, idx2);
        let largest = std::cmp::max(idx1, idx2);
        let slot_one = &self[smallest];
        let slot_two = &self[largest];
        let f_list = [S_CLOSED, S_READING, S_WRITING];

        let flag_one = match slot_one.flag.swap_if_not_any(
            &f_list, 
            S_CLOSED, 
            Ordering::Acquire, 
            Ordering::Relaxed
        ) {
            Ok(v) => v,
            Err(_) => return None,
        };
        let flag_two = match slot_two.flag.swap_if_not_any(
            &f_list, 
            S_CLOSED, 
            Ordering::Acquire,
            Ordering::Relaxed,
        ) {
            Ok(v) => v,
            Err(_) => {
                slot_one.flag.store(flag_one, Ordering::Release);
                return None
            },
        };
        let data_one = unsafe { &mut *slot_one.data.get() };
        let data_two = unsafe { &mut *slot_two.data.get() };

        std::mem::swap(data_one, data_two);

        slot_two.flag.store(flag_one, Ordering::Release);
        slot_one.flag.store(flag_two, Ordering::Release);
        
        Some(())
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
