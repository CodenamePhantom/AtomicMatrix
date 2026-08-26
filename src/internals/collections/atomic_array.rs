//! The AtomicArray works as a thread safe, data array that can be concurrently manipulated and
//! used by separate threads and processes. Participants can push, pop, index and manipulate
//! array slots using the array internal functions, which automatically handles access ordering,
//! ownership and synchronization. Or index values directly using array[idx], although this does
//! not grant ordering, nor synchronization of data access.
//! 
//! # Architecture
//! 
//! The array is composed of (shockingly) an array of Slots, which contain an atomic flag and a
//! UnsafeCell that holds the value stored in the slot.
//! 
//! [Matrix Block]
//!     |
//!     |-> [Slot 1 (Flag/UnsafeCell<T>)]
//!     |-> [Slot 2 (Flag/UnsafeCell<T>)]
//!     |-> [Slot 3 (Flag/UnsafeCell<T>)]
//!     ... up to N slots defined by the user.
//! 
//! Internal operations like pop and push correctly coordinates the state machine to avoid data
//! mutation races, reading of unfinished writes, or deleting a locked block.
//! 
//! # Zero Copy
//! 
//! Due to the use of UnsafeCell, the data cannot be copied from inside the array. Having the
//! methods extract the value from inside the UnsafeCell directly and leaving a None in its place.
//! The array methods are also designed around this behaviour and will react to None slots
//! accordingly.
//! 
//! # Safety
//! 
//! The array allows users to index values and manipulate them directly. Opting for this behaviour
//! implies safelly manipulating the UnsafeCell register and synchronizing read/write/lock states
//! for slots manually. This allow users to manipulate their data however they want, with the
//! trade off of increased complexity and code surface.

use crate::internals::error_collection::AtomicArrayErrors;
use crate::helpers::safe_shm::SafeSHM;
use crate::prelude::*;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU8, Ordering};

pub const STATE_ARRAY: u32 = 10_001;

const S_OPEN: u8 = 0;
const S_CLOSED: u8 = 1;
const S_WRITING: u8 = 2;
const S_READING: u8 = 3;
const S_FREE: u8 = 4;

/// Array Slot structure.
/// 
/// It holds the atomic flag for the current slot as well as the value wrapped in an UnsafeCell.
/// This allows the value to be initalized as empty safelly for all participants in the array.
#[derive(SafeSHM)]
pub struct Slot<T> {
    flag: AtomicU8,
    data: UnsafeCell<Option<T>>,
}

/// A RAII guarded reference to a value inside a slot.
/// 
/// This struct allows a participant to hold a value while locking the slot it came from. When the
/// guard comes out of scope, the Drop behaviour atomically changes the state of this slot to
/// S_OPEN.
pub struct StrictRefMut<'a, T> {
    data: &'a mut T,
    slot: &'a Slot<T>,
}

/// The AtomicArray structure.
/// 
/// It holds an array containing all the slots declared by the user. This declaration is made at
/// compile time through a generic const value.
#[derive(SafeSHM)]
pub struct AtomicArray<T, const N: usize> {
    slots: [Slot<T>; N],
}

/// SAFETY:
/// 
/// Since this struct is merely a static reference to the actual data inside the matrix, sending it
/// across threads is safe
unsafe impl<T: SafeSHM, const N: usize> Send for AtomicArray<T, N> {}
unsafe impl<T: SafeSHM, const N: usize> Sync for AtomicArray<T, N> {}

impl<T: SafeSHM, const N: usize> AtomicArray<T, N> {
    /// Allocates a new AtomicArray and return its reference.
    /// 
    /// The array will live inside of the matrix as long as the SHM segment is valid.
    /// 
    /// ### Params
    /// @handler: A SharedHandler instance to allocate the array
    /// 
    /// ### Returns
    /// An optional static reference to the AtomicArray inside the matrix.
    pub fn new(handler: SharedHandler) -> Option<&'static Self> {
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

    /// Allocates a new AtomicArray using a caller provided map function and return its reference.
    /// 
    /// The array will live inside of the matrix as long as the SHM segment is valid.
    /// 
    /// ### Params
    /// @handler: A SharedHandler instance to allocate the array \
    /// @f: The map function to initialize the array. The function must return a Slot<T>
    /// 
    /// ### Returns
    /// An optional static reference to the AtomicArray inside the matrix.
    pub fn new_from_map<F>(handler: SharedHandler, f: F) -> Option<&'static Self>
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

    /// Queries an AtomicArray that already exists inside the matrix and return its reference.
    /// 
    /// If the offset provided does not match into an AtomicArray, None will be returned instead.
    /// 
    /// ### Params
    /// @handler: A Shared handler instance to query the array from \
    /// @src: The offset of the AtomicArray inside the matrix
    /// 
    /// ### Returns
    /// An optional static reference to the queried AtomicArray
    pub fn from(handler: SharedHandler, src: u32) -> Option<&'static Self> {
        let block = handler.get_block::<AtomicArray<T, N>>(src).ok()?;

        let arr_guard = handler.read(&block).ok()?;
        let atomic_array: &Self = unsafe { &*(&*arr_guard as *const Self) };

        Some(atomic_array)
    }

    /// Get a stale copy of the value within the informed index without locking.
    /// 
    /// If this slot doesn't have a value, an error will be returned instead.
    /// 
    /// ### Params
    /// @idx: The index of the slot to get the value from.
    /// 
    /// ### Returns
    /// A result containing either a copy of the value, or an AtomicArrayErrors.
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
    /// This allows processes to observe the value while its being updated by other producers on 
    /// the fly.
    /// 
    /// If this slot doesn't have a value, an error will be returned instead.
    /// 
    /// ### Params
    /// @idx: The index of the slot to get the value from.
    /// 
    /// ### Returns
    /// A result containing either a reference to the value, or an AtomicArrayErrors.
    pub fn ref_get(&self, idx: u32) -> Result<&T, AtomicArrayErrors> {
        let slot = &self[idx];

        match unsafe { &*slot.data.get() } {
            Some(v) => Ok(v),
            None => Err(AtomicArrayErrors::EmptyIndexError { slot_idx: idx }),
        }
    }

    /// Lock a slot and returns a mutable reference to its internal value.
    ///
    /// This can ideally be used to isolate a single slot inside a closed loop, since sharing the 
    /// reference allows other threads to manipulate it even when locked.
    /// 
    /// ### Safety
    /// 
    /// This strict ref is wrapped in a RAII guard, that will release the lock if the value ever
    /// gets dropped by the caller.
    /// 
    /// ### Params
    /// @idx: The index of the slot to get the value from.
    /// 
    /// ### Returns
    /// A result containing either the value wrapped in a StrictRefMut, or an AtomicArrayErrors.
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
    /// Fails if it doesn't manage to set it to WRITTING.
    /// 
    /// ### Params
    /// @idx: The index of the slot to get the value from \
    /// @value: The value to set the slot to
    /// 
    /// ### Returns
    /// A result containing either an empty Ok, or an AtomicArrayErrors.
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
    /// 
    /// ### Params
    /// @idx: The index of the slot to get the value from \
    /// @value: The value to set the slot to \
    /// @retries: The number of retries before failing the call
    /// 
    /// ### Returns
    /// A result containing either an empty Ok, or an AtomicArrayErrors.
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
    /// 
    /// ### Params
    /// @value: The value to set in the slot
    /// 
    /// ### Returns
    /// A result containing either the slot that the data was pushed to, or an AtomicArrayErrors
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
    /// 
    /// ### Params
    /// @value: The value to set in the slot
    /// 
    /// ### Returns
    /// A result containing either a StrictRefMut of the pushed slot, or an AtomicArrayErrors
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
    /// This function completelly ignores the state machine and resets the whole array
    pub fn clear(&self) {
        for idx in 0..N {
            let slot = &self[idx as u32];

            slot.flag.store(S_FREE, Ordering::Release);

            let data = unsafe { &mut *slot.data.get() };
            let _ = data.take();
        }
    }

    /// removes the last value in the array and return it
    /// 
    /// If the array is completely empty, the call will fail.
    /// 
    /// ### Returns
    /// A result containing either the value, or an AtomicArrayErrors
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

    /// Cleans a slots and set its flag to free.
    /// 
    /// ### Params
    /// @idx: The index of the slot to clean.
    pub fn clean(&self, idx: u32) {
        let slot = &self[idx];

        slot.flag.store(S_FREE, Ordering::Release);

        let data = unsafe { &mut *slot.data.get() };
        let _ = data.take();
    }

    /// Unlock a slot without clearing its value
    /// 
    /// ### Params
    /// @idx: The index of the slot to clean.
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

    /// Swap the place of two values inside the array.
    /// 
    /// This method will only execute if both slots are open.
    /// 
    /// ### Params
    /// @idx1: The index of the first slot to swap \
    /// @idx2: The index of the second slot to swap
    /// 
    /// ### Returns
    /// An option indicating if the swap was successful or not.
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
