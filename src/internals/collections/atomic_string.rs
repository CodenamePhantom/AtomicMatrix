//! AtomicStrings works as a thread safe implementation of dynamic sized strings for the
//! AtomicMatrix. It consists of a 255 char slot block, followed by a u32 containing the offset to
//! the next 255 char block on the chain.
//!
//! Strings that are written will be broken down to the char array and casted into this block. If
//! your string surpasses 255 chars, a new block with state STRING_CHAIN will be allocated with more
//! 255 slots to allocate the overflow, and the offset to this spill block will be appended into the
//! u32 pointer. If the string does not overflow, the whole data will be written into this single
//! block and the pointer will remain at zero.
//!
//! ### Architecture diagram
//!
//! [Matrix Block]
//!     |
//!     |-> [AtomicString (255 chars)(u32 pointer)]
//!             |
//!         [Matrix Block]
//!             |
//!             |-> [Overflow string (255 chars)(u32 pointer)]
//!                     |
//!                     ... Up to N overflowned blocks.
//!
//! The first AtomicString block works as the head for the segment and is never deallocated.
//!
//! ### Runtime mutability.
//!
//! New strings that are passed to an existing AtomicString will dinamically allocate new overflow
//! blocks as needed. This ensures that AtomicStrings doesn't need compile time known sizes and
//! remains dynamic. Overflow blocks are also limited to 255 chars and will spawn a subsequent
//! chain block if this limit is surpassed, that will be attached to the previous block in the
//! chain.
//!
//! If the string is mutated into a size smaller than the previous iteration, that makes an
//! overflow useless, this block will be deallocated from the string and its pointer will be zeroed
//!
//! ### Corruption
//!
//! If a write operation fails after the string has been touched, the AtomicString is_corrupted
//! flag will be set to true. Corrupted strings cannot be read by participants and will return
//! None immediately after trying.
//!
//! Write operations will pass through this state normally and overwrite the string. This behaviour
//! cleanse the state with a new set of chars and reverts the flag back to false.
//!
//! ### Quorum idling
//!
//! AtomicStrings have an optional agreement threshold that can be set. Strings that have a defined
//! threshold requires participants to agree on its reading before the last leaving participant
//! migrates the state machine from READING to IDLE.
//!
//! [WRITING] -> String has a quorum threshold? ----NO-> [IDLE]
//!                 |
//!                 Yes (16 readers)
//!                 |
//!                 |->[NEW]
//!                     |-> Reader 1 agrees (+1)
//!                     |-> Reader 2 agrees (+2)
//!                     ...
//!                     |-> Reader 15 agrees (+15)
//!                     |-> Reader 16 agrees (+16) ------> [IDLE] -> Writer overwrites it
//!
//! This ensures that the slowest reader has time to  reach the string and acquire its value before
//! it can be overwritten.
//!
//! If None is passed at the string declaration, the agreement quorum will not happen and any reader
//! that reaches 0 first will move the state of the string to IDLE.
//!
//! The quorum cannot be modified after the string has been created.
//!
//! ### Rolling back.
//!
//! Failed operations implement a Defer rollback that performs an action when the method is returned
//! earlier.
//!
//! - **Write Operations**: Write operations that have touched the string will flip the corruption
//! flag and switch the state back to IDLE. Strings that have not been touched will simply be
//! flipped back to IDLE
//! - **Read Operations**: Read fails will execute the default decrease operation (see
//! `reader_decrease()` function)
//!
//! ### Safety
//!
//! AtomicStrings are synchronized internally and follow the RwLock philosophy where everyone can
//! read at any time, but only one process/thread can write at any given time. Therefore, they are
//! not considered trully lock-free in the general sense, but they will also not spin lock your
//! process/thread uppon trying to manipulate it, returning None instead of locking.
//!
//! ### Panic!
//!
//! Failures in string operations (like partial writing, failing to allocate a chain link, or local
//! string corruptions) are considered recoverable and will return None from the method. However, if
//! any errors occurs during the `AtomicString::new()` call rollback, the operation is deemed
//! unrecoverable (the matrix itself could be corrupted at this stage), and the current participant
//! will panic.
// Quick author note:
// This could be implemented in a easier way... But where's the fun in that? Good luck.

use std::{
    cell::Cell,
    array,
    sync::atomic::{ AtomicBool, AtomicU8, AtomicU32, AtomicUsize, Ordering },
};

use crate::{ helpers::type_guard::TypeGuardMut, prelude::* };

/// Marks a string as IDLE, so writers know when its available for writing
const IDLE: u8 = 0;
/// Marks a string as READING, so writers don't overwrite the string when its being accessed
const READING: u8 = 1;
/// Marks a string as WRITING, so readers don't access a partialy written string
const WRITING: u8 = 2;
/// Marks a string as NEW, so writers don't overwrite a newly written string
const NEW: u8 = 3;
/// Sets the head block as an AtomicString block
const STATE_STRING: u32 = 900_000;
/// Sets a string link as an StringChain block
const STRING_CHAIN: u32 = 900_001;
/// How many chars will be stored per chain link
const CHAR_COUNT: usize = 64;

enum ClosureFlags {
    Break,
    Error,
    Next(u32),
}

/// AtomicString struct
///
/// It works as the head segment of the atomic string, holding critical metadata that all
/// participants will need access to.
#[derive(SafeSHM)]
#[repr(C)]
pub struct AtomicString {
    /// The current state of the AtomicString.
    state: AtomicU8,
    /// How many readers are currently accessing the string.
    readers: AtomicU32,
    /// The agreement threshold defined at start up.
    agreement: AtomicU32,
    /// The current amount of readers that agreed to read this string.
    curr_agreed: AtomicU32,
    /// The length of the currently stored string.
    string_length: AtomicUsize,
    /// The char contents, stored in a fixed size array.
    str: [char; CHAR_COUNT],
    /// The offset to the next link in this string's chain.
    next_overflow: AtomicU32,
    /// The current corruption state of this string.
    is_corrupted: AtomicBool,
    /// The offset to the head of this AtomicString block.
    this_offset: u32,
}

/// StringChain struct
///
/// A partial representation of the AtomicString that holds only the char contents and the offset
/// to the next block in the chain.
#[derive(SafeSHM)]
#[repr(C)]
pub struct StringChain {
    /// THe char contents of this chain link, also stored in a fixed size array.
    str: [char; CHAR_COUNT],
    /// THe offset to the next link in this string's chain.
    next_overflow: AtomicU32,
}

/// Defer is a RAII trigger that accepts a rollback function to be executed on the AtomicString.
///
/// Methods implement this to either free strings back, decrement readers, sinalize corruption, or
/// whatever the current saga may require as compensation.
///
/// ### Panic!
///
/// If the rollback function also requires calls that would fail, they invariantly have to panic!
/// the caller if something goes wrong, as the closure does not accept any kind of returns from it.
struct Defer<F: FnOnce()>(Option<F>);

impl<F: FnOnce()> Defer<F> {
    /// Sets the rollback function to be executed on dropping the Defer while its still armed.
    ///
    /// ### Params
    /// @f: The rollback function for the RAII guard.
    ///
    /// ### Returns
    /// An instance of self that will be dropped at the end of the scope.
    pub fn set(f: F) -> Self {
        return Self(Some(f));
    }

    /// Disarms the rollback in the RAII guard before dropping the scope.
    ///
    /// This ideally should be called after all breakable calls that are recoverable are completed
    /// to avoid triggering a rollback in a successful execution.
    pub fn disarm(&mut self) {
        self.0 = None;
    }
}

/// Drop implementation for Defer. Calls the rollback on being dropped.
impl<F: FnOnce()> Drop for Defer<F> {
    fn drop(&mut self) {
        if let Some(f) = self.0.take() {
            f();
        }
    }
}

impl AtomicString {
    /// Updates the AtomicTimestamp to the UNIX_EPOCH at the time of calling this function.
    ///
    /// The value of the timestamp can be used to compare versions of the string.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations.
    #[inline]
    fn update_versioning(&self, s_handler: SharedHandler) {
        let as_block = s_handler.get_block::<AtomicString>(self.offset()).unwrap();
        let block_header = unsafe { as_block.header() };

        block_header.last_edit.set_now();
    }

    /// Commits a write operation back to the string state machine.
    ///
    /// Strings that have a defined agreement threshold are marked as NEW, so no writers can
    /// overwrite the value before all participants agree to have read it. Strings without a
    /// threshold are marked as IDLE directly.
    #[inline]
    fn commit_write(&self) {
        self.is_corrupted.store(false, Ordering::Release);
        if self.agreement.load(Ordering::Acquire) > 0 {
            self.state.store(NEW, Ordering::Release);
        } else {
            self.state.store(IDLE, Ordering::Release);
        }
    }

    /// Performs an atomic operation to decrease the reader value and migrate the string to IDLE.
    ///
    /// The Quorum idling protocol is calculated here.
    #[inline]
    fn reader_decrease(&self) {
        self.readers
            .fetch_update(Ordering::Release, Ordering::Acquire, |mut v| {
                let agreement = self.agreement.load(Ordering::Acquire);
                v -= 1;

                if agreement > 0 {
                    if self.curr_agreed.fetch_add(1, Ordering::Release) + 1 == agreement {
                        self.curr_agreed.store(0, Ordering::Release);
                        self.state.store(IDLE, Ordering::Release);
                    }
                } else if v == 0 {
                    self.state.store(IDLE, Ordering::Release);
                }

                return Some(v);
            })
            .unwrap();
    }

    /// Helper function to map string blocks.
    ///
    /// It iterates over the string chain using the next offset available, collects the block for
    /// this chain link and executes a user provided closure with this block and a caller provided
    /// context (a mutable reference to a variable constructed locally).
    ///
    /// For chain blocks, if getting the next block fails, the caller will receive a None instead
    /// of the block se they can decide what is to be done with this failed event. If the caller
    /// decides to return None to this method as well, the iteration will break and finaly return.
    ///
    /// Errors that happen inside the closure should be explicitly returned wrapped in a Some(Err),
    /// so the traversal can know that a step failed and break out with a None instead of Some(()).
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations. \
    /// @context: A caller provided context to be used within the closure. Must be a mutable ref. \
    /// @head_fn: The closure to execute on the head block of the string. \
    /// @chain_fn: The closure to execute on the chain blocks.
    ///
    /// ### Returns
    /// An empty option stating the success of this method execution.
    fn traverse_string<T, F, G>(
        &self,
        s_handler: SharedHandler,
        context: &mut T,
        mut head_fn: F,
        mut chain_fn: G
    )
        -> Option<()>
        where
            F: FnMut(TypeGuardMut<AtomicString>, &mut T) -> ClosureFlags,
            G: FnMut(Option<(TypeGuardMut<StringChain>, u32)>, &mut T) -> ClosureFlags
    {
        let mut is_head = true;
        let mut next_link = self.next_overflow.load(Ordering::Acquire);

        loop {
            if is_head {
                let as_block = s_handler.get_block(self.offset()).ok()?;
                let str_ref = s_handler.read_mut(&as_block).ok()?;
                match head_fn(str_ref, context) {
                    ClosureFlags::Next(v) => {
                        next_link = v;
                    }
                    ClosureFlags::Error => {
                        return None;
                    }
                    ClosureFlags::Break => {
                        break;
                    }
                }
                is_head = false;
            } else {
                if next_link != 0 {
                    let chain_block = s_handler.get_block(next_link).ok()?;
                    let str_ref = s_handler.read_mut::<StringChain>(&chain_block).ok()?;
                    next_link = match
                        chain_fn(Some((str_ref, chain_block.header_offset())), context)
                    {
                        ClosureFlags::Next(v) => v,
                        ClosureFlags::Error => {
                            return None;
                        }
                        ClosureFlags::Break => {
                            break;
                        }
                    };
                } else {
                    next_link = match chain_fn(None, context) {
                        ClosureFlags::Next(v) => v,
                        ClosureFlags::Error => {
                            return None;
                        }
                        ClosureFlags::Break => {
                            break;
                        }
                    };
                }
            }
        }

        return Some(());
    }

    /// Acquires a read access to the string.
    ///
    /// This method executes a swap on the string state to READING except if its in state WRITING.
    ///
    /// Any call that fails inside of this method will immediately apply the rollback function and
    /// return None.
    ///
    /// ### Params
    /// @exec: The function to execute in case the swap succeeds.
    ///
    /// ### Returns
    /// An option stating the successful execution of this method
    #[inline]
    fn acquire_read<F>(&self, exec: F) -> Option<()> where F: FnOnce() -> Option<()> {
        if self.state.swap_if_not(WRITING, READING, Ordering::Acquire, Ordering::Relaxed).is_ok() {
            let mut rollback = Defer::set(|| {
                self.reader_decrease();
            });

            self.readers.fetch_add(1, Ordering::Release);

            exec()?;

            rollback.disarm();
            self.reader_decrease();

            return Some(());
        } else {
            return None;
        }
    }

    /// Acquires a write access to the string.
    ///
    /// This method executes a CAS on the string state to WRITING only if its in IDLE first.
    ///
    /// Any call that fails inside of this method will immediately apply the rollback function and
    /// return None.
    ///
    /// ### Params
    /// @exec: The function to execute in case the CAS succeeds.
    ///
    /// ### Returns
    /// An option stating the successful execution of this method
    #[inline]
    fn acquire_write<F>(
        &self,
        exec: F,
        touched: &Cell<bool>,
        s_handler: SharedHandler
    ) -> Option<()>
        where F: FnOnce() -> Option<()>
    {
        if self.state.compare_exchange(IDLE, WRITING, Ordering::SeqCst, Ordering::Relaxed).is_ok() {
            let mut rollback = Defer::set(|| {
                if touched.get() {
                    self.is_corrupted.store(true, Ordering::Release);
                }
                self.state.store(IDLE, Ordering::Release);
            });

            exec()?;

            rollback.disarm();

            self.commit_write();
            self.update_versioning(s_handler);

            return Some(());
        } else {
            return None;
        }
    }

    /// Creates a new AtomicString inside the matrix.
    ///
    /// It breaks down the string into chunks of 255 chars and write it into an AtomicString
    /// representation inside the block. The first chunk is appended directly inside the head
    /// struct str field, and the subsequent segments allocate a dedicated StringChain block that
    /// will hold its contents.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations. \
    /// @value: The string to write into the struct. \
    /// @agreement: The number of threads that needs to agree on the reading of a string before
    /// if can be marked as IDLE.
    ///
    /// ### Returns
    /// An static reference of the AtomicString.
    pub fn new(
        s_handler: SharedHandler,
        value: String,
        agreement: Option<u32>
    ) -> Option<&'static mut AtomicString> {
        use std::cell::Cell;

        let mut as_block = s_handler.allocate::<AtomicString>().ok()?;
        let mut block_offset = as_block.header_offset();
        let mut prev: Option<Block<StringChain>> = None;
        let mut chars = value.chars().peekable();
        let is_head = Cell::new(true);

        let mut rollback = Defer::set(|| {
            is_head.set(true);
            loop {
                if is_head.get() {
                    let corrupted_block = s_handler
                        .get_block::<AtomicString>(block_offset)
                        .unwrap();
                    let corrupted_ref = s_handler.read(&corrupted_block).unwrap();
                    block_offset = corrupted_ref.next_overflow.load(Ordering::Acquire);
                    s_handler.free(corrupted_block).unwrap();
                    is_head.set(false);
                } else {
                    let corrupted_block = s_handler.get_block::<StringChain>(block_offset).unwrap();
                    let corrupted_ref = s_handler.read(&corrupted_block).unwrap();
                    block_offset = corrupted_ref.next_overflow.load(Ordering::Acquire);
                    s_handler.free(corrupted_block).unwrap();

                    if block_offset == 0 {
                        break;
                    }
                }
            }
        });

        let agreement_to_apply: u32;
        if let Some(agreement_no) = agreement {
            agreement_to_apply = agreement_no;
        } else {
            agreement_to_apply = 0;
        }

        loop {
            let char_arr: [char; CHAR_COUNT] = array::from_fn(|_| chars.next().unwrap_or('\0'));

            if is_head.get() {
                let atomic_string = AtomicString {
                    state: AtomicU8::new(IDLE),
                    readers: AtomicU32::new(0),
                    agreement: AtomicU32::new(agreement_to_apply),
                    curr_agreed: AtomicU32::new(0),
                    string_length: AtomicUsize::new(value.chars().count()),
                    str: char_arr,
                    next_overflow: AtomicU32::new(0),
                    is_corrupted: AtomicBool::new(false),
                    this_offset: as_block.header_offset(),
                };
                s_handler
                    .write_transition(
                        &mut as_block,
                        atomic_string,
                        STATE_ALLOCATED,
                        STATE_STRING,
                        Ordering::Release
                    )
                    .ok()?;

                is_head.set(false);
            } else {
                let string_chain = StringChain {
                    str: char_arr,
                    next_overflow: AtomicU32::new(0),
                };
                let mut chain_block = s_handler.allocate::<StringChain>().ok()?;

                if let Some(ref mut previous) = prev {
                    let prev_ref = s_handler.read_mut(previous).ok()?;
                    prev_ref.next_overflow.store(chain_block.header_offset(), Ordering::Release);
                } else {
                    let as_ref = s_handler.read_mut::<AtomicString>(&mut as_block).ok()?;
                    as_ref.next_overflow.store(chain_block.header_offset(), Ordering::Release);
                }

                s_handler
                    .write_transition(
                        &mut chain_block,
                        string_chain,
                        STATE_ALLOCATED,
                        STRING_CHAIN,
                        Ordering::Release
                    )
                    .ok()?;

                prev = Some(chain_block);
            }

            if chars.peek().is_none() {
                break;
            }
        }

        let mut str_guard = s_handler.read_mut(&as_block).ok()?;
        let str: &mut Self = unsafe { &mut *(&mut *str_guard as *mut Self) };
        rollback.disarm();

        str.update_versioning(s_handler);

        return Some(str);
    }

    /// Returns an AtomicString instance from an already existing block.
    ///
    /// If the offset is not valid for any reason (it doesn't exist, or it's not a valid AtomicString),
    /// the function will fail and return None.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations. \
    /// @offset: The offset for the AtomicString block.
    ///
    /// ### Returns
    /// An optional static reference to the AtomicString.
    pub fn from(s_handler: SharedHandler, offset: u32) -> Option<&'static mut Self> {
        let as_block = s_handler.get_block::<AtomicString>(offset).ok()?;

        let mut str_guard = s_handler.read_mut(&as_block).ok()?;
        let str: &mut Self = unsafe { &mut *(&mut *str_guard as *mut Self) };

        return Some(str);
    }

    /// Writes a new String into the AtomicString segment.
    ///
    /// It reuses previously allocated AtomicStrings and StringChains to write the new string, and
    /// deallocates the remaining unused chains to save space. The head of the AtomicString is the
    /// only block that never gets deallocated.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations. \
    /// @value: The string to write into the struct.
    ///
    /// ### Returns
    /// An empty option stating if the operation was successful or not.
    pub fn write(&mut self, s_handler: SharedHandler, value: String) -> Option<()> {
        let touched = Cell::new(false);
        let mut chars = value.chars().peekable();
        let mut prev_str: Option<TypeGuardMut<StringChain>> = None;

        self.acquire_write(
            || {
                self.traverse_string(
                    s_handler,
                    &mut chars,
                    |mut str_ref, chars_ctx| {
                        let char_arr: [char; CHAR_COUNT] = array::from_fn(|_|
                            chars_ctx.next().unwrap_or('\0')
                        );

                        str_ref.str = char_arr;
                        str_ref.string_length.store(value.chars().count(), Ordering::Release);

                        touched.set(true);
                        return ClosureFlags::Next(str_ref.next_overflow.load(Ordering::Acquire));
                    },
                    |str_pack, chars_ctx| {
                        let mut next_offset = 0;
                        let char_arr: [char; CHAR_COUNT] = array::from_fn(|_|
                            chars_ctx.next().unwrap_or('\0')
                        );
                        let new_string = StringChain {
                            str: char_arr,
                            next_overflow: AtomicU32::new(0),
                        };

                        let (mut str_ref, this_offset) = match str_pack {
                            Some(v) => {
                                next_offset = v.0.next_overflow.load(Ordering::Acquire);
                                v
                            }
                            None => {
                                if chars_ctx.peek().is_none() {
                                    return ClosureFlags::Break;
                                } else {
                                    let Ok(block) = s_handler.allocate::<StringChain>() else {
                                        return ClosureFlags::Error;
                                    };
                                    if s_handler.set_state(&block, STRING_CHAIN).is_err() {
                                        return ClosureFlags::Error;
                                    }

                                    (s_handler.read_mut(&block).unwrap(), block.header_offset())
                                }
                            }
                        };
                        if let Some(ref mut prev) = prev_str {
                            prev.next_overflow.store(this_offset, Ordering::Release);
                        } else {
                            self.next_overflow.store(this_offset, Ordering::Release);
                        }

                        *str_ref = new_string;
                        prev_str = Some(str_ref);

                        if chars_ctx.peek().is_none() {
                            while next_offset != 0 {
                                let Ok(mut extra_block) =
                                    s_handler.get_block::<StringChain>(next_offset) else {
                                    return ClosureFlags::Error;
                                };
                                let Ok(extra_str) = s_handler.read_mut(&mut extra_block) else {
                                    return ClosureFlags::Error;
                                };

                                next_offset = extra_str.next_overflow.load(Ordering::Acquire);
                                if s_handler.free(extra_block).is_err() {
                                    return ClosureFlags::Error;
                                }
                            }
                            return ClosureFlags::Break;
                        } else {
                            return ClosureFlags::Next(next_offset);
                        }
                    }
                )
            },
            &touched,
            s_handler
        )
    }

    /// Appends a new string at the end of the current stored value
    ///
    /// The provided value will be appended immediately after the ending of the current string (read
    /// any char that is NILL), allocating any necessary chain links to fit this new string inside
    /// the struct.
    ///
    /// ### Params
    /// @s_handler: SharedHandler instance for matrix operations. \
    /// @value: The string to append at the end of the current stored string.
    ///
    /// ### Returns
    /// An option stating the successful execution of this method.
    pub fn append(&self, s_handler: SharedHandler, value: String) -> Option<()> {
        if self.is_corrupted.load(Ordering::Acquire) {
            return None;
        }

        let touched = Cell::new(false);
        let mut chars = value.chars().peekable();
        let mut prev_str: Option<TypeGuardMut<StringChain>> = None;

        self.acquire_write(
            || {
                self.traverse_string(
                    s_handler,
                    &mut chars,
                    |mut str_ref, chars_ctx| {
                        for ch in str_ref.str.iter_mut() {
                            if *ch == '\0' {
                                *ch = chars_ctx.next().unwrap_or('\0');
                            }
                        }
                        let old_length = str_ref.string_length.load(Ordering::Acquire);
                        str_ref.string_length.store(
                            value.chars().count() + old_length,
                            Ordering::Release
                        );
                        touched.set(true);

                        return ClosureFlags::Next(str_ref.next_overflow.load(Ordering::Acquire));
                    },
                    |str_pack, chars_ctx| {
                        let template_str = StringChain {
                            str: ['\0'; CHAR_COUNT],
                            next_overflow: AtomicU32::new(0),
                        };

                        let mut next_offset = 0;
                        let (mut str_ref, this_offset) = match str_pack {
                            Some(v) => {
                                next_offset = v.0.next_overflow.load(Ordering::Acquire);
                                v
                            }
                            None => {
                                if chars_ctx.peek().is_none() {
                                    return ClosureFlags::Break;
                                } else {
                                    let Ok(mut v) = s_handler.allocate::<StringChain>() else {
                                        return ClosureFlags::Error;
                                    };
                                    unsafe {
                                        if s_handler.write(&mut v, template_str).is_err() {
                                            return ClosureFlags::Error;
                                        }
                                    }
                                    if s_handler.set_state(&v, STRING_CHAIN).is_err() {
                                        return ClosureFlags::Error;
                                    }

                                    (s_handler.read_mut(&v).unwrap(), v.header_offset())
                                }
                            }
                        };
                        if let Some(ref mut prev) = prev_str {
                            prev.next_overflow.store(this_offset, Ordering::Release);
                        } else {
                            self.next_overflow.store(this_offset, Ordering::Release);
                        }

                        if str_ref.next_overflow.load(Ordering::Acquire) == 0 {
                            touched.set(true);
                            for ch in str_ref.str.iter_mut() {
                                if *ch == '\0' {
                                    *ch = chars_ctx.next().unwrap_or('\0');
                                }
                            }

                            prev_str = Some(str_ref);
                            if chars_ctx.peek().is_none() {
                                return ClosureFlags::Break;
                            } else {
                                return ClosureFlags::Next(next_offset);
                            }
                        } else {
                            return ClosureFlags::Next(
                                str_ref.next_overflow.load(Ordering::Acquire)
                            );
                        }
                    }
                )
            },
            &touched,
            s_handler
        )
    }

    /// Reads an string back from the struct.
    ///
    /// It follows the overflow chain until there is no more links to load from.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations.
    ///
    /// ### Returns
    /// An optional string loaded from the AtomicString segment.
    pub fn read(&self, s_handler: SharedHandler) -> Option<String> {
        if self.is_corrupted.load(Ordering::Acquire) {
            return None;
        }

        let mut str_buffs: Vec<String> = Vec::new();

        if
            self
                .acquire_read(|| {
                    self.traverse_string(
                        s_handler,
                        &mut str_buffs,
                        |str_ref, buff| {
                            let head_str = self.str
                                .iter()
                                .take_while(|&&c| c != '\0')
                                .collect();

                            buff.push(head_str);

                            return ClosureFlags::Next(
                                str_ref.next_overflow.load(Ordering::Acquire)
                            );
                        },
                        |mut str_pack, buff| {
                            if let Some((str_ref, _)) = str_pack.take() {
                                let str_chain = str_ref.str
                                    .iter()
                                    .take_while(|&&c| c != '\0')
                                    .collect();

                                buff.push(str_chain);

                                return ClosureFlags::Next(
                                    str_ref.next_overflow.load(Ordering::Acquire)
                                );
                            } else {
                                return ClosureFlags::Break;
                            }
                        }
                    )
                })
                .is_some()
        {
            let final_str = str_buffs.join("");

            return Some(final_str);
        } else {
            return None;
        }
    }

    /// Compares a String value against the AtomicString currently written.
    ///
    /// If the string doesn't match or manipulating the blocks fail for any reason, this call will
    /// return false.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix manipulation. \
    /// @value: The value to compare agains the AtomicString.
    ///
    /// ### Returns
    /// A bool stating if the provide string matches the AtomicString value.
    pub fn eq(&self, s_handler: SharedHandler, value: String) -> bool {
        let mut chars = value.chars().peekable();

        if self.is_corrupted.load(Ordering::Acquire) {
            return false;
        }

        if
            self
                .acquire_read(|| {
                    self.traverse_string(
                        s_handler,
                        &mut chars,
                        |str_ref, chars_ctx| {
                            let char_arr: [char; CHAR_COUNT] = array::from_fn(|_|
                                chars_ctx.next().unwrap_or('\0')
                            );
                            for (idx, char) in char_arr.iter().enumerate() {
                                if *char != self.str[idx] {
                                    return ClosureFlags::Error;
                                }
                            }

                            return ClosureFlags::Next(
                                str_ref.next_overflow.load(Ordering::Acquire)
                            );
                        },
                        |mut str_pack, chars_ctx| {
                            let char_arr: [char; CHAR_COUNT] = array::from_fn(|_|
                                chars_ctx.next().unwrap_or('\0')
                            );
                            let next_offset: u32;
                            if let Some((str_ref, _)) = str_pack.take() {
                                next_offset = str_ref.next_overflow.load(Ordering::Acquire);

                                for (idx, char) in char_arr.iter().enumerate() {
                                    if *char != str_ref.str[idx] {
                                        return ClosureFlags::Error;
                                    }
                                }
                            } else {
                                return ClosureFlags::Break;
                            }

                            return ClosureFlags::Next(next_offset);
                        }
                    )
                })
                .is_some()
        {
            return true;
        } else {
            return false;
        }
    }

    /// Executes a closure function againts each char in the AtomicString.
    ///
    /// It will traverse the whole chain of chars, including StringChains, and execute the closure
    /// until it captures the first \0 (NULL) char.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations.
    /// @f: The closure function to execute against the chars.
    ///
    /// ### Returns
    /// An empty option stating the success of the call execution.
    pub fn for_each_char<F>(&self, s_handler: SharedHandler, mut f: F) -> Option<()>
        where F: FnMut(char)
    {
        if self.is_corrupted.load(Ordering::Acquire) {
            return None;
        }

        self.acquire_read(|| {
            self.traverse_string(
                s_handler,
                &mut f,
                |str_ref, func| {
                    self.str
                        .iter()
                        .take_while(|&&c| c != '\0')
                        .for_each(|v| func(*v));

                    return ClosureFlags::Next(str_ref.next_overflow.load(Ordering::Acquire));
                },
                |mut str_pack, func| {
                    let next_offset;
                    if let Some((str_ref, _)) = str_pack.take() {
                        next_offset = str_ref.next_overflow.load(Ordering::Acquire);

                        str_ref.str
                            .iter()
                            .take_while(|c| **c != '\0')
                            .for_each(|v| func(*v));

                        return ClosureFlags::Next(next_offset);
                    } else {
                        return ClosureFlags::Break;
                    }
                }
            )
        })
    }

    /// Executes a closure function againts a mutable reference to each char in the AtomicString.
    ///
    /// It will traverse the whole chain of chars, including StringChains, and execute the closure
    /// until it captures the first \0 (NULL) char.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations.
    /// @f: The closure function to execute against the chars mutable reference.
    ///
    /// ### Returns
    /// An empty option stating the success of the call execution.
    pub fn for_each_char_mut<F>(&mut self, s_handler: SharedHandler, mut f: F) -> Option<()>
        where F: FnMut(&mut char)
    {
        if self.is_corrupted.load(Ordering::Acquire) {
            return None;
        }
        let touched = Cell::new(false);

        self.acquire_write(
            || {
                self.traverse_string(
                    s_handler,
                    &mut f,
                    |mut str_ref, func| {
                        str_ref.str
                            .iter_mut()
                            .take_while(|c| **c != '\0')
                            .for_each(|v| func(v));

                        return ClosureFlags::Next(str_ref.next_overflow.load(Ordering::Acquire));
                    },
                    |mut str_pack, func| {
                        let next_offset;
                        if let Some((mut str_ref, _)) = str_pack.take() {
                            next_offset = str_ref.next_overflow.load(Ordering::Acquire);

                            str_ref.str
                                .iter_mut()
                                .take_while(|c| **c != '\0')
                                .for_each(|v| func(v));

                            return ClosureFlags::Next(next_offset);
                        } else {
                            return ClosureFlags::Break;
                        }
                    }
                )
            },
            &touched,
            s_handler
        )
    }
    /// Clears the whole string, leaving only the AtomicString head block filled with NULL chars.
    ///
    /// This function follows the same premise of the write call, where only one writer can
    /// manipulate the AtomicString at a given time, while no one is reading it. So, if the call
    /// fails to CAS the state from IDLE, the call will fail.
    ///
    /// ### Params
    /// @s_handler:  A SharedHandler instance for matrix operations.
    ///
    /// ### Returns
    /// An empty option stating the success of the operation.
    pub fn clean(&mut self, s_handler: SharedHandler) -> Option<()> {
        let touched = Cell::new(false);

        self.acquire_write(
            || {
                self.traverse_string(
                    s_handler,
                    &mut String::new(),
                    |mut str_ref, _| {
                        let char_arr: [char; CHAR_COUNT] = ['\0'; CHAR_COUNT];

                        str_ref.str = char_arr;
                        str_ref.string_length.store(0, Ordering::Release);

                        return ClosureFlags::Next(str_ref.next_overflow.load(Ordering::Acquire));
                    },
                    |mut str_pack, _| {
                        if let Some((str_ref, this_offset)) = str_pack.take() {
                            let Ok(block) = s_handler.get_block::<StringChain>(this_offset) else {
                                return ClosureFlags::Error;
                            };
                            let next_offset = str_ref.next_overflow.load(Ordering::Acquire);
                            if s_handler.free(block).is_err() {
                                return ClosureFlags::Error;
                            }

                            return ClosureFlags::Next(next_offset);
                        } else {
                            return ClosureFlags::Break;
                        }
                    }
                )
            },
            &touched,
            s_handler
        )
    }

    /// Gets the current last_edit timestamp from the AtomicString block header.
    ///
    /// This value can be used to determine whether the string has been modified since the last
    /// query of versioning.
    ///
    /// ### DISCLAIMER
    ///
    /// AtomicString does not store versioning numbers internally. Callers are required to fetch
    /// timestamps locally on the process/thread and compare it according to their own use case.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix manipulation.
    ///
    /// ### Returns
    /// The current AtomicTimestamp value for this AtomicString in u64 format.
    pub fn get_versioning(&self, s_handler: SharedHandler) -> u64 {
        let as_block = s_handler.get_block::<AtomicString>(self.offset()).unwrap();
        let block_header = unsafe { as_block.header() };

        block_header.last_edit.get()
    }

    /// Returns the string chars splitted in a Vec.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix manipulation.
    ///
    /// ### Returns
    /// An optional Vec of chars representing the AtomicString value.
    pub fn chars(&self, s_handler: SharedHandler) -> Option<Vec<char>> {
        if let Some(string) = self.read(s_handler) {
            return Some(string.chars().collect::<Vec<char>>());
        } else {
            return None;
        }
    }

    /// Returns the length of the current string.
    pub fn len(&self) -> usize {
        return self.string_length.load(Ordering::Acquire);
    }

    /// Returns the offset to the AtomicString head inside the matrix.
    pub fn offset(&self) -> u32 {
        return self.this_offset;
    }

    /// Return the amount of agreements pending for this string to be marked as IDLE.
    pub fn pending_agreements(&self) -> u32 {
        if
            self.state.load(Ordering::Acquire) == READING &&
            self.agreement.load(Ordering::Acquire) > 0
        {
            return self.agreement.load(Ordering::Acquire) - self.curr_agreed.load(Ordering::Acquire);
        } else {
            return 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use crate::{
        core::{ handlers::HandlerFunctions, matrix::AtomicMatrix },
        internals::collections::{ atomic_string::AtomicString, memory_scale },
    };

    const SIZE: usize = memory_scale::sixteen::KB;

    #[test]
    fn string_cross_thread_reading() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let atomic_string = AtomicString::new(
            handler.share(),
            "I'm a string!".into(),
            None
        ).unwrap();
        let validation_block = handler.allocate::<bool>().unwrap();
        let v_ref = handler.read(&validation_block).unwrap();

        std::thread::scope(|s| {
            let as_offset = atomic_string.offset();
            let v_offset = validation_block.header_offset();
            let s_handler = handler.share();

            s.spawn(move || {
                let local_as = AtomicString::from(s_handler, as_offset).unwrap();
                let mut local_v = s_handler.get_block::<bool>(v_offset).unwrap();
                unsafe {
                    s_handler
                        .write(&mut local_v, local_as.eq(s_handler, "I'm a string!".into()))
                        .unwrap()
                }
            });
        });

        assert!(*v_ref);
    }

    #[test]
    fn string_change_readable() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let atomic_string = AtomicString::new(
            handler.share(),
            "I'm a string!".into(),
            None
        ).unwrap();
        let validation_block = handler.allocate::<bool>().unwrap();
        let v_ref = handler.read(&validation_block).unwrap();

        std::thread::scope(|s| {
            let as_offset = atomic_string.offset();
            let v_offset = validation_block.header_offset();
            let s_handler = handler.share();

            s.spawn(move || {
                let local_as = AtomicString::from(s_handler, as_offset).unwrap();
                let mut local_v = s_handler.get_block::<bool>(v_offset).unwrap();

                loop {
                    if local_as.eq(s_handler, "Now i'm a new string!".into()) {
                        unsafe {
                            s_handler.write(&mut local_v, true).unwrap();
                        }
                        break;
                    }
                }
            });
            s.spawn(move || {
                std::thread::sleep(Duration::from_millis(300));
                let local_as = AtomicString::from(s_handler, as_offset).unwrap();

                loop {
                    if local_as.write(s_handler, "Now i'm a new string!".into()).is_some() {
                        break;
                    }
                }
            });
        });

        assert!(*v_ref);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_failed_string_comparison() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let atomic_string = AtomicString::new(
            handler.share(),
            "I'm a string!".into(),
            None
        ).unwrap();
        let validation_block = handler.allocate::<bool>().unwrap();
        let v_ref = handler.read(&validation_block).unwrap();

        std::thread::scope(|s| {
            let as_offset = atomic_string.offset();
            let v_offset = validation_block.header_offset();
            let s_handler = handler.share();

            s.spawn(move || {
                let local_as = AtomicString::from(s_handler, as_offset).unwrap();
                let mut local_v = s_handler.get_block::<bool>(v_offset).unwrap();

                unsafe { 
                    s_handler.write(&mut local_v, !local_as.eq(s_handler, "Now i'm a new string!".into())).unwrap() 
                }
            });
        });

        assert!(*v_ref);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_looooooooooooong_strings() {
        let long_string =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*()-_=+[]{}<>?/ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*()-_=+[]{}<>?"; // sowy

        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let atomic_string = AtomicString::new(handler.share(), long_string.into(), None).unwrap();
        let validation_block = handler.allocate::<bool>().unwrap();
        let v_ref = handler.read(&validation_block).unwrap();

        std::thread::scope(|s| {
            let as_offset = atomic_string.offset();
            let v_offset = validation_block.header_offset();
            let s_handler = handler.share();

            s.spawn(move || {
                let local_as = AtomicString::from(s_handler, as_offset).unwrap();
                let mut local_v = s_handler.get_block::<bool>(v_offset).unwrap();

                unsafe { 
                    s_handler.write(&mut local_v, local_as.eq(s_handler, long_string.into())).unwrap() 
                }
            });
        });

        assert!(*v_ref);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_quorum_idling() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let atomic_string = AtomicString::new(handler.share(), "".into(), Some(4)).unwrap();
        let validation_block = handler.allocate::<bool>().unwrap();
        let v_ref = handler.read(&validation_block).unwrap();

        atomic_string.write(handler.share(), "This string requires quorum".into()).unwrap();
        assert_eq!(atomic_string.state.load(Ordering::Acquire), NEW);

        std::thread::scope(|s| {
            let s_handler = handler.share();
            let as_offset = atomic_string.offset();
            let v_offset = validation_block.header_offset();

            for _ in 0..4 {
                s.spawn(move || {
                    let local_as = AtomicString::from(s_handler, as_offset).unwrap();
                    let mut local_v = s_handler.get_block::<bool>(v_offset).unwrap();

                    while local_as.read(s_handler).is_none() {}

                    if local_as.pending_agreements() == 0 {
                        unsafe { s_handler.write(&mut local_v, true).unwrap() }
                    } else {
                        assert!(local_as.state.load(Ordering::Acquire) == READING);
                    }
                });
            }
        });

        assert!(*v_ref);

        unsafe { handler.die().unwrap() }
    }

    #[test]
    fn test_for_each_char() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let atomic_string = AtomicString::new(
            handler.share(),
            "This is a string with 5 S's".into(),
            Some(4)
        ).unwrap();
        let count_block = handler.allocate::<u32>().unwrap();
        let count_ref = handler.read(&count_block).unwrap();

        std::thread::scope(|s| {
            let as_offset = atomic_string.offset();
            let c_offset = count_block.header_offset();
            let s_handler = handler.share();

            s.spawn(move || {
                let local_as = AtomicString::from(s_handler, as_offset).unwrap();
                let local_c = s_handler.get_block::<u32>(c_offset).unwrap();
                let mut c_ref = s_handler.read_mut(&local_c).unwrap();

                local_as
                    .for_each_char(s_handler, |v| {
                        if v == 's' || v == 'S' {
                            *c_ref += 1;
                        }
                    })
                    .unwrap();
            });
        });

        assert_eq!(*count_ref, 5);
    }

    #[test]
    fn test_for_each_char_mut() {
        let handler = AtomicMatrix::bootstrap(None, SIZE).unwrap();
        let atomic_string = AtomicString::new(
            handler.share(),
            "I'm a string".into(),
            Some(4)
        ).unwrap();

        std::thread::scope(|s| {
            let as_offset = atomic_string.offset();
            let s_handler = handler.share();

            s.spawn(move || {
                let local_as = AtomicString::from(s_handler, as_offset).unwrap();

                local_as
                    .for_each_char_mut(s_handler, |v| {
                        *v = 'a';
                    })
                    .unwrap();
            });
        });

        assert_eq!(atomic_string.read(handler.share()).unwrap(), "aaaaaaaaaaaa")
    }
}
