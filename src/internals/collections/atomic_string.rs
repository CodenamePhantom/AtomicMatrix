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

use std::{ array, sync::atomic::{ AtomicBool, AtomicU8, AtomicU32, AtomicUsize, Ordering } };
use crate::{ helpers::type_guard::TypeGuardMut, prelude::* };

/// Marks a string as IDLE, so writers know when its available for writing
const IDLE: u8 = 0;
/// Marks a string as READING, so writers don't overwrite the string when its being accessed
const READING: u8 = 1;
/// Marks a string as WRITING, so readers don't access a partialy written string
const WRITING: u8 = 2;
/// Marks a string as NEW, so writers don't overwrite a newly written string.
const NEW: u8 = 3;
/// Sets the head block as an AtomicString block
const STATE_STRING: u32 = 900_000;
/// Sets a string link as an StringChain block
const STRING_CHAIN: u32 = 900_001;

const CHAR_COUNT: usize = 64;

/// AtomicString struct
///
/// It works as the head segment of the atomic string, holding critical metadata that all
/// participants will need access to.
#[derive(SafeSHM)]
pub struct AtomicString {
    state: AtomicU8,
    readers: AtomicU32,
    agreement: AtomicU32,
    curr_agreed: AtomicU32,
    string_length: AtomicUsize,
    str: [char; CHAR_COUNT],
    next_overflow: AtomicU32,
    is_corrupted: AtomicBool,
    this_offset: u32,
}

/// StringChain struct
///
/// A partial representation of the AtomicString that holds only the char contents and the offset
/// to the next block in the chain.
#[derive(SafeSHM)]
pub struct StringChain {
    str: [char; CHAR_COUNT],
    next_overflow: AtomicU32,
}

struct Defer<F: FnOnce()>(Option<F>);
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
    fn update_versioning(&self, s_handler: SharedHandler) {
        let as_block = s_handler.get_block::<AtomicString>(self.offset()).unwrap();
        let block_header = unsafe { as_block.header() };

        block_header.last_edit.set_now();
    }

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
    /// if can be marked as IDLE. \
    ///
    /// ### Returns
    /// An static reference of the AtomicString.
    pub fn new(
        s_handler: SharedHandler,
        value: String,
        agreement: Option<u32>
    ) -> Option<&'static AtomicString> {
        let mut as_block = s_handler.allocate::<AtomicString>().ok()?;

        let mut prev: Option<Block<StringChain>> = None;
        let mut is_head = true;
        let mut chars = value.chars().peekable();

        let agreement_to_apply: u32;
        if let Some(agreement_no) = agreement {
            agreement_to_apply = agreement_no;
        } else {
            agreement_to_apply = 0;
        }

        loop {
            let char_arr: [char; CHAR_COUNT] = array::from_fn(|_| chars.next().unwrap_or('\0'));

            if is_head {
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

                is_head = false;
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

        let str_guard = s_handler.read(&as_block).ok()?;
        let str: &Self = unsafe { &*(&*str_guard as *const Self) };

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
    pub fn from(s_handler: SharedHandler, offset: u32) -> Option<&'static Self> {
        let as_block = s_handler.get_block::<AtomicString>(offset).ok()?;

        let str_guard = s_handler.read(&as_block).ok()?;
        let str: &Self = unsafe { &*(&*str_guard as *const Self) };

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
        use std::cell::Cell;

        let touched = Cell::new(false);
        let mut rollback = Defer(
            Some(|| {
                if touched.get() {
                    self.is_corrupted.store(true, Ordering::Release);
                }
                self.state.store(IDLE, Ordering::Release);
            })
        );

        let mut chars = value.chars().peekable();
        let mut is_head = true;
        let mut prev_block: Option<Block<StringChain>> = None;
        let mut next_link = 0;

        if self.state.compare_exchange(IDLE, WRITING, Ordering::SeqCst, Ordering::Relaxed).is_ok() {
            loop {
                let char_arr: [char; CHAR_COUNT] = array::from_fn(|_| chars.next().unwrap_or('\0'));

                if is_head {
                    next_link = self.next_overflow.swap(0, Ordering::Release);

                    self.str = char_arr;
                    self.string_length.store(value.chars().count(), Ordering::Release);

                    is_head = false;
                    touched.set(true);
                } else {
                    let mut chain_block: Block<StringChain>;
                    let string_chain: TypeGuardMut<StringChain>;
                    let new_string_chain = StringChain {
                        str: char_arr,
                        next_overflow: AtomicU32::new(0),
                    };

                    if next_link != 0 {
                        match s_handler.get_block::<StringChain>(next_link) {
                            Ok(v) => {
                                chain_block = v;
                                string_chain = s_handler.read_mut(&chain_block).ok()?;
                                next_link = string_chain.next_overflow.swap(0, Ordering::Release);
                            }
                            Err(_) => {
                                chain_block = s_handler.allocate::<StringChain>().ok()?;
                                next_link = 0;
                            }
                        }
                    } else {
                        chain_block = s_handler.allocate::<StringChain>().ok()?;
                    }

                    if let Some(ref mut prev) = prev_block {
                        let prev_ref = s_handler.read_mut::<StringChain>(&prev).ok()?;
                        prev_ref.next_overflow.store(
                            chain_block.header_offset(),
                            Ordering::Release
                        );
                    } else {
                        self.next_overflow.store(chain_block.header_offset(), Ordering::Release);
                    }

                    unsafe {
                        s_handler.write(&mut chain_block, new_string_chain).ok()?;
                    }
                    s_handler.set_state(&chain_block, STRING_CHAIN).ok()?;

                    prev_block = Some(chain_block);
                }

                if chars.peek().is_none() {
                    break;
                }
            }
        } else {
            rollback.0 = None;
            return None;
        }

        while next_link != 0 {
            let mut extra_block = s_handler.get_block::<StringChain>(next_link).ok()?;
            let extra_str = s_handler.read_mut(&mut extra_block).ok()?;
            next_link = extra_str.next_overflow.load(Ordering::Acquire);
            s_handler.free(extra_block).ok()?;
        }

        rollback.0 = None;

        self.is_corrupted.store(false, Ordering::Release);
        if self.agreement.load(Ordering::Acquire) > 0 {
            self.state.store(NEW, Ordering::Release);
        } else {
            self.state.store(IDLE, Ordering::Release);
        }

        self.update_versioning(s_handler);

        return Some(());
    }

    pub fn append(&self, s_handler: SharedHandler, value: String) -> Option<()> {
        unimplemented!()
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
        let mut rollback = Defer(
            Some(|| {
                self.reader_decrease();
            })
        );

        let mut is_head = true;
        let mut str_buffs: Vec<String> = Vec::new();
        let mut next_link = 0;

        if self.is_corrupted.load(Ordering::Acquire) {
            rollback.0 = None;
            return None;
        }

        if self.state.swap_if_not(WRITING, READING, Ordering::Acquire, Ordering::Relaxed).is_ok() {
            self.readers.fetch_add(1, Ordering::Release);

            loop {
                if is_head {
                    let head_str: String = self.str
                        .iter()
                        .take_while(|&&c| c != '\0')
                        .collect();
                    str_buffs.push(head_str);

                    next_link = self.next_overflow.load(Ordering::Acquire);
                    is_head = false;
                } else {
                    let next_chain = s_handler.get_block::<StringChain>(next_link).ok()?;
                    let str_ref = s_handler.read(&next_chain).ok()?;
                    let str_chain: String = str_ref.str
                        .iter()
                        .take_while(|&&c| c != '\0')
                        .collect();
                    str_buffs.push(str_chain);

                    next_link = str_ref.next_overflow.load(Ordering::Acquire);
                }

                if next_link == 0 {
                    break;
                }
            }

            self.reader_decrease();
        } else {
            rollback.0 = None;
            return None;
        }

        rollback.0 = None;

        let final_str = str_buffs.join("");

        return Some(final_str);
    }

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
        let mut next_link = 0;
        let mut is_head = true;

        if self.is_corrupted.load(Ordering::Acquire) {
            return false;
        }

        if self.state.swap_if_not(WRITING, READING, Ordering::Acquire, Ordering::Relaxed).is_ok() {
            self.readers.fetch_add(1, Ordering::Release);

            loop {
                let char_arr: [char; CHAR_COUNT] = array::from_fn(|_| chars.next().unwrap_or('\0'));

                if is_head {
                    for (idx, char) in char_arr.iter().enumerate() {
                        if *char != self.str[idx] {
                            self.reader_decrease();
                            return false;
                        }
                    }

                    is_head = false;
                    next_link = self.next_overflow.load(Ordering::Acquire);
                } else {
                    if next_link != 0 {
                        let Ok(chain_block) = s_handler.get_block::<StringChain>(next_link) else {
                            self.reader_decrease();
                            return false;
                        };
                        let Ok(str_ref) = s_handler.read(&chain_block) else {
                            self.reader_decrease();
                            return false;
                        };

                        for (idx, char) in char_arr.iter().enumerate() {
                            if *char != str_ref.str[idx] {
                                self.reader_decrease();
                                return false;
                            }
                        }

                        next_link = str_ref.next_overflow.load(Ordering::Acquire);
                    } else if chars.peek().is_some() {
                        self.reader_decrease();
                        return false;
                    }
                }

                if chars.peek().is_none() {
                    break;
                }
            }

            self.reader_decrease();
        } else {
            return false;
        }

        return true;
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
        let mut rollback = Defer(
            Some(|| {
                self.reader_decrease();
            })
        );

        if self.is_corrupted.load(Ordering::Acquire) {
            rollback.0 = None;
            return None;
        }

        let mut next_link = 0;
        let mut is_head = true;

        if self.state.swap_if_not(WRITING, READING, Ordering::Acquire, Ordering::Relaxed).is_ok() {
            self.readers.fetch_add(1, Ordering::Release);

            loop {
                if is_head {
                    self.str
                        .iter()
                        .take_while(|&&c| c != '\0')
                        .for_each(|v| f(*v));

                    next_link = self.next_overflow.load(Ordering::Acquire);
                    is_head = false;
                } else {
                    let chain_block = s_handler.get_block::<StringChain>(next_link).ok()?;
                    let str_ref = s_handler.read(&chain_block).ok()?;

                    str_ref.str
                        .iter()
                        .take_while(|&&c| c != '\0')
                        .for_each(|v| f(*v));

                    next_link = str_ref.next_overflow.load(Ordering::Acquire);
                }

                if next_link == 0 {
                    break;
                }
            }

            self.reader_decrease();
        } else {
            rollback.0 = None;
            return None;
        }

        rollback.0 = None;

        return Some(());
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
        use std::cell::Cell;

        let touched = Cell::new(false);
        let mut rollback = Defer(
            Some(|| {
                if touched.get() {
                    self.is_corrupted.store(true, Ordering::Release);
                }
                self.state.store(IDLE, Ordering::Release);
            })
        );

        let mut next_link;

        if self.state.compare_exchange(IDLE, WRITING, Ordering::SeqCst, Ordering::Relaxed).is_ok() {
            let char_arr: [char; CHAR_COUNT] = ['\0'; CHAR_COUNT];

            self.str = char_arr;
            self.string_length.store(0, Ordering::Release);

            next_link = self.next_overflow.load(Ordering::Acquire);
            touched.set(true);
            self.next_overflow.store(0, Ordering::Release);
        } else {
            rollback.0 = None;
            return None;
        }

        while next_link != 0 {
            let mut extra_block = s_handler.get_block::<StringChain>(next_link).ok()?;
            let extra_str = s_handler.read_mut(&mut extra_block).ok()?;
            next_link = extra_str.next_overflow.load(Ordering::Acquire);
            s_handler.free(extra_block).ok()?;
        }

        rollback.0 = None;

        self.is_corrupted.store(false, Ordering::Release);
        if self.agreement.load(Ordering::Acquire) > 0 {
            self.state.store(NEW, Ordering::Release);
        } else {
            self.state.store(IDLE, Ordering::Release);
        }

        self.update_versioning(s_handler);

        return Some(());
    }

    /// Return the amount of agreements pending for this string to be marked as IDLE.
    pub fn pending_agreements(&self) -> u32 {
        return self.agreement.load(Ordering::Acquire) - self.curr_agreed.load(Ordering::Acquire);
    }
}
