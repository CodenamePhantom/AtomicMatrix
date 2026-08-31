//! AtomicStrings works as a thread safe implementation of dynamic sized strings for the
//! AtomicMatrix. It consists of a 255 char slot block, followed by a u32 containing the offset to
//! the next 255 char block on the chain.
//!
//! Strings that are written will be broken down to the char array and casted into this block. If
//! your string surpasses 255 chars, a new block with state STRING_CHAIN will be allocated for the
//! exact size of the overflow, and the offset to this spill block will be appended into the u32
//! pointer. If the string does not overflow, the whole data will be written into this single block
//! and the pointer will remain at zero.
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
//! Strings that overflow are broken down into 255 chunks and allocated into a chain of blocks.
//!
//! ### Safety
//!
//! AtomicStrings are synchronized internally and follow the RwLock philosophy where everyone can
//! read at any time, but only one process/thread can write at any given time. Therefore, they are
//! not considered trully lock-free in the general sense, but they will also not spin lock your
//! process/thread uppon trying to manipulate it, returning an error instead of locking.

use std::{ array, sync::atomic::{ AtomicU8, AtomicU32, AtomicUsize, Ordering } };
use crate::{ helpers::type_guard::TypeGuardMut, prelude::* };

/// Marks a string as IDLE, so writers know when its available for writing
const IDLE: u8 = 0;
/// Marks a string as READING, so writers don't overwrite the string when its being accessed
const READING: u8 = 1;
/// Marks a string as WRITING, so readers don't access a partialy written string
const WRITING: u8 = 2;
/// Sets the head block as an AtomicString block
const STATE_STRING: u32 = 900_000;
/// Sets a string link as an StringChain block
const STRING_CHAIN: u32 = 900_001;

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
    str: [char; 255],
    next_overflow: AtomicU32,
    this_offset: u32,
}

/// StringChain struct
/// 
/// A partial representation of the AtomicString that holds only the char contents and the offset
/// to the next block in the chain.
#[derive(SafeSHM)]
pub struct StringChain {
    str: [char; 255],
    next_overflow: AtomicU32,
}

impl AtomicString {
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
    ) -> &'static AtomicString {
        let mut as_block = s_handler.allocate::<AtomicString>().unwrap();
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
            let char_arr: [char; 255] = array::from_fn(|_| chars.next().unwrap_or('\0'));

            if is_head {
                let atomic_string = AtomicString {
                    state: AtomicU8::new(IDLE),
                    readers: AtomicU32::new(0),
                    agreement: AtomicU32::new(agreement_to_apply),
                    curr_agreed: AtomicU32::new(0),
                    string_length: AtomicUsize::new(value.chars().count()),
                    str: char_arr,
                    next_overflow: AtomicU32::new(0),
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
                    .unwrap();

                is_head = false;
            } else {
                let string_chain = StringChain {
                    str: char_arr,
                    next_overflow: AtomicU32::new(0),
                };
                let mut chain_block = s_handler.allocate::<StringChain>().unwrap();

                if let Some(ref mut previous) = prev {
                    let prev_ref = s_handler.read_mut(previous).unwrap();
                    prev_ref.next_overflow.store(chain_block.header_offset(), Ordering::Release);
                } else {
                    let as_ref = s_handler.read_mut::<AtomicString>(&mut as_block).unwrap();
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
                    .unwrap();

                prev = Some(chain_block);
            }

            if chars.peek().is_none() {
                break;
            }
        }

        let str_guard = s_handler.read(&as_block).unwrap();
        let str: &Self = unsafe { &*(&*str_guard as *const Self) };

        return str;
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
        let mut chars = value.chars().peekable();
        let mut is_head = true;
        let mut prev_block: Option<Block<StringChain>> = None;
        let mut next_link = 0;

        if self.state.compare_exchange(IDLE, WRITING, Ordering::SeqCst, Ordering::Relaxed).is_ok() {
            loop {
                let char_arr: [char; 255] = array::from_fn(|_| chars.next().unwrap_or('\0'));

                if is_head {
                    next_link = self.next_overflow.swap(0, Ordering::Release);

                    self.str = char_arr;
                    self.string_length.store(value.chars().count(), Ordering::Release);

                    is_head = false;
                } else {
                    let mut chain_block: Block<StringChain>;
                    let string_chain: TypeGuardMut<StringChain>;
                    let new_string_chain = StringChain {
                        str: char_arr,
                        next_overflow: AtomicU32::new(0),
                    };

                    if next_link != 0 {
                        chain_block = s_handler.get_block::<StringChain>(next_link).unwrap();
                        string_chain = s_handler.read_mut(&chain_block).unwrap();
                        next_link = string_chain.next_overflow.swap(0, Ordering::Release);
                    } else {
                        chain_block = s_handler.allocate::<StringChain>().unwrap();
                    }

                    if let Some(ref mut prev) = prev_block {
                        let prev_ref = s_handler.read_mut::<StringChain>(&prev).unwrap();
                        prev_ref.next_overflow.store(
                            chain_block.header_offset(),
                            Ordering::Release
                        );
                    } else {
                        self.next_overflow.store(chain_block.header_offset(), Ordering::Release);
                    }

                    unsafe {
                        s_handler.write(&mut chain_block, new_string_chain).unwrap();
                    }
                    s_handler.set_state(&chain_block, STRING_CHAIN).unwrap();

                    prev_block = Some(chain_block);
                }

                if chars.peek().is_none() {
                    break;
                }
            }
        } else {
            return None;
        }

        while next_link != 0 {
            let mut extra_block = s_handler.get_block::<StringChain>(next_link).unwrap();
            let extra_str = s_handler.read_mut(&mut extra_block).unwrap();
            next_link = extra_str.next_overflow.load(Ordering::Acquire);
            s_handler.free(extra_block).unwrap();
        }

        self.state.store(IDLE, Ordering::Release);

        return Some(());
    }

    /// Reads an string back from the struct.
    /// 
    /// The follows the overflow chain there is no more links to load from.
    /// 
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations.
    /// 
    /// ### Returns
    /// An optional string loaded from the AtomicString segment.
    pub fn read(&self, s_handler: SharedHandler) -> Option<String> {
        let mut is_head = true;
        let mut str_buffs: Vec<String> = Vec::new();
        let mut next_link = 0;

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
                    let next_chain = s_handler.get_block::<StringChain>(next_link).unwrap();
                    let str_ref = s_handler.read(&next_chain).unwrap();
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

            self.readers.fetch_update(Ordering::Release, Ordering::Acquire, |mut v| {
                let agreement = self.agreement.load(Ordering::Acquire);
                let curr_agreed = self.curr_agreed.load(Ordering::Acquire);
                v -= 1;

                if agreement > 0 && curr_agreed == agreement {
                    self.curr_agreed.store(0, Ordering::Release);
                    self.state.store(IDLE, Ordering::Release);
                } else if v == 0 {
                    self.state.store(IDLE, Ordering::Release);
                }

                return Some(v)
            }).unwrap();
        } else {
            return None
        }

        let final_str = str_buffs.join("");
        
        return Some(final_str)
    }

    /// Agrees to the reading of a string.
    /// 
    /// This increases the counter to finaly release a string as IDLE to the writers.
    pub fn agree(&self) {
        self.curr_agreed.fetch_add(1, Ordering::Release);
    }

    pub fn chars(&self, s_handler: SharedHandler) -> Option<Vec<char>> {
        if let Some(string) = self.read(s_handler) {
            return Some(string.chars().collect::<Vec<char>>())
        } else {
            return None
        }
    }

    /// Returns the length of the current string.
    pub fn len(&self) -> usize {
        return self.string_length.load(Ordering::Acquire)
    }

    /// Returns the offset to the AtomicString head inside the matrix.
    pub fn offset(&self) -> u32 {
        return self.this_offset
    }
}
