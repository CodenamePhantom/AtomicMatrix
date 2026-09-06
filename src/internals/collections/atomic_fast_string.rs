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
//!             |
//!             |-> [ShadowString (virtual write; to be commited)]
//!
//! The AtomicFastString block works as the enclosure of the string segment and it's never
//! deallocated.
//!
//! ### Runtime mutability
//!
//! New strings that are written into the AtomicFastString never reutilizes an existing block. All
//! writes are allocated in a temporary buffer called ShadowString. ShadowStrings are temporary
//! representations of the next state candidate of the string owned by the current process. These
//! strings have to be commited to the AtomicFastString, which will swap the current pointer and
//! deallocate the old string
//!
//! Reading strings takes a snapshot of the current contents into the reader scope, ensures a write
//! operation will never corrupt the string halfway through a reading operations.
//!
//! Both of these concepts assures a lock-free write/read manipulation of the string, with the
//! tradeoff of not having the strict access hierarchy and ordering AtomicStrings have.
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
//! ShadowStrings are not commited to the metadata unless explicitly stated with the commit()
//! method, although they implement a default RAII guard that will wipe the string if it has not
//! been commited at any point. This RAII can be configured to perform custom business logic
//! on Drop.
//!
//! ### Safety
//!
//! AtomicFastString ensures that all pointer swaps happens atomically in a consistent sequential
//! ordering, but thats it. If your use case requires reading agreements, or string more advanced
//! writting lifecycles, thats on you.
use std::sync::atomic::{ AtomicU8, AtomicU32, Ordering };

use crate::{ helpers::{ rollback::Rollback, type_guard::UnsizedGuard }, prelude::* };

const FAST_STRING: u32 = 100_000;
const BUF_IDLE: u32 = 100_001;
const BUF_READING: u32 = 100_002;
const BUF_RETIRED: u32 = 100_003;

#[derive(SafeSHM)]
pub struct AtomicFastString {
    state: AtomicU8,
    pointer: AtomicU32,
    this_offset: u32,
}

pub struct ShadowString {
    str: UnsizedGuard<str>,
    len: usize,
    rollback: Option<Rollback<Box<dyn FnOnce()>>>,
    parent_offset: u32,
    this_block: Block<str>,
    handler: SharedHandler,
}

pub struct StringBuf {
    str: Box<str>,
    len: usize,
    readers: AtomicU32,
    parent_offset: u32,
    this_block: Block<str>,
    handler: SharedHandler,
}

impl AtomicFastString {
    pub fn new(s_handler: SharedHandler, value: &str) -> &'static Self {
        let str_buf = s_handler.allocate_marked::<str>(value).unwrap();

        let mut afs_block = s_handler.allocate::<AtomicFastString>().unwrap();
        let afs_offset = afs_block.header_offset();
        let afs = AtomicFastString {
            state: AtomicU8::new(0),
            pointer: AtomicU32::new(str_buf.header_offset()),
            this_offset: afs_offset,
        };

        s_handler
            .write_transition(&mut afs_block, afs, STATE_ALLOCATED, FAST_STRING, Ordering::Release)
            .unwrap();

        let afs_guard = s_handler.read(&afs_block).unwrap();
        let afs_ref = unsafe { &*(&*afs_guard as *const Self) };

        return afs_ref;
    }

    pub fn from(s_handler: SharedHandler, offset: u32) -> &'static Self {
        let afs_block = s_handler.get_block::<AtomicFastString>(offset).unwrap();

        let afs_guard = s_handler.read(&afs_block).unwrap();
        let afs_ref = unsafe { &*(&*afs_guard as *const Self) };

        return afs_ref;
    }

    pub fn shadow_write(&self, s_handler: SharedHandler, value: &str) -> ShadowString {
        let str_len = value.len();

        let shadow_str_block = s_handler.allocate_marked::<str>(value).unwrap();
        let str_ref = s_handler.read_marked(&shadow_str_block).unwrap();

        let mut shadow_str = ShadowString {
            str: str_ref,
            len: str_len,
            rollback: None,
            parent_offset: self.this_offset,
            this_block: shadow_str_block,
            handler: s_handler,
        };

        shadow_str.set_default_rollback();

        return shadow_str;
    }

    pub fn read(s_handler: SharedHandler) -> StringBuf {
        unimplemented!()
    }
}

impl ShadowString {
    pub fn set_default_rollback(&mut self) {
        let header_offset = self.this_block.header_offset();
        let handler = self.handler;
        self.rollback = Some(
            Rollback::set(
                Box::new(move || {
                    let header_ptr = RelativePtr::<BlockHeader>::new(header_offset);
                    handler.matrix().ack(&header_ptr, handler.base_ptr());
                })
            )
        );
    }

    pub fn update_rollback<F: FnOnce() + 'static>(&mut self, func: F) {
        match self.rollback {
            Some(ref mut v) => v.update(Box::new(func)),
            None => { self.rollback = Some(Rollback::set(Box::new(func))) },
        }
    }

    pub fn disarm_rollback(&mut self) {
        if let Some(ref mut rollback) = self.rollback {
            rollback.disarm();
        }
    }

    pub fn rewrite(&self, value: &str) -> Option<()> {
        unimplemented!()
    }

    pub fn write_modifier(&self, value: &str) -> Option<()> {
        unimplemented!()
    }

    pub fn modify(&self, value: &str) -> Option<()> {
        unimplemented!()
    }

    pub fn regex(&self, regex_str: &str, flags: &str) -> Option<()> {
        unimplemented!()
    }

    pub fn commit(mut self) -> Option<()> {
        let afs_block = self.handler.get_block::<AtomicFastString>(self.parent_offset).ok()?;
        let afs_ref = self.handler.read_mut(&afs_block).ok()?;

        let old_offset = afs_ref.pointer.swap(self.this_block.header_offset(), Ordering::Acquire);
        let old_buf = self.handler.get_block::<StringBuf>(old_offset).ok()?;

        let header_state = unsafe { old_buf.header() };

        if header_state.state.load(Ordering::Acquire) == BUF_READING {
            if header_state.state.compare_exchange(
                BUF_READING,
                BUF_RETIRED,
                Ordering::Release,
                Ordering::Relaxed
            ).is_err() {
                self.handler.free(old_buf).ok()?;
            };
        } else if header_state.state.load(Ordering::Acquire) == BUF_IDLE {
            self.handler.free(old_buf).ok()?;
        } else {
            self.disarm_rollback();
            return None
        }

        let header_ptr = RelativePtr::<BlockHeader>::new(self.this_block.header_offset());
        self.handler.matrix().ack(&header_ptr, self.handler.base_ptr());
        self.disarm_rollback();

        return Some(());
    }

    pub fn str(&self) -> &str {
        &*self.str
    }

    pub fn len(&self) -> usize {
        self.len
    }
}
