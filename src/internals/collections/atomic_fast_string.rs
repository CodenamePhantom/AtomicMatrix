//! AtomicFastString works as a counterpart for the AtomicString, but faster (duh).
//!
//! AtomicFastString works with the raw bytes from strings into a single block architecture (all
//! the string content is registered into a single block) that is linked into the main static
//! AtomicFastString struct.
//!
//! The surface of the API is way smaller, and provides way less help than AtomicString, making
//! it way more difficult to work with depending on how you need to use it... but its fast tho.
//!
//! ### Architecture diagram
//!
//! ```text
//! [Matrix Block]
//!     |
//!     |-> [AtomicFastString (metadata)(StringBuf pointer)]
//!             |
//!         [Matrix Block]
//!             |
//!             |-> [StringBuf (string contents)]
//!             |
//!             |-> [ShadowString (virtual write; to be commited)]
//! ```
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
//! operation will never corrupt the string halfway through a reading operations. Readers take
//! temporary ownership of the string to ensure it has enough time to copy the string contents to
//! its own heap memory.
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

use std::sync::atomic::{AtomicU32, Ordering};

use crate::core::handlers::MarkedBlocks;
use crate::{
    helpers::{rollback::Rollback, type_guard::UnsizedGuard},
    internals::error_collection::AtomicFSErrors,
    prelude::*,
};

const FAST_STRING: u32 = 100_000;
const BUF_IDLE: u32 = 100_001;
const BUF_READING: u32 = 100_002;
const BUF_RETIRED: u32 = 100_003;

#[derive(SafeSHM)]
pub struct AtomicFastString {
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
    parent_offset: u32,
    handler: SharedHandler,
}

impl AtomicFastString {
    /// Creates a new AtomicFastString inside the matrix.
    ///
    /// The value provided at start is immediatelly written as the current StringBuf in exercice.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix manipulation. \
    /// @value: The value to initialize the string.
    ///
    /// ### Returns
    /// An instance of self.
    pub fn new(s_handler: SharedHandler, value: &str) -> Result<&'static Self, AtomicFSErrors> {
        let str_buf = s_handler
            .allocate_marked::<str>(value)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let mut afs_block = s_handler
            .allocate::<AtomicFastString>()
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let afs_offset = afs_block.header_offset();
        let afs = AtomicFastString {
            pointer: AtomicU32::new(str_buf.header_offset()),
            this_offset: afs_offset,
        };

        s_handler
            .write_transition(
                &mut afs_block,
                afs,
                STATE_ALLOCATED,
                FAST_STRING,
                Ordering::Release,
            )
            .map_err(|e| AtomicFSErrors::StateTransitionFailed {
                state: FAST_STRING,
                reason: e.to_string(),
            })?;

        let afs_guard = s_handler
            .read(&afs_block)
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let afs_ref = unsafe { &*(&*afs_guard as *const Self) };

        return Ok(afs_ref);
    }

    /// Returns an AtomicFastString instance from an already existing block.
    ///
    /// If the offset is not valid for any reason (it doesn't exist, or it's not a valid
    /// AtomicFastString), the function will fail and return None.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix operations. \
    /// @offset: The offset for the AtomicString block.
    ///
    /// ### Returns
    /// An optional static reference to the AtomicFastString.
    pub fn from(s_handler: SharedHandler, offset: u32) -> Result<&'static Self, AtomicFSErrors> {
        let afs_block = s_handler
            .get_block::<AtomicFastString>(offset)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let header = unsafe { afs_block.header() };
        if header.state.load_if(FAST_STRING, Ordering::Acquire).is_err() {
            return Err(AtomicFSErrors::NotAString{ offset: offset });
        }

        let afs_guard = s_handler
            .read(&afs_block)
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let afs_ref = unsafe { &*(&*afs_guard as *const Self) };

        return Ok(afs_ref);
    }

    /// Generates a new ShadowString buffer and return it to the caller.
    ///
    /// ### Params:
    /// @s_handler: A SharedHandler instance for matrix manipulation. \
    /// @value: The value to initialize the string.
    ///
    /// ### Returns
    /// A ShadowString instance.
    pub fn shadow_write(
        &self,
        s_handler: SharedHandler,
        value: &str,
    ) -> Result<ShadowString, AtomicFSErrors> {
        let str_len = value.len();

        let shadow_str_block = s_handler
            .allocate_marked::<str>(value)
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let str_ref = s_handler
            .read_marked(&shadow_str_block)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let mut shadow_str = ShadowString {
            str: str_ref,
            len: str_len,
            rollback: None,
            parent_offset: self.this_offset,
            this_block: shadow_str_block,
            handler: s_handler,
        };

        shadow_str.set_default_rollback();

        return Ok(shadow_str);
    }

    /// Generates a new StringBuf from a snapshot of the AtomicFastString pointer contents.
    ///
    /// StringBufs are fully owned by the caller, and take ownership of the local matrix buffer
    /// for as long as it takes to generate a snapshot of the contents. Subsequent read requests
    /// run a busy loop spin and retry acquiring the read. This loop can occur 512 times before
    /// returning None.
    ///
    /// ### Params
    /// @s_handler: A SharedHandler instance for matrix manipulation.
    ///
    /// ### Returns
    /// An optional instance of StringBuf, if the method managed to successfully generate a
    /// snapshot of the local buffer contents.
    pub fn read(&self, s_handler: SharedHandler) -> Result<StringBuf, AtomicFSErrors> {
        let str_buf_block = s_handler
            .get_marked::<str>(self.pointer.load(Ordering::Acquire))
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let header = unsafe { str_buf_block.header() };

        let mut retry_count = 0;

        loop {
            if header
                .state
                .compare_exchange(BUF_IDLE, BUF_READING, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                let str_ref = s_handler
                    .read_marked(&str_buf_block)
                    .map_err(|e| AtomicFSErrors::Inner(e))?;

                let str_value = &*str_ref;
                let str_len = str_ref.len();

                let str_buf = StringBuf {
                    str: str_value.into(),
                    len: str_len,
                    parent_offset: self.this_offset,
                    handler: s_handler,
                };

                if header.state.load(Ordering::Acquire) == BUF_RETIRED {
                    s_handler
                        .free_marked(str_buf_block)
                        .map_err(|e| AtomicFSErrors::Inner(e))?;
                } else {
                    header.state.store(BUF_IDLE, Ordering::Release);
                }

                return Ok(str_buf);
            } else {
                retry_count += 1;

                if retry_count >= 512 {
                    return Err(AtomicFSErrors::MaxRetryReached);
                } else {
                    std::hint::spin_loop();
                }
            }
        }
    }
}

impl ShadowString {
    /// Sets the default rollback action for failed string operations that can fail and cause this
    /// instance to drop.
    pub fn set_default_rollback(&mut self) {
        let header_offset = self.this_block.header_offset();
        let handler = self.handler;
        self.rollback = Some(Rollback::set(Box::new(move || {
            let header_ptr = RelativePtr::<BlockHeader>::new(header_offset);
            handler.matrix().ack(&header_ptr, handler.base_ptr());
        })));
    }

    /// Updates the current rollback action with a caller provided custom closure.
    ///
    /// ### Params
    /// @func: The closure to execute in case the rollback is triggered.
    pub fn update_rollback<F: FnOnce() + 'static>(&mut self, func: F) {
        match self.rollback {
            Some(ref mut v) => v.update(Box::new(func)),
            None => {
                self.rollback = Some(Rollback::set(Box::new(func)));
            }
        }
    }

    /// Disarms the rollback so it doesn't execute at a drop event.
    pub fn disarm_rollback(&mut self) {
        if let Some(ref mut rollback) = self.rollback {
            rollback.disarm();
        }
    }

    /// Rewrite the entire string and replace the coordinate values in the local struct.
    ///
    /// ### Params
    /// @value: The new string to rewrite.
    ///
    /// ### Returns
    /// An option stating the successful execution of this call.
    pub fn rewrite(mut self, value: &str) -> Result<Self, AtomicFSErrors> {
        let str_len = value.len();

        let shadow_str_block = self
            .handler
            .allocate_marked::<str>(value)
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let str_ref = self
            .handler
            .read_marked(&shadow_str_block)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let old_shadow_str = self.this_block;

        self.this_block = shadow_str_block;
        self.str = str_ref;
        self.len = str_len;

        self.set_default_rollback();
        self.handler
            .free_marked(old_shadow_str)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        return Ok(self);
    }

    /// Commits the shadow string into the head of the parent AtomicFastString.
    ///
    /// The old string can be either deallocated if no one is currently reading, or set to RETIRED
    /// for deferred deallocation by the reader.
    ///
    /// ### Returns
    pub fn commit(mut self) -> Result<(), AtomicFSErrors> {
        let afs_block = self
            .handler
            .get_block::<AtomicFastString>(self.parent_offset)
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let afs_ref = self
            .handler
            .read_mut(&afs_block)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let old_offset = afs_ref
            .pointer
            .swap(self.this_block.header_offset(), Ordering::SeqCst);
        let old_buf = self
            .handler
            .get_block::<StringBuf>(old_offset)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let old_header = unsafe { old_buf.header() };

        if old_header.state.load(Ordering::Acquire) == BUF_READING {
            if old_header
                .state
                .compare_exchange(
                    BUF_READING,
                    BUF_RETIRED,
                    Ordering::Release,
                    Ordering::Relaxed,
                )
                .is_err()
            {
                self.handler
                    .free(old_buf)
                    .map_err(|e| AtomicFSErrors::Inner(e))?;
            }
        } else if old_header.state.load(Ordering::Acquire) == BUF_IDLE {
            self.handler
                .free(old_buf)
                .map_err(|e| AtomicFSErrors::Inner(e))?;
        }

        self.disarm_rollback();
        self.handler
            .set_state_marked(&self.this_block, BUF_IDLE)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        return Ok(());
    }

    /// Commits the shadow string into the head of the parent AtomicFastString.
    ///
    /// Instead of performing an arbitrary action on the old string header, return the whole thing
    /// to the caller for further manipulation.
    ///
    /// ### Returns
    /// An optional BlockHeader.
    pub fn cold_commit(mut self) -> Result<&'static BlockHeader, AtomicFSErrors> {
        let afs_block = self
            .handler
            .get_block::<AtomicFastString>(self.parent_offset)
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let afs_ref = self
            .handler
            .read_mut(&afs_block)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let old_offset = afs_ref
            .pointer
            .swap(self.this_block.header_offset(), Ordering::SeqCst);
        let old_buf = self
            .handler
            .get_block::<StringBuf>(old_offset)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let rel_ptr = old_buf.relative_ptr();
        let old_header = unsafe { rel_ptr.resolve_header(self.handler.base_ptr()) };

        self.disarm_rollback();
        self.handler
            .set_state_marked(&self.this_block, BUF_IDLE)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        return Ok(old_header);
    }

    /// Returns a reference to the current shadow string.
    pub fn str(&self) -> &str {
        &*self.str
    }

    /// Returns the length of the current shadow string.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns a reference to the direct shadow string block.
    pub fn this_block(&self) -> &Block<str> {
        &self.this_block
    }

    /// Prints the current shadow string to stdout.
    pub fn print(&self) {
        println!("{}", &*self.str);
    }
}

impl StringBuf {
    /// Returns the underlying string value for this StringBuf.
    pub fn str(&self) -> &str {
        return &self.str;
    }

    /// Returns the length of the string value for this StringBuf.
    pub fn len(&self) -> usize {
        return self.len;
    }

    /// Converts the current iteration of this StringBuf into a [`ShadowString`].
    ///
    /// Shadow strings can be swapped into the new head of the AtomicFastString.
    ///
    /// ### Returns
    /// A new ShadowString instance, based of this buf values.
    pub fn ctrl_x(self) -> Result<ShadowString, AtomicFSErrors> {
        let str_len = self.str.len();

        let shadow_str_block = self
            .handler
            .allocate_marked::<str>(&self.str)
            .map_err(|e| AtomicFSErrors::Inner(e))?;
        let str_ref = self
            .handler
            .read_marked(&shadow_str_block)
            .map_err(|e| AtomicFSErrors::Inner(e))?;

        let mut shadow_str = ShadowString {
            str: str_ref,
            len: str_len,
            rollback: None,
            parent_offset: self.parent_offset,
            this_block: shadow_str_block,
            handler: self.handler,
        };

        shadow_str.set_default_rollback();

        return Ok(shadow_str);
    }
}
