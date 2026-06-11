//! # Cartographer SHM engine.
//!
//! This extends the caller abilities over SHM file creation with buildable opt-ins that can be
//! toggled through a [`CartographerBuilder`].
//!
//! ### Initialization.
//!
//! There are two main ways that a [`Cartographer`] can be initialized:
//!
//! - Calling the `build()` function inside CartographerBuilder.
//! - Calling `new_from()` in Cartographer passing a CartographerBuilder instance.
//!
//! Whatever method is choosen to create the SHM initialization pipeline, an instance of
//! Cartographer is required to initialize the matrix.
//!
//! ### Opt-ins.
//!
//! Cartographer ships with a default pipeline implementation out of the box that can be used for
//! most cases, but the pipeline is designed to be highly configurable:
//!
//! - Basic configurations such as size, file uuid name and process role.
//! - App protection configurations for AppArmor and SELinux.
//! - File system configuration options for permission and user/group ownerships.
//! - Runtime page PROT_READ|PROT_WRITE flags for sector ownership.
//! - Memfd sealing and file descriptor attachment.
//! - Complex initialization procedures like initialization deferral and pre SYS_READY callbacks.

use crate::internals::error_collection::CartographerErrors;
use crate::prelude::*;
use memmap2::MmapMut;
use std::ffi::CString;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

pub const STATE_CALLBACK: u32 = 4;
pub const CALLBACK_MAGIC: u64 = 0xCADEBABEDEADBEEF;

/// Config enum to separate setups for SHM or MemFd based arenas
///
/// The engine also use these as flags to fail functions that are specific to a type of setup.
/// (e.g.: the create file function is fully designed to spawn file based SHM arenas. So, if a MemFd
/// config was assembled in the builder, this function will return an error)
pub enum CartographerConfig {
    Shm(CartographerShm),
    MemFd(CartographerMemFd),
}

#[repr(C)]
struct CallbackBlock {
    flag: AtomicU32,
    _pad: u32,
    magic: AtomicU64,
    callback: fn(&mut AtomicMatrix),
}

/// SHM based configuration template
pub(crate) struct CartographerShm {
    pub size: LinkMethod,
    pub uuid: Option<String>,
    pub role: ProcessRole,
    pub app_protection: bool,
    pub app_protection_list: Vec<String>,
    pub fs_permission: u32,
    pub fs_uid: u32,
    pub fs_gid: u32,
    pub runtime_protected: bool,
    pub defer: bool,
}

pub(crate) struct CartographerMemFd {}

/// Cartographer struct for SHM arena deployment
///
/// This struct encapsulates the builded configuration for the arena deployment and all functions
/// required to execute the operation. Although a default_run procedure is provided to execute the
/// deployment with ease, all functions related to the pipeline are public and can be executed as
/// deemed by the caller, providing extensive granularity.
pub struct Cartographer {
    pub(crate) config: CartographerConfig,
    pub(crate) base_ptr: Option<*const u8>,
    pub(crate) mmap: Option<*mut u8>,
    pub(crate) matrix: Option<&'static mut AtomicMatrix>,
    pub(crate) before_init: Option<Box<dyn FnOnce(&mut AtomicMatrix)>>,
    pub(crate) attach_callback: Option<fn(&mut AtomicMatrix)>,
}

impl Cartographer {
    /// Creates a new [`Cartographer`] struct from a [`CartographerShmBuilder`] instance.
    ///
    /// ### Params:
    /// @config: The CartographerShmBuilder instance to derivate the configuration from.
    ///
    /// ### Returns:
    /// An instance of Self.
    // Hehe
    pub fn new_shm_from(config: CartographerShmBuilder) -> Self {
        let cartographer = config.build();

        return cartographer;
    }

    /// Creates or attach to a new file based SHM arena.
    ///
    /// If the configured method is attach, calls [`libc::shm_open`] with RDWR flag. If it's New(s),
    /// get the size from the enum option, create the file and truncates it to the requested size.
    /// Both branches attaches the configured file permission at shm_open, although attach calls
    /// don't change the initially defined fs_permission.
    ///
    /// ### Returns:
    /// Either the file descriptor, or a [`CartographerErrors`]:
    ///
    /// - [`CartographerErrors::InvalidFileTypeCall`]: tried to call an file based function on an
    /// MemFd defined configuration.
    /// - [`CartographerErrors::FileCreationError`]: Failed to open or attach to the SHM file.
    /// - [`CartographerErrors::FileTruncateError`]: Failed to truncate the SHM file size.
    pub fn create_file(&self) -> BetterResult<i32, CartographerErrors> {
        let config = match &self.config {
            CartographerConfig::Shm(v) => v,
            _ => {
                return BetterResult::fail(CartographerErrors::InvalidFileTypeCall {
                    file_type: "MemFd".into(),
                });
            }
        };
        let attach_type = &config.size;

        let matrix_name = match &config.uuid {
            Some(v) => CString::new(format!("/dev/shm/matrix-{}\0", v)).unwrap(),
            None => {
                if attach_type == &LinkMethod::Attach {
                    return BetterResult::fail(CartographerErrors::InvalidCrossProcessCall {
                        reason: "Can attach to an existing matrix without providing UUID.".into(),
                    });
                } else {
                    CString::new(format!("/dev/shm/matrix-{}\0", uid_lite::generate_uuid()))
                        .unwrap()
                }
            }
        };
        let size = match attach_type {
            LinkMethod::New(s) => *s as i64,
            LinkMethod::Attach => unreachable!(),
        };

        if attach_type == &LinkMethod::Attach {
            let file =
                unsafe { libc::shm_open(matrix_name.as_ptr(), libc::O_RDWR, config.fs_permission) };

            if file < 0 {
                return BetterResult::fail(CartographerErrors::FileCreationError { err_no: file });
            } else {
                return BetterResult::succeed(file);
            }
        } else {
            let file = unsafe {
                libc::shm_open(
                    matrix_name.as_ptr(),
                    libc::O_RDWR | libc::O_CREAT,
                    config.fs_permission,
                )
            };

            if file < 0 {
                return BetterResult::fail(CartographerErrors::FileCreationError { err_no: file });
            }

            unsafe {
                if libc::ftruncate(file, size) == -1 {
                    libc::shm_unlink(matrix_name.as_ptr() as *const i8);
                    return BetterResult::fail(CartographerErrors::FileTruncateError {
                        err_no: file,
                    });
                }

                libc::fchown(file, config.fs_uid, config.fs_gid);
            }

            return BetterResult::succeed(file);
        }
    }

    /// Enables the initial runtime page protection with [`libc::mprotect`] syscall.
    ///
    /// This function enables memory page write protection at runtime and returns the MprotectKey.
    /// Processes can use this key to temporarely unlock the arena to write infomation and lock it
    /// again at the end of the call. All blocks from the beginning of the matrix to the last used
    /// block are measured to size the initial lock. This size is then padded to default page sizes
    /// (4kb) and passed to mprotect. If the padded size exceeds the total size of the matrix, the
    /// function will fail.
    ///
    /// ### Returns:
    /// Either a MprotectKey, or a [`CartographerErrors`]:
    ///
    /// - [CartographerErrors::RuntimeProtError]: Failed to activate mprotect at the requested
    /// matrix file.
    ///
    // #TODO: implement MprotectKey
    pub fn rolling_runtime_protection(&self) -> BetterResult<(), CartographerErrors> {
        let base_ptr = self.base_ptr.unwrap();
        let start_address = unsafe { base_ptr.add(16 + std::mem::size_of::<AtomicMatrix>()) };

        let current_address = start_address;
        let mut prot_size = 0;

        loop {
            let rel_ptr = RelativePtr::<u8>::new(current_address as u32 + HEADER_SPACE);
            let header = unsafe { rel_ptr.resolve_header(base_ptr) };

            if header.state.load(Ordering::Acquire) == STATE_FREE {
                break;
            }

            let size = header.size.load(Ordering::Acquire);

            prot_size += size;
            unsafe { current_address.add(size as usize) };
        }

        let aligned_size = (prot_size as usize + 4096 - 1) & !(4096 - 1);

        let res = unsafe {
            libc::mprotect(
                base_ptr as *mut libc::c_void,
                aligned_size as libc::size_t,
                libc::PROT_READ,
            )
        };

        if res < 0 {
            return BetterResult::fail(CartographerErrors::RuntimeProtError {
                reason: "Unable to activate mprotect for blocks at initialization".into(),
            });
        };

        BetterResult::succeed(())
    }

    /// Detects if the current app contains a valid app protection system.
    ///
    /// #TODO: separate between apparmor and SELinux.
    pub fn app_protection_detect(path: &str, validation: &str) -> Option<bool> {
        let app_path = CString::new(path).unwrap();
        let app_fd = unsafe { libc::open(app_path.as_ptr(), libc::O_RDONLY) };

        if app_fd >= 0 {
            let mut buf: [u8; 1] = [0];
            unsafe {
                libc::read(app_fd, buf.as_mut_ptr() as *mut libc::c_void, 1);
                libc::close(app_fd);
            }
            return Some(buf[0] == validation.as_bytes()[0]);
        } else {
            return None;
        }
    }

    pub fn app_armor_setup(&self) -> Result<(), CartographerErrors> {
        unimplemented!()
    }

    pub fn selinux_setup(&self) -> Result<(), CartographerErrors> {
        unimplemented!()
    }

    /// Creates a MMAP pointer to the requested SHM file descriptor.
    ///
    /// First, it gets the size of the SHM file, either from the value attached in new, or from the
    /// reuqested fd with [`libc::fstat`] and [`std::mem::zeroed`]. Then it defines the permission
    /// flag based on the provided [`ProcessRole`] (PROT_READ for Reader or PROT_READ | PROT_WRITE
    /// for Writer and above). Finally, it calls [`libc::mmap`] at the file descriptor and returns
    /// the pointer as a `*mut u8`.
    ///
    /// ### Params:
    /// @fd: The file descriptor of the SHM file to map.
    ///
    /// ### Returns:
    /// A result containing either Self, or a [`CartographerErrors`]:
    ///
    /// - [`CartographerErrors::MmapError`]: libc::mmap failed to map the fd into virtual memory.
    #[must_use = "Instance of Cartographer must be retrieved."]
    pub fn mmap_memory(mut self, fd: i32) -> BetterResult<Self, CartographerErrors> {
        let (size, role) = match &self.config {
            CartographerConfig::Shm(c) => {
                let size = match c.size {
                    LinkMethod::New(s) => s as usize,
                    LinkMethod::Attach => {
                        let mut stat = unsafe { std::mem::zeroed::<libc::stat>() };
                        unsafe { libc::fstat(fd, &mut stat) };
                        stat.st_size as usize
                    }
                };
                let role = &c.role;

                (size, role)
            }
            _ => unimplemented!(),
        };

        let permission = if role == &ProcessRole::Reader {
            libc::PROT_READ
        } else {
            libc::PROT_READ | libc::PROT_WRITE
        };

        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                permission,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };

        if ptr == libc::MAP_FAILED {
            return BetterResult::fail(CartographerErrors::MmapError {
                err_no: ptr as i32,
                call: "mmap_memory".into(),
            });
        }

        self.mmap = Some(ptr as *mut u8);
        self.base_ptr = Some(ptr as *const u8);

        BetterResult::succeed(self)
    }

    /// Allocates the attach callback function at the matrix.
    ///
    /// It first validates if an attach_callback was provided at building, then it allocates a
    /// [`CallbackBlock`] at the matrix (ideally the first block), sets up the callback structure
    /// and returns the writing confirmation.
    ///
    /// ### Returns:
    /// Either the writing confirmation, or a [`CartographerErrors`]:
    ///
    /// - [`CartographerErrors::NotACallback`]: A callback was not provided at building.
    /// - [`CartographerErrors::MmapError`]: Unable to allocate the callback at the matrix.
    pub fn sys_callback_implement(&mut self) -> BetterResult<(), CartographerErrors> {
        let base_ptr = self.base_ptr.unwrap();
        let matrix = match &self.matrix {
            Some(v) => v,
            None => return BetterResult::fail(CartographerErrors::InvalidCrossProcessCall {
                reason: "Can't implement callback without either constructing, or implementing the matrix first".into()
            }),
        };

        let build_id = type_tag::fnv1a_32(env!("CARGO_PKG_VERSION").as_bytes());

        let callback = match self.attach_callback {
            Some(v) => v,
            None => {
                return BetterResult::fail(CartographerErrors::CallbackError {
                    reason: "Can't implement attach_callback without a callback being provided."
                        .into(),
                });
            }
        };

        let block_size = std::mem::size_of::<CallbackBlock>() as u32;
        let (block, header) = matrix.allocate(base_ptr, block_size)
            .catch_cast::<_, CartographerErrors>(|e| return BetterResult::fail(CartographerErrors::Inner(e)))
            .finally_as::<_, (&mut CallbackBlock, &mut BlockHeader)>(|v| {
                let rel_ptr = RelativePtr::<CallbackBlock>::new(v.offset());
                
                unsafe {
                    let block = rel_ptr.resolve_mut(base_ptr);
                    let header = rel_ptr.resolve_header_mut(base_ptr);

                    (block, header)
                }
            });

        header.state.store(STATE_CALLBACK, Ordering::Release);
        block.flag.store(build_id, Ordering::Release);
        block.magic.store(CALLBACK_MAGIC, Ordering::Release);
        block.callback = callback;

        BetterResult::succeed(())
    }

    /// Deploys the matrix structures at the metadata section at initialization.
    ///
    /// It exchanges the init guard flag at the first 16 bytes of the segment from SYS_UNITIALIZED
    /// to SYS_FORMATTING, then it lays the matrix struct after the init flag,and returns the
    /// constructed [`AtomicMatrix`] to be used. This method **DO NOT** release the matrix, just
    /// switches the init_guard to SYS_FORMATTING. To release the matrix, call `sys_release()`
    ///
    /// ### Returns:
    /// A result containing either Self, or a [`CartographerErrors`]:
    ///
    /// - [`CartographerErrors::InvalidFileTypeCall`]: Tried to run a file based function with an
    /// MemFd defined configuration.
    /// - [`CartographerErrors::WhyWouldYouDoThat`]: Because why would you try to implement the
    /// metadata in am Attach configuration?
    /// - [`CartographerErrors::SysInitializedError`]: The system is already, or is being
    /// initialized by another process.
    #[must_use = "Instance of Cartographer must be retrieved."]
    pub fn sys_implement(mut self) -> BetterResult<Self, CartographerErrors> {
        let base_ptr = self.base_ptr.unwrap();
        let size = match &self.config {
            CartographerConfig::Shm(c) => match c.size {
                LinkMethod::New(s) => s as usize,
                LinkMethod::Attach => {
                    return BetterResult::fail(CartographerErrors::WhyWouldYouDoThat);
                }
            },
            CartographerConfig::MemFd(_) => unimplemented!(),
        };
        let init_guard = unsafe { &*(base_ptr as *const AtomicU32) };
        let matrix_ptr = unsafe { base_ptr.add(16) as *mut AtomicMatrix };
        let current_offset: u32;

        if init_guard
            .compare_exchange(
                SYS_UNINITIALIZED,
                SYS_FORMATTING,
                Ordering::SeqCst,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            let matrix = AtomicMatrix::init(matrix_ptr, size as u32);

            let matrix_size = std::mem::size_of::<AtomicMatrix>();
            current_offset = (16 + (matrix_size as u32) + 15) & !15;
            let remaining_size = size - (current_offset as usize);

            let (fl, sl) = Mapping::find_indices(remaining_size as u32);

            let header: &mut BlockHeader;
            unsafe {
                header = &mut *(base_ptr.add(current_offset as usize) as *mut BlockHeader);
            }

            header.size.store(remaining_size as u32, Ordering::Release);
            header.state.store(STATE_FREE, Ordering::Release);
            header.prev_phys.store(0, Ordering::Release);
            header.next_free.store(0, Ordering::Release);
            header.created_at.set_now();

            matrix.insert_free_block(base_ptr, current_offset, fl, sl);
            matrix
                .sector_boundaries
                .store(size as u32, Ordering::Release);
        } else {
            return BetterResult::fail(CartographerErrors::SysInitializedError);
        }

        unsafe { self.matrix = Some(&mut *matrix_ptr) };
        BetterResult::succeed(self)
    }

    /// Reconstruct the callback written in the matrix back into the attach_callback property.
    ///
    /// It first queries the first block and checks if it is in callback state. Then it validates
    /// the callback structure (build_id and magic number) to see if the callback is being
    /// reconstructed from a process using the same binary. This ensures that call pointers doesn't
    /// cause a SEGFALT in the vTable. Then it reconstructs the function and attach it to the
    /// attach_callback property.
    ///
    /// ### Returns:
    /// A result containing an empty Ok, or a [`CartographerErrors`]:
    ///
    /// - [CartographerErrors::InvalidCrossProcessCall]: Tried to reconstruct the callback from a
    /// different binary process.
    /// - [`CartographerErrors::NotACallback`]: The first block in the matrix is not a call on
    /// attachment function.
    /// - [`CartographerErrors::InvalidFileTypeCall`]: Tried to call a file based function with a
    /// MemFd defined configuration.
    pub fn sys_attach_callback(&mut self) -> BetterResult<(), CartographerErrors> {
        let base_ptr = self.base_ptr.unwrap();
        let build_id = type_tag::fnv1a_32(env!("CARGO_PKG_VERSION").as_bytes());

        match &self.config {
            CartographerConfig::Shm(_) => {
                let attach_callback_offset = unsafe {
                    base_ptr.add(16 as usize + std::mem::size_of::<AtomicMatrix>()) as usize
                };
                let callback_ptr = RelativePtr::<CallbackBlock>::new(attach_callback_offset as u32);
                let callback = unsafe { callback_ptr.resolve(base_ptr) };
                let h = unsafe { callback_ptr.resolve_header(base_ptr) };

                if h.state.load(Ordering::Relaxed) == STATE_CALLBACK {
                    if callback.flag.load(Ordering::Relaxed) != build_id
                        || callback.magic.load(Ordering::Relaxed) != CALLBACK_MAGIC
                    {
                        return BetterResult::fail(CartographerErrors::InvalidCrossProcessCall {
                            reason: "Can't attach to a callback constructed on a different binary."
                                .into(),
                        });
                    };

                    self.attach_callback = Some(callback.callback);
                } else {
                    return BetterResult::fail(CartographerErrors::CallbackError {
                        reason: "There isn't a callback attached to the matrix.".into(),
                    });
                };

                BetterResult::succeed(())
            }
            CartographerConfig::MemFd(_) => unimplemented!(),
        }
    }

    /// Attach to the SHM metadata.
    ///
    /// It first queries the init_guard and matrix structures from the metadata section, then it
    /// checks if the flag is setted to SYS_READY. If the call passes the checks, it sets the matrix
    /// property on Cartographer and finishes. The system is allowed to retry 512 times before
    /// panicking.
    ///
    /// ### Returns:
    /// An instance of self.
    #[must_use = "Instance of Cartographer must be retrieved."]
    pub fn sys_attach(mut self) -> BetterResult<MatrixHandler, CartographerErrors> {
        let base_ptr = self.base_ptr.unwrap();
        let mut retry_count: u16 = 0;

        let init_guard = unsafe { &*(base_ptr as *const AtomicU32) };
        let matrix = unsafe { base_ptr.add(16) as *mut AtomicMatrix };

        let id: String = match &self.config {
            CartographerConfig::Shm(v) => match &v.uuid {
                Some(v) => v.clone(),
                None => String::from(uid_lite::generate_uuid()),
            },
            CartographerConfig::MemFd(_) => String::from(uid_lite::generate_uuid()),
        };

        let raw = match self.mmap.take() {
            Some(v) => v,
            None => {
                return BetterResult::fail(CartographerErrors::MmapError {
                    err_no: 11,
                    call: "sys_attach".into(),
                });
            }
        };
        let mmap = unsafe { std::ptr::read(raw as *const MmapMut) };

        while init_guard
            .load_if_any(&[SYS_UNINITIALIZED, SYS_FORMATTING], Ordering::Acquire)
            .is_ok()
        {
            if retry_count <= 512 {
                retry_count += retry_count.saturating_add(1);
                std::hint::spin_loop();
            } else {
                panic!("Unable to connect to provided matrix!");
            }
        }

        let handler = MatrixHandler::new(unsafe { &mut *matrix }, mmap, 0, id);

        BetterResult::succeed(handler)
    }

    /// Releases the system for all processes returning a [`MatrixHandler`] in the process.
    ///
    /// First, it gets the configured UUID, the mmap instance cast as MmapMut, and the matrix reference.
    /// Then it calls the before_init callback, and tries to swap the init_guard from SYS_FORMATTING
    /// to SYS_READY. If sucessful, it assembles the MatrixHandler and returns it.
    ///
    /// ### Returns:
    /// A result containing either the MatrixHandler, or a [`CartographerErrors`]:
    ///
    /// - [`CartographerErrors::MmapError`]: no mmap or matrix instance found in Cartographer struct.
    /// - [`CartographerErrors::InvalidCrossProcessCall`]: unable to swap init_guard to SYS_READY.
    pub fn sys_release(&mut self) -> BetterResult<MatrixHandler, CartographerErrors> {
        let base_ptr = self.base_ptr.unwrap();
        let init_guard = unsafe { &*(base_ptr as *mut AtomicU32) };

        let id: String = match &self.config {
            CartographerConfig::Shm(v) => match &v.uuid {
                Some(v) => v.clone(),
                None => String::from(uid_lite::generate_uuid()),
            },
            CartographerConfig::MemFd(_) => String::from(uid_lite::generate_uuid()),
        };

        let raw = match self.mmap.take() {
            Some(v) => v,
            None => {
                return BetterResult::fail(CartographerErrors::MmapError {
                    err_no: 11,
                    call: "sys_release, retrieve raw mmap failed".into(),
                });
            }
        };
        let mmap = unsafe { std::ptr::read(raw as *const MmapMut) };

        let matrix = match self.matrix.take() {
            Some(v) => v,
            None => {
                return BetterResult::fail(CartographerErrors::MmapError {
                    err_no: 12,
                    call: "sys_release, retrieve matrix failed".into(),
                });
            }
        };

        if let Some(callback) = self.before_init.take() {
            callback(matrix)
        };

        if init_guard
            .compare_exchange(
                SYS_FORMATTING,
                SYS_READY,
                Ordering::Release,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            let handler = MatrixHandler::new(matrix, mmap, 0, id);

            BetterResult::succeed(handler)
        } else {
            BetterResult::fail(CartographerErrors::InvalidCrossProcessCall {
                reason: "Failed to swap init_guard to SYS_READY.".into(),
            })
        }
    }

    /// Default cartographer deployment pipeline to be used.
    ///
    /// This provides a clear usable path that ends up returning a [`MatrixHandler`], unless a very
    /// specific problem happens (sys errors, files released or deleted, etc). Callers can opt out
    /// of this implementation and construct their own pipeline by hand, provided they know what
    /// they're doing.
    ///
    /// ### Returns:
    /// An result containing either the MatrixHandler, or a [`CartographerErrors`] related to the
    /// step that failed.
    pub fn default_run(mut self) -> BetterResult<MatrixHandler, CartographerErrors> {
        // Controller variables
        let file_descriptor: i32;

        // Extract pipeline flags from builder config.
        let (app_protection, runtime_protected, defer) = match &self.config {
            CartographerConfig::Shm(v) => {
                file_descriptor = self
                    .create_file()
                    .catch(|e| return BetterResult::fail(e))
                    .finally(|cv| cv);
                (v.app_protection, v.runtime_protected, v.defer)
            }
            CartographerConfig::MemFd(_) => unimplemented!(),
        };

        // App protection deployment. To be implemented.
        if app_protection {
            unimplemented!()
        };

        // Generates mmap.
        self = self
            .mmap_memory(file_descriptor)
            .catch(|e| return BetterResult::fail(e))
            .finally(|v| v);

        // Config deferral checkpoint
        if !defer {
            // Processes that pass execute administrative deployment.
            self = self
                .sys_implement()
                .catch(|e| return BetterResult::fail(e))
                .finally(|v| v);

            if runtime_protected {
                self.rolling_runtime_protection()
                    .chain_up::<CartographerErrors>();
            };

            if self.attach_callback.is_some() {
                self.sys_callback_implement()
                    .chain_up::<CartographerErrors>();
            };

            self.sys_release()
                .catch(|e| return BetterResult::fail(e))
                .finally_return(|v| return BetterResult::succeed(v))
        } else {
            // attach callback is invoked at this stage, provided the process pass the binary check.
            self.sys_attach_callback().chain_up::<CartographerErrors>();
            if let Some(callback) = self.attach_callback.take() {
                match &mut self.matrix {
                    Some(v) => callback(v),
                    None => {
                        return BetterResult::fail(CartographerErrors::CallbackError {
                            reason: "Unable to execute attach_callback function.".into(),
                        });
                    }
                };
            }

            // Processes that fails awaits implementation before attaching.
            self.sys_attach()
                .catch(|e| return BetterResult::fail(e))
                .finally_return(|v| return BetterResult::succeed(v))
        }
    }
}
