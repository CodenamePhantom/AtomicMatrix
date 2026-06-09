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

use crate::helpers::type_tag::fnv1a_32;
use crate::internals::error_collection::{CartographerErrors, MatrixErrors};
use crate::prelude::*;

use std::collections::hash_map::DefaultHasher;
use std::ffi::CString;
use std::hash::{Hash, Hasher};
use std::sync::atomic::{AtomicU32, Ordering};

pub const STATE_CALLBACK: u32 = 4;

/// Config enum to separate setups for SHM or MemFd based arenas
///
/// The engine also use these as flags to fail functions that are specific to a type of setup.
/// (e.g.: the create file function is fully designed to spawn file based SHM arenas. So, if a MemFd
/// config was assembled in the builder, this function will return an error)
pub enum CartographerConfig {
    Shm(CartographerShm),
    MemFd(CartographerMemFd),
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
    pub fn create_file(&self) -> Result<i32, CartographerErrors> {
        let config = match &self.config {
            CartographerConfig::Shm(v) => v,
            _ => return Err(CartographerErrors::InvalidFileTypeCall),
        };

        let name = match &config.uuid {
            Some(v) => CString::new(format!("/dev/shm/matrix-{}", v)).unwrap(),
            None => CString::new(format!("/dev/shm/matrix-{}", uid_lite::generate_uuid())).unwrap(),
        };
        let size = match config.size {
            LinkMethod::New(s) => s as i64,
            LinkMethod::Attach => unreachable!(),
        };

        if config.size == LinkMethod::Attach {
            let file = unsafe { libc::shm_open(name.as_ptr(), libc::O_RDWR, config.fs_permission) };

            if file < 0 {
                return Err(CartographerErrors::FileCreationError);
            } else {
                return Ok(file);
            }
        } else {
            let file = unsafe {
                libc::shm_open(
                    name.as_ptr(),
                    libc::O_RDWR | libc::O_CREAT,
                    config.fs_permission,
                )
            };

            if file < 0 {
                return Err(CartographerErrors::FileCreationError);
            }

            unsafe {
                if libc::ftruncate(file, size) == -1 {
                    libc::shm_unlink(name.as_ptr() as *const i8);
                    return Err(CartographerErrors::FileTruncateError);
                }

                libc::fchown(file, config.fs_uid, config.fs_gid);
            }

            return Ok(file);
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
    /// ### Params:
    /// @base_ptr: The offset to the beginning of the matrix in the current process.
    ///
    /// ### Returns:
    /// Either a MprotectKey, or a [`CartographerErrors`]:
    ///
    /// - [CartographerErrors::RuntimeProtError]: Failed to activate mprotect at the requested
    /// matrix file.
    pub fn rolling_runtime_protection(base_ptr: *const u8) -> Result<(), CartographerErrors> {
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
            return Err(CartographerErrors::RuntimeProtError(
                "Unable to activate mprotect for blocks at initialization".into(),
            ));
        };

        Ok(())
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
    /// Either the pointer to the mmap or a [`CartographerErrors`]:
    ///
    /// - [`CartographerErrors::InvalidFileTypeCall`]: Tried to call an file based function on an
    /// MemFd defined configuration.
    /// - [`CartographerErrors::MmapError`]: libc::mmap failed to map the fd into virtual memory.
    pub fn mmap_memory(&self, fd: i32) -> Result<*mut u8, CartographerErrors> {
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
            _ => return Err(CartographerErrors::InvalidFileTypeCall),
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
            return Err(CartographerErrors::MmapError);
        }

        Ok(ptr as *mut u8)
    }

    /// Deploys the matrix structures at the metadata section at initialization.
    ///
    /// It exchanges the init guard flag at the first 16 bytes of the segment from SYS_UNITIALIZED
    /// to SYS_FORMATTING, then it lays the matrix struct after the init flag, executes the
    /// before_init callback with the matrix as argument and returns the constructed [`AtomicMatrix`]
    /// to be used. This is the last function to be called in the pipeline, as it releases the use
    /// of the matrix for other processes.
    ///
    /// ### Params:
    /// @base_ptr: The offset to the beginning of the matrix in the current process.
    ///
    /// ### Returns:
    /// Either the constructed matrix, or a [`CartographerErrors`]:
    ///
    /// - [`CartographerErrors::InvalidFileTypeCall`]: Tried to run a file based function with an
    /// MemFd defined configuration.
    /// - [`CartographerErrors::WhyWouldYouDoThat`]: Because why would you try to implement the
    /// metadata in am Attach configuration?
    /// - [`CartographerErrors::SysInitializedError`]: The system is already, or is being
    /// initialized by another process.
    pub fn sys_implement(
        &mut self,
        base_ptr: *const u8,
    ) -> Result<&'static AtomicMatrix, CartographerErrors> {
        let size = match &self.config {
            CartographerConfig::Shm(c) => match c.size {
                LinkMethod::New(s) => s as usize,
                LinkMethod::Attach => {
                    return Err(CartographerErrors::WhyWouldYouDoThat(
                        "Really... Why?".into(),
                    ));
                }
            },
            CartographerConfig::MemFd(_) => return Err(CartographerErrors::InvalidFileTypeCall),
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

            if let Some(callback) = self.before_init.take() {
                callback(matrix);
            }

            init_guard.store(SYS_READY, Ordering::SeqCst);
        } else {
            return Err(CartographerErrors::SysInitializedError);
        }

        unsafe { Ok(&mut *matrix_ptr) }
    }

    pub fn sys_attach_callback(&mut self, base_ptr: *const u8) -> Result<(), CartographerErrors> {
        let build_id = fnv1a_32(env!("CARGO_PKG_VERSION").as_bytes());

        match &self.config {
            CartographerConfig::Shm(_) => {
                let attach_callback_offset = unsafe {
                    base_ptr.add(16 as usize + std::mem::size_of::<AtomicMatrix>()) as usize
                };
                let callback_rel_ptr = RelativePtr::<u8>::new(attach_callback_offset as u32);
                let h = unsafe { callback_rel_ptr.resolve_header(base_ptr) };

                if h.state.load(Ordering::Relaxed) == STATE_CALLBACK {
                    let callback_flag =
                        unsafe { &*(*callback_rel_ptr.resolve(base_ptr) as *const AtomicU32) };
                    let callback_magic = unsafe {
                        &*(*callback_rel_ptr.resolve(base_ptr.add(16)) as *const AtomicU32)
                    };
                    let callback = unsafe {
                        *callback_rel_ptr.resolve(base_ptr.add(32)) as *const fn(&mut AtomicMatrix)
                    };

                    if callback_flag.load(Ordering::Relaxed) != build_id
                        || callback_magic.load(Ordering::Relaxed) != 0xCADEBABEDEADBEEF
                    {
                        return Err(CartographerErrors::InvalidCrossProcess);
                    };

                    self.attach_callback = unsafe { Some(*callback) };
                } else {
                    return Err(CartographerErrors::NotACallback);
                };

                Ok(())
            }
            CartographerConfig::MemFd(_) => {
                return Err(CartographerErrors::InvalidFileTypeCall);
            }
        }
    }

    pub fn sys_attach(&self, base_ptr: *const u8) -> &'static AtomicMatrix {
        let retry_count: u32 = 0;

        let init_guard = unsafe { &*(base_ptr as *const AtomicU32) };
        let matrix = unsafe { base_ptr.add(16) as *mut AtomicMatrix };

        while init_guard
            .load_if_any(&[SYS_UNINITIALIZED, SYS_FORMATTING], Ordering::Acquire)
            .is_ok()
        {
            if retry_count <= 512 {
                retry_count.checked_add(1);
                std::hint::spin_loop();
            } else {
                panic!("Unable to connect to provided matrix!");
            }
        }

        return unsafe { &mut *matrix };
    }

    pub fn default_run(&self) -> Result<MatrixHandler, MatrixErrors> {
        unimplemented!()
    }
}
