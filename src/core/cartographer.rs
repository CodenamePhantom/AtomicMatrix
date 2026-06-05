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

use crate::internals::error_collection::{CartographerErrors, MatrixErrors};
use crate::prelude::*;

use std::ffi::CString;
use std::ops::Deref;
use std::sync::atomic::{AtomicU32, Ordering};

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
pub struct CartographerShm {
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

pub struct CartographerMemFd {}

pub struct Cartographer {
    pub(crate) config: CartographerConfig,
    pub(crate) before_init: Option<Box<dyn FnOnce()>>,
    pub(crate) attach_callback: Option<Box<dyn FnOnce()>>,
}

impl Cartographer {
    // Hehe
    pub fn new_shm_from(config: CartographerShmBuilder) -> Self {
        let cartographer = config.build();

        return cartographer;
    }

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

    pub fn targeted_runtime_protection(
        &self,
        base_ptr: *const u8,
        targets: &[RelativePtr<u8>],
    ) -> Result<(), CartographerErrors> {
        unimplemented!()
    }

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
                callback();
            }

            init_guard.store(SYS_READY, Ordering::SeqCst);
        } else {
            return Err(CartographerErrors::SysInitializedError);
        }

        unsafe { Ok(&mut *matrix_ptr) }
    }

    pub fn default_run(&self) -> Result<MatrixHandler, MatrixErrors> {
        unimplemented!()
    }
}
