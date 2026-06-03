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

use crate::core::cartographer_builder::*;
use std::os::fd::RawFd;

pub struct CartographerConfig {
    pub size: LinkMethod,
    pub uuid: Option<String>,
    pub role: ProcessRole,
    pub app_protection: bool,
    pub app_protection_list: Vec<String>,
    pub fs_permission: u32,
    pub fs_uid: u32,
    pub fs_gid: u32,
    pub runtime_protected: bool,
    pub memfd: bool,
    pub memfd_fd: Option<RawFd>,
    pub memfd_seal_write: bool,
    pub defer: bool,
}

pub struct Cartographer {
    pub(crate) config: CartographerConfig,
    pub(crate) before_init: Option<Box<dyn FnOnce()>>,
}

impl Cartographer {
    // Hehe
    pub fn new_from(config: CartographerBuilder) -> Self {
        let cartographer = config.build().unwrap();

        return cartographer;
    }
}
