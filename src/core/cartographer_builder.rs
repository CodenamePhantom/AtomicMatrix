//! # Cartographer builder.
//! 
//! Provides a builder pattern construction for Cartographer configuration.

// Quick author note:
// May God forgive me for the hacks I´m about to do. But its 2:30 in the morning, and I must sleep.
// Update:
// The hacks are no longer!

use crate::prelude::*;
use std::os::fd::RawFd;

#[derive(PartialEq)]
pub enum ProcessRole {
    Writer,
    Reader,
    Moderator,
    Owner,
}

#[derive(PartialEq)]
pub enum LinkMethod {
    New(u32),
    Attach
}

pub struct CartographerBuilder {}

pub struct CartographerShmBuilder {
    // basics
    size: LinkMethod,
    uuid: Option<String>,
    role: ProcessRole,

    // app protection
    apparmor: bool,
    apparmor_confined: bool,
    selinux: bool,
    app_allowed_list: Vec<String>,

    // file system permissions.
    fs_permission: u32,
    fs_uid: u32,
    fs_gid: u32,

    // Runtime Protection
    runtime_protected: bool,

    // Complex init
    defer: bool,
    before_init: Option<Box<dyn FnOnce(&mut AtomicMatrix) + 'static>>,
    attach_callback: Option<fn(&mut AtomicMatrix)>,
}

pub struct CartographerMemFdBuilder {
    // basics
    size: LinkMethod,
    fd: Option<RawFd>,
    fd_broker_addr: String,
    role: ProcessRole,

    // protection
    runtime_protected: bool,
    memfd_secret: bool,
    memfd_seal_write: bool,

    // Complex init
    defer: bool,
    before_init: Option<Box<dyn FnOnce(&mut AtomicMatrix) + 'static>>,
    attach_callback: Option<Box<dyn FnOnce() + 'static>>,
}

impl CartographerBuilder {
    pub fn shm(link: LinkMethod) -> CartographerShmBuilder{ 
        CartographerShmBuilder {
            size: link,
            uuid: None,
            role: ProcessRole::Reader,
            apparmor: false,
            apparmor_confined: false,
            selinux: false,
            app_allowed_list: Vec::<String>::new(),
            fs_permission: 0o600,
            fs_uid: 0,
            fs_gid: 0,
            runtime_protected: false,
            defer: false,
            before_init: None,
            attach_callback: None,
        }
    }

    pub fn memfd(link: LinkMethod, broker_addr: String,) -> CartographerMemFdBuilder {
        CartographerMemFdBuilder { 
            size: link, 
            fd: None, 
            fd_broker_addr: broker_addr,
            role: ProcessRole::Reader, 
            runtime_protected: false,
            memfd_secret: false, 
            memfd_seal_write: false, 
            defer: false, 
            before_init: None, 
            attach_callback: None 
        }
    }
}

impl CartographerShmBuilder {
    pub fn uuid(mut self, uuid: String) -> Self {
        self.uuid = Some(uuid);
        self
    }

    pub fn role(mut self, role: ProcessRole) -> Self {
        self.role = role;
        self
    }

    pub fn apparmor(mut self, toggle: bool) -> Self {
        self.apparmor = toggle;
        self
    }

    pub fn app_allowed_list(mut self, list: Vec<String>) -> Self {
        self.app_allowed_list = list;
        self
    }

    pub fn fs_permission(mut self, permission: u32) -> Self {
        self.fs_permission = permission;
        self
    }

    pub fn fs_uid(mut self, uid: u32) -> Self {
        self.fs_uid = uid;
        self
    }

    pub fn fs_gid(mut self, gid: u32) -> Self {
        self.fs_gid = gid;
        self
    }

    pub fn runtime_protected(mut self, is_protected: bool) -> Self {
        self.runtime_protected = is_protected;
        self
    }

    pub fn defer(mut self, defer: bool) -> Self {
        self.defer = defer;
        self
    }

    pub fn before_init(mut self, action: impl FnOnce(&mut AtomicMatrix) + 'static) -> Self {
        self.before_init = Some(Box::new(action));
        self
    }

    pub fn attach_callback(mut self, action: fn(&mut AtomicMatrix)) -> Self {
        self.attach_callback = Some(action);
        self
    }

    pub fn build(self) -> Cartographer {
        Cartographer {
            config: CartographerConfig::Shm(CartographerShm {
                size: self.size,
                uuid: self.uuid,
                role: self.role,
                app_protection: self.apparmor,
                app_protection_list: self.app_allowed_list,
                fs_permission: self.fs_permission,
                fs_uid: self.fs_uid,
                fs_gid: self.fs_gid,
                runtime_protected: self.runtime_protected,
                defer: self.defer
            }),
            before_init: self.before_init,
            attach_callback: self.attach_callback,
        }
    }
}
