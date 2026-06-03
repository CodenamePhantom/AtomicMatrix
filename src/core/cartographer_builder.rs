//! # Cartographer builder.
//! 
// Quick author note:
// May God forgive me for the hacks I´m about to do. But its 2:30 in the morning, and I must sleep.
use crate::core::cartographer::*;
use std::os::fd::RawFd;

pub enum ProcessRole {
    Writer,
    Reader,
    Moderator,
    Owner,
}

pub enum LinkMethod {
    New(u32),
    Attach
}

pub struct CartographerBuilder<F = fn()>
where
    F: FnOnce() + 'static,
{
    // basics
    size: LinkMethod,
    uuid: Option<String>,
    role: ProcessRole,

    // app protection
    app_protection: bool,
    app_protection_list: Vec<String>,

    // file system permissions.
    fs_permission: u32,
    fs_uid: u32,
    fs_gid: u32,

    // Runtime Protection
    runtime_protected: bool,

    // Memfd sealing
    memfd: bool,
    memfd_fd: Option<RawFd>,
    memfd_seal_write: bool,

    // Complex init
    defer: bool,
    before_init: Option<F>,
}

impl CartographerBuilder<fn()> {
    pub fn new(size: LinkMethod) -> Self{ 
        Self {
            size,
            uuid: None,
            role: ProcessRole::Reader,
            app_protection: false,
            app_protection_list: Vec::<String>::new(),
            fs_permission: 0o600,
            fs_uid: 0,
            fs_gid: 0,
            runtime_protected: false,
            memfd: false,
            memfd_fd: None,
            memfd_seal_write: false,
            defer: false,
            before_init: None,
        }
    }
}

impl<F> CartographerBuilder<F> 
where
    F: FnOnce() + 'static,
{

    pub fn uuid(mut self, uuid: String) -> Self {
        self.uuid = Some(uuid);
        self
    }

    pub fn role(mut self, role: ProcessRole) -> Self {
        self.role = role;
        self
    }

    pub fn app_protection(mut self, toggle: bool) -> Self {
        self.app_protection = toggle;
        self
    }

    pub fn app_protection_list(mut self, list: Vec<String>) -> Self {
        self.app_protection_list = list;
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

    pub fn memfd(mut self, is_sealed: bool) -> Self {
        self.memfd = is_sealed;
        self
    }

    pub fn memfd_fd(mut self, file_desc: Option<RawFd>) -> Self {
        self.memfd_fd = file_desc;
        self
    }

    pub fn memfd_seal_write(mut self, is_write_sealed: bool) -> Self {
        self.memfd_seal_write = is_write_sealed;
        self
    }

    pub fn defer(mut self, defer: bool) -> Self {
        self.defer = defer;
        self
    }

    pub fn before_init<NewF>(self, action: NewF) -> CartographerBuilder<NewF> 
    where
        NewF: FnOnce() + 'static,
    {
        CartographerBuilder {
            size: self.size,
            uuid: self.uuid,
            role: self.role,
            app_protection: self.app_protection,
            app_protection_list: self.app_protection_list,
            fs_permission: self.fs_permission,
            fs_uid: self.fs_uid,
            fs_gid: self.fs_gid,
            runtime_protected: self.runtime_protected,
            memfd: self.memfd,
            memfd_fd: self.memfd_fd,
            memfd_seal_write: self.memfd_seal_write,
            defer: self.defer,
            before_init: Some(action),
        }
    }

    pub fn build(self) -> Result<Cartographer, ()> {
        Ok(Cartographer {
            config: CartographerConfig {
                size: self.size,
                uuid: self.uuid,
                role: self.role,
                app_protection: self.app_protection,
                app_protection_list: self.app_protection_list,
                fs_permission: self.fs_permission,
                fs_uid: self.fs_uid,
                fs_gid: self.fs_gid,
                runtime_protected: self.runtime_protected,
                memfd: self.memfd,
                memfd_fd: self.memfd_fd,
                memfd_seal_write: self.memfd_seal_write,
                defer: self.defer
            },
            before_init: self.before_init.map(|f| Box::new(f) as Box<dyn FnOnce()>)
        })
    }
}
