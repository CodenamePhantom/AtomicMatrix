//! # Fencer API.
//!
//! Fencer is a load-balancing/sectorization implementation around the matrix.
//! It consists of a routing struct that is able to hold multiple matrix instances
//! as well as send messages across them. It has a built in ACL map to know which
//! structs have permission to each sector to ensure security and granularity
//! throughout the platform ecosystem.
//!
//! ## Architecture
//!
//! The fencer sits as an agnostic middleware between the matrix arena and the
//! app layer. When a route guard is applied to sectors, each request has to
//! pass through the Fencer Protocol to be granted Read/Write permission to that
//! specific sector. If a load balancer is configured, Fencer will automatically
//! spawn temporary balance sectors to share the workload horizontaly and clear
//! them when the work has stabilized and the space is no longer needed.
//!
//!```text
//! [`App Layer`]       [`Fencer Layer Protocol`]
//!     |                      |          |
//!     |<------------> [`Route Guard`]   |--> &[`AtomicMatrix`](BalanceSect1)
//!     |                                 |--> &[`AtomicMatrix`](BalanceSect2)
//!     |--> &[`AtomicMatrix`](PrivSect1) |--> etc...
//!     |--> &[`AtomicMatrix`](PrivSect2)
//!     |--> etc...
//!```
//!
//! Following the Zero-Copy phylosophy of the underlying layers, only a reference
//! to the requested sector is provided to the callers. Since the matrix is fully
//! thread safe, this refence can be used by multiple processes simultaneously.
//!
//! ## Granularity
//!
//! Configuration of Fencer properties and constraints are completelly delegated
//! to the caller specific implementation structure. This allows Fencer to adapt
//! to multiple edge cases easely, but increases complexity surface, as it requires
//! a better knowledge of the code running under the hood to properly configure
//! the structs. Although, a default config is baked in the constructor is none
//! is provided.
//!
//! ## Public layer
//!
//! This structure comes with a PublicMatrix struct that allows external third party
//! software to plug into your App Layer ecosystem seamlessly, provided the module has
//! the internal data configuration to read/write intelligible data.
//!
//! # Safety
//!
//! This extension is **EXPERIMENTAL** and its not recommended to be used in
//! production code as per the current state of its development. User discretion
//! is advised

use crate::prelude::*;
use std::sync::atomic::{AtomicU32, Ordering};

pub const STATE_FENCER: u32 = 10_001;

pub const MAX_ACL_ENTRIES: usize = 64;
pub const PERM_READ: u8 = 1 << 0;
pub const PERM_WRITE: u8 = 1 << 1;
pub const PERM_OWN: u8 = 1 << 2;

#[derive(Debug)]
pub enum FencerErrors {
    UnauthorizedRead,
    UnauthorizedWrite,
    SectorError(String),
    SectorSpawnError(String),
    SectorAttachingError,
    RoutingError,
    PermissionCheckError(String),
}

#[derive(Clone, Copy)]
pub enum SectorModel {
    Private,
    Public,
    Balancer,
}

pub struct Config {
    pub sector_limit: u32,
    pub balancer_limit: u32,
    pub fencer_id: String,
    pub default_profile: SectorModel,
}

pub struct ACL {
    owner: [u8; 16],
    sector_id: u32,
    version: SectorModel,
    entry_count: u32,
    entries: [Option<ACLEntry>; MAX_ACL_ENTRIES],
}

#[derive(Clone, Copy, PartialEq)]
pub struct ACLEntry {
    module_id: [u8; 16],
    permissions: u8,
}

pub struct Fencer {
    buf_size: u32,
    sect_attached: AtomicU32,
    bal_attached: AtomicU32,
    guard_range: (AtomicU32, AtomicU32),
    config: Config,
}

pub struct RouteGuard<'a> {
    private_id: &'a str,
    fencer_metadata: u32,
    sector_handler: MatrixHandler,
}

pub struct Balancer {}

impl Fencer {
    pub fn new<'a>(cfg: Config) -> (&'a Self, MatrixHandler) {
        let id = String::from("dataplane-".to_owned() + &cfg.fencer_id);
        let fencer_size = std::mem::size_of::<Fencer>() as u32;
        let acl_size = std::mem::size_of::<ACL>() as u32;
        let metadata_session = std::mem::size_of::<AtomicMatrix>() as u32 + 16;
        let util_size = (HEADER_SPACE + fencer_size)
            + ((HEADER_SPACE + acl_size) * (cfg.sector_limit + cfg.balancer_limit));

        let handler =
            AtomicMatrix::bootstrap(Some(id), (metadata_session + util_size) as usize).unwrap();

        let first_h = handler.matrix().query(metadata_session).offset() - HEADER_SPACE;
        let h_struct = unsafe {
            handler
                .matrix()
                .query(metadata_session)
                .resolve_header(handler.base_ptr())
        };

        if h_struct.state.load(Ordering::Relaxed) != STATE_FENCER {
            let block = handler.allocate::<Fencer>().unwrap();
            let routeg_range_1 = AtomicU32::new(block.pointer.offset() + fencer_size);
            let routeg_range_2 = AtomicU32::new(block.pointer.offset() + fencer_size);
            let fencer =
                unsafe { handler.base_ptr().add(block.pointer.offset() as usize) as *mut Fencer };

            h_struct.state.store(STATE_FENCER, Ordering::Release);

            unsafe {
                std::ptr::write(
                    fencer,
                    Fencer {
                        sect_attached: AtomicU32::new(0),
                        bal_attached: AtomicU32::new(0),
                        buf_size: HEADER_SPACE + acl_size,
                        guard_range: (routeg_range_1, routeg_range_2),
                        config: cfg,
                    },
                );
                (&*fencer, handler)
            }
        } else {
            unsafe {
                let fencer =
                    handler.base_ptr().add((first_h + HEADER_SPACE) as usize) as *mut Fencer;

                (&*fencer, handler)
            }
        }
    }

    pub fn spawn_sector<'a>(
        &self,
        dataplane: &MatrixHandler,
        size: usize,
        priv_id: &'a str,
        perms: Vec<ACLEntry>,
    ) -> Result<RouteGuard<'a>, FencerErrors> {
        let sect_amount = self.sect_attached.fetch_add(1, Ordering::Relaxed);
        if sect_amount >= self.config.sector_limit {
            self.sect_attached.fetch_sub(1, Ordering::Relaxed);
            return Err(FencerErrors::SectorSpawnError(
                "Sector limit reached".to_string(),
            ));
        }

        let block = dataplane.allocate::<ACL>().unwrap();

        let id = block.pointer.offset() - HEADER_SPACE;
        let acl = self.generate_acl(id, priv_id, None, perms).unwrap();
        unsafe { block.pointer.write(dataplane.base_ptr(), acl) };
        let current_range_peak = self.guard_range.1.load(Ordering::Acquire);
        if id > current_range_peak {
            self.guard_range.1.store(id, Ordering::Release);
        }

        let handler = AtomicMatrix::bootstrap(Some(id.to_string()), size).unwrap();

        let guard = RouteGuard {
            private_id: priv_id,
            fencer_metadata: block.pointer.offset(),
            sector_handler: handler,
        };

        Ok(guard)
    }

    pub fn register_sector(&self) -> RouteGuard<'_> {
        unimplemented!()
    }

    pub fn generate_acl<'a>(
        &self,
        id: u32,
        priv_id: &'a str,
        prof: Option<SectorModel>,
        perms: Vec<ACLEntry>,
    ) -> Result<ACL, FencerErrors> {
        let owner = priv_id.as_bytes();
        let mut owner_tag = [0; 16];
        for (i, v) in owner.iter().enumerate() {
            if i >= 16 {
                break;
            }

            owner_tag[i] = *v;
        }
        if perms.len() > MAX_ACL_ENTRIES {
            return Err(FencerErrors::SectorSpawnError(
                    format!("Too many ACL entries (max {})", MAX_ACL_ENTRIES)
            ))
        }

        let profile_name = prof.unwrap_or_else(|| self.config.default_profile);

        let mut entries: [Option<ACLEntry>; MAX_ACL_ENTRIES] = [None; MAX_ACL_ENTRIES];
        for (i, v) in perms.iter().enumerate() {
            entries[i] = Some(*v);
        }

        Ok(ACL {
            owner: owner_tag,
            sector_id: id,
            version: profile_name,
            entry_count: perms.len() as u32,
            entries,
        })
    }

    pub fn get_sector<'a>(&self, handler: &MatrixHandler, sec_id: u32, priv_id: &'a str) -> Result<RouteGuard<'a>, FencerErrors> {
        let block = Block::<ACL>::from_offset(handler.matrix().query(sec_id).offset());
        let mut module_tag = [0; 16];

        for (i, v) in priv_id.as_bytes().iter().enumerate() {
            if i >= 16 {
                break;
            }

            module_tag[i] = *v;
        }

        let acl_data = unsafe { handler.read(&block) };

        let entry = acl_data.entries
            .iter()
            .filter_map(|e| e.as_ref())
            .find(|e| e.module_id == module_tag);
        let perms = match entry {
            Some(e) => e.permissions,
            None if acl_data.owner == module_tag => PERM_READ | PERM_WRITE | PERM_OWN,
            None => return Err(FencerErrors::SectorAttachingError)
        };

        if perms & PERM_READ == 0 {
            return Err(FencerErrors::UnauthorizedRead)
        }

        let sect_handler = AtomicMatrix::bootstrap(Some(sec_id.to_string()), 0).unwrap();

        Ok(RouteGuard {
            private_id: priv_id,
            fencer_metadata: block.pointer.offset(),
            sector_handler: sect_handler,
        })
    }
}

impl<'a> RouteGuard<'a> {
    pub fn route_msg(&self) -> Result<(), FencerErrors> {
        unimplemented!()
    }

    pub fn load_msg(&self) -> Result<(), FencerErrors> {
        unimplemented!()
    }

    pub fn clear_sector(&self) {
        unimplemented!()
    }

    pub fn decommission_sector(&self) {
        unimplemented!()
    }

    pub fn check_permissions(&self) -> bool {
        unimplemented!()
    }
}
