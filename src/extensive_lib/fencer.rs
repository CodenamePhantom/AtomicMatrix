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
//! Following the Zero-Copy philosophy of the underlying layers, only a reference
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

use better_result::BetterResult;

use crate::{
    extensive_lib::looper,
    internals::{
        collections::atomic_ringbuffer::*,
        error_collection::FencerErrors,
    },
    prelude::*,
};
use std::sync::atomic::{AtomicU32, Ordering};

// Internal consts for state and permission control
pub const STATE_FENCER: u32 = 10_001;

pub const MAX_ACL_ENTRIES: usize = 64;
pub const PERM_READ: u8 = 1 << 0;
pub const PERM_WRITE: u8 = 1 << 1;
pub const PERM_OWN: u8 = 1 << 2;

/// Model used by the sectors. Could either be Private, Public, or Balancer.
#[derive(Clone, Copy)]
pub enum SectorModel {
    Private,
    Public,
    Balancer,
}

/// The configuration struct used as current context by Fencer.
///
/// This struct is required to be passed on [`Fencer`] startup
pub struct Config {
    pub sector_limit: u32,
    pub balancer_limit: u32,
    pub fencer_id: String,
    pub default_profile: SectorModel,
    pub balancer_spawn: Option<u32>,
    pub balancer_remove: Option<u32>,
    pub balancer_capacity: Option<u32>,
}

/// A control list struct that stores all metadata related to its corresponding
/// sector.
///
/// It holds all necessary context about the struct, such as ID, Model and so on,
/// and all the access entries for authorized modules, and their permissions.
pub struct ACL {
    owner: [u8; 16],
    _sector_id: u32,
    _version: SectorModel,
    _entry_count: u32,
    entries: [Option<ACLEntry>; MAX_ACL_ENTRIES],
}

/// The ACL entry struct that holds the authorized module ID and its permission
/// inside the current sector.
///
/// Permissions are written and read as bitwise operations
#[derive(Clone, Copy, PartialEq)]
pub struct ACLEntry {
    module_id: [u8; 16],
    permissions: u8,
}

/// The fencer coordinator structure that holds all necessary metadata about the
/// current fencer deployment.
///
/// It is written at the beginning of the dataplane at spawn to be used simultane-
/// ously across independent threads and processes
pub struct Fencer {
    _buf_size: u32,
    sect_attached: AtomicU32,
    _bal_attached: AtomicU32,
    guard_range: (AtomicU32, AtomicU32),
    config: Config,
}

/// A guard structure that provides checked operations inside its corresponding
/// sector.
///
/// It comes with all the necessary objects to execute functions inside the current
/// sector with a permission check step that fails if the caller tries to execute
/// unauthorized operations.
pub struct RouteGuard<'a> {
    private_id: &'a str,
    acl: &'static ACL,
    acl_offset: u32,
    inbox: &'a AtomicRingBuffer,
    sector_handler: MatrixHandler,
}

pub struct Balancer<T: Fn() -> ()> {
    cap: u32,
    action: T,
    threshold_spawn: u32,
    threshold_decomission: u32,
    sectors: Vec<BalancerSectors>,
}

pub struct BalancerSectors {
    handler: MatrixHandler,
    capacity: u32,
    occupation: u32,
}

impl Fencer {
    /// Spawn a new [`Fencer`] object and returns both the Fencer reference, and
    /// the Handler related to the Dataplane created.
    ///
    /// It first tries to create a new Dataplane (a dedicated [`AtomicMatrix`] instance
    /// for metada control), if one already exists, it simply attaches to it. Then 
    /// it tries to write the fencer struct at the first head available in the matrix, 
    /// if one already exists, it stops the bootstraping and returns. This ensures that 
    /// any isolated process can be attached to the Fencer environment without bursting 
    /// the already existing displays.
    ///
    /// # Params:
    ///
    /// @cfg: The configuration to be applied in the Fencer.
    ///
    /// #Returns
    ///
    /// A tuple containing a reference to Fencer inside the Dataplane, and the Dataplane
    /// itself.
    pub fn new<'a>(cfg: Config) -> (&'a Self, MatrixHandler) {
        let id = String::from("dataplane-".to_owned() + &cfg.fencer_id);
        let fencer_size = std::mem::size_of::<Fencer>() as u32;
        let acl_size = std::mem::size_of::<ACL>() as u32;
        let metadata_session = std::mem::size_of::<AtomicMatrix>() as u32 + 16;
        let util_size = (HEADER_SPACE + fencer_size)
            + ((HEADER_SPACE + acl_size) * (cfg.sector_limit + cfg.balancer_limit));

        let handler =
            AtomicMatrix::bootstrap(Some(id), (metadata_session + util_size) as usize).unwrap();

        let first_h = unsafe { handler.matrix().query(metadata_session).offset() - HEADER_SPACE };
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
                        _bal_attached: AtomicU32::new(0),
                        _buf_size: HEADER_SPACE + acl_size,
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

    /// Creates a new independent sector wrapped in a [`RouteGuard`].
    ///
    /// Generates a completelly new, sectorized [`AtomicMatrix`] instance, generates
    /// a new RouteGuard with provided permissions, attach a new ACL in the dataplane
    /// and returns the RouteGuard object with the [`ACL`] and [`MatrixHandler`]
    /// stored in it. If spawning a new sector fails for any reason, the function
    /// will return a [`FencerErrors`] instead.
    ///
    /// # Params:
    ///
    /// @dataplane: The [`Fencer`] Dataplane reference. \
    /// @size: The size for the new sector. \
    /// @priv_id: The id of the caller module. \
    /// @perms: The initial permissions to be attached to the [`RouteGuard`]
    ///
    /// # Returns
    ///
    /// A result containing either the RouteGuard, or a `SectorSpawnError`
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
        let acl = self.generate_acl(id, dataplane.hash_id(priv_id), None, perms).unwrap();
        unsafe { block.pointer.write(dataplane.base_ptr(), acl) };
        let current_range_peak = self.guard_range.1.load(Ordering::Acquire);
        if id > current_range_peak {
            self.guard_range.1.store(id, Ordering::Release);
        }

        let handler = AtomicMatrix::bootstrap(Some(id.to_string()), size).unwrap();
        let rb = match AtomicRingBuffer::new::<u32>(1024, handler.share(), Behaviour::Drop) {
            Some(v) => v,
            None => {
                return Err(FencerErrors::SectorSpawnError(String::from(
                    "Failed to create inbox",
                )))
            }
        };

        let guard = RouteGuard {
            private_id: priv_id,
            acl: unsafe { dataplane.read::<ACL>(&block).unwrap() },
            acl_offset: block.pointer.offset(),
            sector_handler: handler,
            inbox: rb,
        };

        Ok(guard)
    }

    /// Create and write a new ACL at the tail of the Dataplane.
    ///
    /// It first checks if the quantity of provided ACLs surpasses the defined cap
    /// for this Fencer instance. If the request doesn't break the cap limit, it
    /// starts the ACL generation process. If it does, breaks the function returning
    /// a [`FencerErrors`].
    ///
    /// # Params
    ///
    /// @id: The id of the sector created. \
    /// @priv_id: The id of the caller module. \
    /// @prof: The profile that this sector will have, choosen from [`SectorModel`] \
    /// @perms: The ACLEntry list to attach at this ACL.
    ///
    /// # Returns
    ///
    /// A result containing either the ACL created, or a `SectorSpawnError`
    pub fn generate_acl<'a>(
        &self,
        id: u32,
        priv_id: [u8; 16],
        prof: Option<SectorModel>,
        perms: Vec<ACLEntry>,
    ) -> Result<ACL, FencerErrors> {
        if perms.len() > MAX_ACL_ENTRIES {
            return Err(FencerErrors::SectorSpawnError(format!(
                "Too many ACL entries (max {})",
                MAX_ACL_ENTRIES
            )));
        }

        let profile_name = prof.unwrap_or_else(|| self.config.default_profile);

        let mut entries: [Option<ACLEntry>; MAX_ACL_ENTRIES] = [None; MAX_ACL_ENTRIES];
        for (i, v) in perms.iter().enumerate() {
            entries[i] = Some(*v);
        }

        Ok(ACL {
            owner: priv_id,
            _sector_id: id,
            _version: profile_name,
            _entry_count: perms.len() as u32,
            entries,
        })
    }

    /// Get an existing sector from the Dataplane and attach to it.
    ///
    /// First it checks if the caller module has permissions inside this sector.
    /// If none is found or the caller doesn't have read permissions, the function
    /// breaks with a [`FencerErrors`]. If not, generates a new [`RouteGuard`],
    /// wraps the sector in it and returns to the caller.
    ///
    /// # Params
    ///
    /// @dataplane: The [`Fencer`] internal matrix instance. \
    /// @sec_id: The sector to be attached. \
    /// @priv_id: The id of the caller.
    ///
    /// # Returns
    ///
    /// A result containing either the RouteGuard of the sector, or a
    /// `SectorAttachingError`
    pub fn get_sector<'a>(
        &self,
        dataplane: &MatrixHandler,
        sec_id: u32,
        priv_id: &'a str,
    ) -> Result<RouteGuard<'a>, FencerErrors> {
        let block = unsafe { Block::<ACL>::from_offset(dataplane.matrix().query(sec_id).offset()) };
        let module_tag = dataplane.hash_id(priv_id);

        let acl_data = unsafe { dataplane.read(&block).unwrap() };

        let entry = acl_data
            .entries
            .iter()
            .filter_map(|e| e.as_ref())
            .find(|e| e.module_id == module_tag);
        let perms = match entry {
            Some(e) => e.permissions,
            None if acl_data.owner == module_tag => PERM_READ | PERM_WRITE | PERM_OWN,
            None => return Err(FencerErrors::SectorAttachingError),
        };

        if perms & PERM_READ == 0 {
            return Err(FencerErrors::SectorAttachingError);
        }

        let sect_handler = AtomicMatrix::bootstrap(Some(sec_id.to_string()), 0).unwrap();
        let m_iter = looper::Looper::new(sect_handler.share());

        let mut iter_obj = m_iter.filter(|lv| {
            let state = lv.view_header().state.load(Ordering::Relaxed);
            if state == STATE_RINGBUFFER {
                true
            } else {
                false
            }
        });

        let rb = match iter_obj.next() {
            Some(v) => v,
            None => return Err(FencerErrors::SectorAttachingError),
        };

        Ok(RouteGuard {
            private_id: priv_id,
            acl: unsafe { dataplane.read::<ACL>(&block).unwrap() },
            acl_offset: block.pointer.offset(),
            sector_handler: sect_handler,
            inbox: rb.view_data_as::<AtomicRingBuffer>(),
        })
    }

    pub fn new_balancer<T: Fn() -> ()>(&self, action: T) -> Result<Balancer<T>, FencerErrors> {
        let config = &self.config;

        if config.balancer_spawn.is_none() || config.balancer_remove.is_none() || config.balancer_capacity.is_none() {
            return Err(FencerErrors::SectorError(
                    String::from("Cannot create a balancer with empty balancer spawn and remove parameters")))
        } else {
            let balancer = Balancer {
                cap: config.balancer_limit.to_owned(),
                action,
                threshold_spawn: config.balancer_spawn.unwrap().to_owned(),
                threshold_decomission: config.balancer_remove.unwrap().to_owned(),
                sectors: Vec::<BalancerSectors>::new(),
            };

            Ok(balancer)
        }
    }
}

impl<'a> RouteGuard<'a> {
    /// Publish a new message inside the sector.
    ///
    /// First it checks if the caller has write permission in this sector, then it
    /// allocates the message and push it to the sector [`AtomicRingBuffer`]. If
    /// the module doesn't have permission to write, or enqueuing the message in the
    /// ring buffer fails, the function will return a [`FencerErrors`].
    ///
    /// # Params
    ///
    /// @msg<T>: The message to be routed into the sector, typed as a generic.
    ///
    /// # Returns
    ///
    /// A result containing either Ok(()), or a [`FencerErrors`]
    pub fn route_msg<T>(&self, msg: T) -> Result<(), FencerErrors> {
        if !self.check_permissions(PERM_WRITE) {
            return Err(FencerErrors::UnauthorizedWrite);
        };

        let mut block = match self.sector_handler.allocate::<T>() {
            BetterResult::Val(Some(v)) => v,
            BetterResult::Err(Some(e)) => return Err(FencerErrors::SectorError(format!("{:?}", e))),
            _ => todo!(),
        };

        self.sector_handler.write(&mut block, msg);

        match self.inbox.enqueue::<u32>(block.pointer.offset()) {
            Ok(_) => return Ok(()),
            Err(e) => return Err(FencerErrors::SectorError(format!("{:?}", e))),
        };
    }

    /// Reads messages from the sector, if it has any.
    ///
    /// First it checks if the caller has read permission in this sector. then it
    /// reads the next head in the ring buffer and pops the block from the sector
    /// instance. If the caller has no permission to read, it returns a [`FencerErrors`].
    ///
    /// Since the ring buffer returns None if there are no new messages but the read
    /// was successful, the message return is an [`Option<T>`] containing either the
    /// message or [`None`] if it was empty.
    ///
    /// # Returns
    ///
    /// A result containing either Option<msg>, or a [`FencerErrors`]
    pub fn load_msg<T>(&self) -> Result<Option<&T>, FencerErrors> {
        if !self.check_permissions(PERM_READ) {
            return Err(FencerErrors::UnauthorizedRead);
        };

        let msg_offset = match self.inbox.dequeue::<u32>() {
            Some(v) => v,
            None => return Ok(None),
        };

        let msg_block = Block::<T>::from_offset(*msg_offset);
        let msg = unsafe { self.sector_handler.read(&msg_block).unwrap() };

        return Ok(Some(msg));
    }

    /// Clears the entire sector, leaving only the ring buffer available.
    ///
    /// First it checks if the caller has write permissions in this sector. Then it
    /// iterates over every block present in the matrix that is not a ring buffer
    /// freeing them.
    ///
    /// # Returns
    ///
    /// A result containing either Ok(()), or a [`FencerErrors`]
    pub fn clear_sector(&self) -> Result<(), FencerErrors> {
        if !self.check_permissions(PERM_WRITE) {
            return Err(FencerErrors::UnauthorizedWrite);
        }

        let m_iter = looper::Looper::new(self.sector_handler.share());

        for view in m_iter {
            if view.view_header().state.load(Ordering::Relaxed) != STATE_RINGBUFFER {
                self.sector_handler
                    .free_at(view.view_offset() - HEADER_SPACE);
            } else {
                continue;
            };
        }

        Ok(())
    }

    /// Deletes de current sector from the system.
    ///
    /// This completelly removes the sector ACL from the Dataplane and deletes the
    /// SHM file used by it. As this function is final, only the owner of this sector
    /// (the one who created it) has the permission to execute a decomission.
    ///
    /// # Safety
    ///
    /// This function completelly wipes the sector from the system, but doesn't removes
    /// any other RouteGuard acquired by other modules that still points to this sector.
    /// Decomissioning a sector without keeping this behaviour in mind may cause UB.
    ///
    /// # Params
    ///
    /// @dataplane: A reference to the Fencer's [`MatrixHandler`]
    ///
    /// # Returns
    ///
    /// A result containing either Ok(()), or a [`FencerErrors`].
    pub fn decommission_sector(&self, dataplane: &MatrixHandler) -> Result<(), FencerErrors> {
        if !self.check_permissions(PERM_OWN) {
            return Err(FencerErrors::SectorError(String::from(
                "Sector cannot be decomission by someone other than Owner.",
            )));
        }
        
        dataplane.free_at(self.acl_offset);
        match self.sector_handler.die() {
            BetterResult::Val(Some(_)) => Ok(()),
            BetterResult::Err(Some(e)) => Err(FencerErrors::SectorError(format!("{:?}", e))),
            _ => todo!()
        }
    }

    /// Returns the [`ACLEntry`] corresponding to the caller's provided id.
    ///
    /// If no ACLEntry is found for the id, the function breaks with an error.
    ///
    /// # Returns
    ///
    /// A result containing either the [`ACLEntry`], or a u8 indicating the error
    ///
    // #TODO: implement a real defined error for this function.
    pub fn get_permissions(&self, id: [u8; 16]) -> Result<ACLEntry, u8> {
        for entry in self.acl.entries {
            match entry {
                Some(v) => {
                    if v.module_id == id {
                        return Ok(v);
                    }
                }
                None => continue,
            }
        }

        return Err(1);
    }

    /// Checks if the caller has the requested permission inside the current 
    /// [`RouteGuard`].
    ///
    /// It returns a bool indicating if the user has or not the permission required
    /// by the perm_check.
    ///
    /// # Params
    ///
    /// @perm_check: The permission to check against the caller.
    ///
    /// # Returns
    /// 
    /// A bool indicating if the user has or not the permission.
    fn check_permissions(&self, perm_check: u8) -> bool {
        let hashed_id = self.sector_handler.hash_id(self.private_id);
        match self.get_permissions(hashed_id) {
            Ok(v) => {
                if v.permissions & perm_check == 0 && v.permissions & PERM_OWN == 0 {
                    return false;
                }
            }
            Err(_) => return false,
        };

        true
    }
}

impl<T: Fn() -> ()> Balancer<T> {
    pub fn balance<F>(&self, msg: F) {
        unimplemented!()
    }

    fn spawn_balancer() {
        unimplemented!()
    }

    fn decommission_balancer() {
        unimplemented!()
    }
}
