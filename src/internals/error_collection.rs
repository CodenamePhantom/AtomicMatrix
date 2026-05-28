/// -------------------------------------------------------------------------------------
///
/// Extensive Lib errors
///
/// -------------------------------------------------------------------------------------
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

/// -------------------------------------------------------------------------------------
///
/// Core errors
///
/// -------------------------------------------------------------------------------------
#[derive(Debug)]
pub enum MatrixErrors {
    MatrixInitializationError(String),
    MatrixAttachingError,
    OutOfMemory,
    EmptyBitmapError,
}

#[derive(Debug)]
pub enum HandlerErrors {
    AllocationFailed(String),
    ReservedState(u32),
    TransitionFailed(u32),
    InvalidOffset(u32),
    DecomissionFailed {
        path: String,
        reason: std::io::Error,
    },
}

/// -------------------------------------------------------------------------------------
///
/// Collection errors
///
/// -------------------------------------------------------------------------------------
#[derive(Debug)]
pub enum BufferErrors {
    TooManyProducers,
    DropBehaviour,
    BufferFull,
}

#[derive(Debug)]
pub enum AtomicArrayErrors {
    EmptyIndexError,
    NotAnArrayError,
    BlockedSlotError(String),
    AtomicWriteFailed,
    SetOpError(String),
}
