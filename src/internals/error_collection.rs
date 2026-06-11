use crate::matrix_error;
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

// -------------------------------------------------------------------------------------
//
// Core errors
//
// -------------------------------------------------------------------------------------
matrix_error! {
    #[derive(Debug)]
    pub enum MatrixErrors {
        MatrixInitializationError { reason: String } => "Failed to initialized matrix: {reason}",
        MatrixAttachingError => "",
        OutOfMemory => "",
        EmptyBitmapError => "",
        OutOfBounds => "",
        InvalidBlock => "",
    }
}

matrix_error! {
    #[derive(Debug)]
    pub enum HandlerErrors {
        TypeMismatchError => "The provided type doesn't match the block type tag.",
        AllocationFailed { reason: String } => "Failed to allocate: {reason}",
        ReservedState { state: u32 } => "State {state} is reserved for matrix ops.",
        InvalidOffset { offset: u32 } => "Offset {offset} not found.",
        DecomissionFailed {
            path: String,
            reason: std::io::Error
        } => "Failed to decommission matrix file: Path -> {path} | Reason -> {reason}",
        TransitionFailed { 
            old_state: u32,
            new_state: u32
        } => "Failed to switch state: {old_state} -> {new_state}",
    }
    from {
        InnerMatrixError(MatrixErrors),
    }
}

matrix_error! {
    #[derive(Debug)]
    pub enum CartographerErrors {
        CallbackError { reason: String } => "Failed to execute callback procedures: {reason}",
        InvalidCrossProcessCall { reason: String } => "The requested action is not a valid cross process call: {reason}",
        InvalidFileTypeCall { file_type: String } => "The requested action is not available for {file_type} constructions",
        MmapError { err_no: i32, call: String } => "Mmap operation failed. Error n: {err_no}, caller: {call}",
        FileCreationError { err_no: i32 } => "File create/attach syscall failed: {err_no}",
        FileTruncateError { err_no: i32 } => "File size truncate failed: {err_no}",
        RuntimeProtError { reason: String } => "Runtime protection engine failed: {reason}",
        WhyWouldYouDoThat => "Really... why?",
        SysInitializedError => "The system is, or was already initialized",
    }
    from {
        Io(std::io::Error),
        Inner(MatrixErrors),
    }
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
