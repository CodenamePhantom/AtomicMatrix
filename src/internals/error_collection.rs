use better_result::better_error;

// -------------------------------------------------------------------------------------
//
// Extensive Lib errors
//
// -------------------------------------------------------------------------------------
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
better_error! {
    #[derive(Debug)]
    pub enum MatrixErrors {
        MatrixInitializationError { reason: String } => "Failed to initialized matrix: {reason}",
        MatrixAttachingError => "Failed to attach to the specified AtomicMatrix",
        OutOfMemory => "The SHM segment ran out of memory",
        EmptyBitmapError => "The TLSF bitmap has no available segments to allocate this request",
        OutOfBounds => "The requested offset lies out of the AtomicMatrix bounds",
        InvalidBlock => "The block requested is in an invalid state, or doesn't exist",
        MisalignedHeader => "The offset provided is not aligned within header constraints",
    }
}

better_error! {
    #[derive(Debug)]
    pub enum HandlerErrors {
        TypeMismatchError => "The provided type doesn't match the block type tag.",
        PanicRecovery => "Invalid deref yielded a panic.",
        AllocationFailed { reason: String } => "Failed to allocate: {reason}",
        ReservedState { state: u32 } => "State {state} is reserved for matrix ops.",
        InvalidOffset { offset: u32 } => "Offset {offset} not found.",
        BlockOverflow {
            block_size: usize,
            payload_size: usize
        } => "The requested size doesn't fit the block: Req Size -> {payload_size} | Available size -> {block_size}",
        DecomissionFailed {
            path: String,
            reason: std::io::Error
        } => "Failed to decommission matrix file: Path -> {path} | Reason -> {reason}",
        TransitionFailed { 
            requested_state: u32,
            current_state: u32
        } => "Failed to switch state: {requested_state} -> {current_state}",
    }
    from {
        InnerMatrixError(MatrixErrors),
    }
}

better_error! {
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

// -------------------------------------------------------------------------------------
//
// Collection errors
//
// -------------------------------------------------------------------------------------
better_error! {
    #[derive(Debug)]
    pub enum BufferErrors {
        TooManyProducers => "Too many threads hanging on the buffer",
        DropBehaviour => "Drop behaviour",
        BufferFull => "The buffer is completely full",
    }
}

#[derive(Debug)]
pub enum AtomicArrayErrors {
    EmptyIndexError,
    NotAnArrayError,
    BlockedSlotError(String),
    AtomicWriteFailed,
    SetOpError(String),
}
