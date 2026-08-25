pub mod internals;
pub mod core;
pub mod extensive_lib;
pub mod helpers;

/// Default imports for atomic_matrix. Included:
///
/// - core::handlers::*,
/// - core::matrix::*,
/// - core::matrix::helpers::*,
/// - core::cartographer::*,
/// - core::cartographer_builder::*,
/// - internals::collections::*
/// - helpers::atomic_ext::AtomicExtensions,
///
/// All errors, macros and extensive_lib modules should be explicitly imported if needed.
pub mod prelude {
    pub use crate::core::{
        handlers::*,
        matrix::{
            *,
            helpers::*,
        },
        cartographer::*,
        cartographer_builder::*,
    };
    pub use crate::internals::collections::*;
    pub use crate::helpers::{
        atomic_ext::AtomicExtensions,
        type_tag,
        type_guard,
        safe_shm::SafeSHM,
    };
}
