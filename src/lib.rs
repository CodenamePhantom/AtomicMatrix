pub mod internals;
pub mod core;
pub mod extensive_lib;
pub mod helpers;

pub mod prelude {
    pub use crate::core::{
        handlers::*,
        matrix::{
            *,
            helpers::*,
        }
    };
    pub use crate::internals::collections::*;
    pub use crate::helpers::*;
}
