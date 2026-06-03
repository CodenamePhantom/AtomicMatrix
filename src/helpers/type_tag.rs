//! # Type Tagging generation for cross-process type referencing.
//!
//! All types passed through this helper can be safely coordinated between threads, as passing the
//! same type from a different caller process will result in the same hash.

use crate::prelude::{ Block, TAG_SIZE };

use std::any::type_name;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

/// Generate a new type_tag in u32 format from the type passed to this function.
///
/// The type is extracted using [`type_name`] then run through rust internal hash_map module 
/// [`DefaultHasher`]. The value is then returned as an u32.
///
/// ### Params:
/// @<T>: The type to be hashed, passed as a typed call.
///
/// ### Returns:
/// The hashed type name in u32 format.
pub fn make<T>() -> u32 {
    let name = type_name::<T>();
    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish() as u32
}

/// Compares the passed type to the type tag inside the provided block.
///
/// The type is extracted from both the passed in type through [`make`], and from the block. Both
/// value are then compared against each other and the bool resulting from the check is returned.
///
/// ### Params:
/// @<T>: The type to compare, passed as a typed call.
/// @block: The block to check the type against.
///
/// ### Returns:
/// A bool stating if the types matches or not.
pub fn compare<T>(block: &Block<T>) -> bool {
    let base = block.pointer.offset() - TAG_SIZE;

    let stored_tag = unsafe { (base as *const u32).read() };
    let expected_tag = make::<T>();

    stored_tag == expected_tag
}
