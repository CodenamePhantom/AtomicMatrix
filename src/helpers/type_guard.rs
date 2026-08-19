use crate::prelude::*;
use std::sync::atomic::Ordering;
use std::panic::UnwindSafe;

/// Runtime safe type wrapper for deref a matrix value reference.
/// 
/// It holds no direct reference to the value other than its type, opting instead, for calculating
/// and validating its existance every time it is dereferenced. This approach trades runtime UB 
/// events for deterministic panics and errors that can be treated locally.
#[derive(Debug)]
pub struct TypeGuard<T> {
    val_offset: u32,
    offset: u32,
    base_ptr: *const u8,
    _marker: std::marker::PhantomData<*const T>
}

/// Runtime safe type wrapper for deref a matrix mutable value reference.
/// 
/// It holds no direct reference to the value other than its type, opting instead, for calculating
/// and validating its existance every time it is dereferenced. This approach trades runtime UB 
/// events for deterministic panics and errors that can be treated locally.
#[derive(Debug)]
pub struct TypeGuardMut<T> {
    val_offset: u32,
    offset: u32,
    base_ptr: *const u8,
    _marker: std::marker::PhantomData<*const T>
}

impl<T> TypeGuard<T> {
    /// Recalculates the data header location and validate that it is in fact, a header.
    ///
    /// ### Returns:
    /// An optional reference to the BlockHeader, if it passes all the checks.
    fn get_header(&self) -> Option<&BlockHeader> {
        let header_ptr = unsafe { self.base_ptr.add(self.offset as usize) as *const BlockHeader };
        let header = unsafe { header_ptr.as_ref()? };

        if header.signature != HEADER_SIGNATURE { return None };
        if header.state.load_if_any(&[STATE_ACKED, STATE_FREE, STATE_COALESCING], Ordering::Acquire).is_ok() {
            return None
        }

        Some(header)
    }

    /// Creates a new TypeGuard struct from the raw data coordinates.
    ///
    /// ### Params:
    /// @val_offset: The offset to the immediate start of the data bytes. \
    /// @offset: The offset to the absolute start of the block segment. \
    /// @base_ptr: The pointer to the base of the SHM segment.
    ///
    /// ### Returns:
    /// An instance of Self.
    pub(crate) fn new(val_offset: u32, offset: u32, base_ptr: *const u8) -> Self {
         Self {
            val_offset,
            offset,
            base_ptr,
            _marker: std::marker::PhantomData
         }
    }

    /// Validates the header of the current data coordinates and returns a reference to the data
    /// inside the matrix.
    ///
    /// ### Returns:
    /// An optional reference to the data, if the block header is valid.
    pub fn try_get(&self) -> Option<&T> {
        self.get_header()?;

        unsafe {
            let val_ptr = self.base_ptr.add(self.val_offset as usize) as *const T;
            val_ptr.as_ref()
        }
    }
}

/// Derefing this struct will run `try_get()` in the coordinates to fetch the reference. Panic if
/// fails.
impl<T> std::ops::Deref for TypeGuard<T> {
    type Target = T;

    fn deref(&self) -> &T {
        self.try_get()
            .expect("TypeGuard: access to invalidated, freed, or corrupted block")
    }
}

impl<T> TypeGuardMut<T> {
    /// Recalculates the data header location and validate that it is in fact, a header.
    ///
    /// ### Returns:
    /// An optional reference to the BlockHeader, if it passes all the checks.
    fn get_header(&self) -> Option<&BlockHeader> {
        let header_ptr = unsafe { self.base_ptr.add(self.offset as usize) as *const BlockHeader };
        let header = unsafe { header_ptr.as_ref()? };

        if header.signature != HEADER_SIGNATURE { return None };
        if header.state.load_if_any(&[STATE_ACKED, STATE_FREE, STATE_COALESCING], Ordering::Acquire).is_ok() {
            return None
        }

        Some(header)
    }

    /// Creates a new TypeGuard struct from the raw data coordinates.
    ///
    /// ### Params:
    /// @val_offset: The offset to the immediate start of the data bytes. \
    /// @offset: The offset to the absolute start of the block segment. \
    /// @base_ptr: The pointer to the base of the SHM segment.
    ///
    /// ### Returns:
    /// An instance of Self.
    pub(crate) fn new(val_offset: u32, offset: u32, base_ptr: *const u8) -> Self {
         Self {
            val_offset,
            offset,
            base_ptr,
            _marker: std::marker::PhantomData
         }
    }

    /// Validates the header of the current data coordinates and returns a reference to the data
    /// inside the matrix.
    ///
    /// ### Returns:
    /// An optional mutable reference to the data, if the block header is valid.
    pub fn try_get(&self) -> Option<&T> {
        self.get_header()?;

        unsafe {
            let val_ptr = self.base_ptr.add(self.val_offset as usize) as *const T;
            val_ptr.as_ref()
        }
    }

    /// Validates the header of the current data coordinates and returns a mutable reference to the 
    /// data inside the matrix.
    ///
    /// ### Returns:
    /// An optional mutable reference to the data, if the block header is valid.
    pub fn try_get_mut(&mut self) -> Option<&mut T> {
        self.get_header()?;

        unsafe {
            let val_ptr = self.base_ptr.add(self.val_offset as usize) as *mut T;
            val_ptr.as_mut()
        }
    }
}

/// Derefing this struct will run `try_get()` in the coordinates to fetch the mutable reference. Panic if
/// fails.
impl<T> std::ops::Deref for TypeGuardMut<T> {
    type Target = T;

    fn deref(&self) -> &T {
        self.try_get()
            .expect("TypeGuard: access to invalidated, freed, or corrupted block")
    }
}

/// Derefing this struct will run `try_get()` in the coordinates to fetch the mutable reference. 
/// Panic if fails.
impl<T> std::ops::DerefMut for TypeGuardMut<T> {
    fn deref_mut(&mut self) -> &mut T {
        self.try_get_mut()
                .expect("TypeGuard: access to invalidated, freed, or corrupted block")
    }
}

/// UnwindSafe is implemented to ensure that panics can be converted to errors.
impl<T> UnwindSafe for TypeGuard<T> {}
impl<T> UnwindSafe for TypeGuardMut<T> {}
