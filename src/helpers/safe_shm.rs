use std::cell::{ Cell, UnsafeCell };
use std::marker::PhantomData;
use std::sync::atomic::*;

/// A marker struct that tells the compiler the implemented type is AtomicMatrix safe.
///
/// Types that implement this trait are not heap allocated and does not wrap any pointer indirection,
/// as allocating a value to the matrix casts the value directly into bytes, and doesn't grab any value
/// from indirect pointer Structs like Strings, Vec, Boxes and others.
///
/// Most base primitives that come with rust have this trait implemented by default on the crate.
///
/// ### DISCLAIMER
/// Callers can implement this trait to whatever types they want, even heap allocated ones. But this
/// behaviour is not recommended and it **WILL** cause undefined behaviour.
pub unsafe trait SafeSHM: 'static {}
pub unsafe trait SafeSHMUnsized: SafeSHM {
    unsafe fn from_raw_parts(ptr: *const u8, byte_len: usize) -> *const Self;
}

/// Sized types
unsafe impl SafeSHM for u8 {}
unsafe impl SafeSHM for u16 {}
unsafe impl SafeSHM for u32 {}
unsafe impl SafeSHM for u64 {}
unsafe impl SafeSHM for u128 {}
unsafe impl SafeSHM for usize {}
unsafe impl SafeSHM for i8 {}
unsafe impl SafeSHM for i16 {}
unsafe impl SafeSHM for i32 {}
unsafe impl SafeSHM for i64 {}
unsafe impl SafeSHM for i128 {}
unsafe impl SafeSHM for isize {}
unsafe impl SafeSHM for f32 {}
unsafe impl SafeSHM for f64 {}
unsafe impl SafeSHM for bool {}
unsafe impl SafeSHM for char {}
unsafe impl SafeSHM for () {}
unsafe impl SafeSHM for AtomicBool {}
unsafe impl SafeSHM for AtomicU8 {}
unsafe impl SafeSHM for AtomicU16 {}
unsafe impl SafeSHM for AtomicU32 {}
unsafe impl SafeSHM for AtomicU64 {}
unsafe impl SafeSHM for AtomicUsize {}
unsafe impl SafeSHM for AtomicI8 {}
unsafe impl SafeSHM for AtomicI16 {}
unsafe impl SafeSHM for AtomicI32 {}
unsafe impl SafeSHM for AtomicI64 {}
unsafe impl SafeSHM for AtomicIsize {}
unsafe impl<T: SafeSHM> SafeSHM for AtomicPtr<T> {}
unsafe impl<T: SafeSHM> SafeSHM for UnsafeCell<T> {}
unsafe impl<T: SafeSHM> SafeSHM for Cell<T> {}
unsafe impl<T: SafeSHM> SafeSHM for Option<T> {}
unsafe impl<T: SafeSHM> SafeSHM for PhantomData<T> {}
unsafe impl<T: SafeSHM, const N: usize> SafeSHM for [T; N] {}

/// Unsized types
unsafe impl SafeSHM for str {}
unsafe impl SafeSHMUnsized for str {
    unsafe fn from_raw_parts(ptr: *const u8, byte_len: usize) -> *const Self {
        std::ptr::slice_from_raw_parts(ptr, byte_len) as *const str
    }
}
unsafe impl<T: SafeSHM> SafeSHM for [T] {}
unsafe impl<T: SafeSHM> SafeSHMUnsized for [T] {
    unsafe fn from_raw_parts(ptr: *const u8, byte_len: usize) -> *const Self {
        std::ptr::slice_from_raw_parts(ptr as *const T, byte_len / std::mem::size_of::<T>())    
    }
}
