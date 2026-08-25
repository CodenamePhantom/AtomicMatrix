use std::cell::{Cell, UnsafeCell};
use std::marker::PhantomData;
use std::sync::atomic::*;

pub unsafe trait SafeSHM: 'static {}

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