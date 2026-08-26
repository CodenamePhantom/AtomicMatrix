use crate::helpers::safe_shm::SafeSHM;

/// Executes a single deref operation over a TypeGuard, casting the panic to a Result if it occurs.
///
/// ### Disclaimer
/// This macro ensures the casting of a single access to Result, if more than one operation is
/// chained inside this macro (e.g.: { *a += 1; *b *= 2 }), and a panic occurs from any of them, all
/// the subsequent logic will die with the unwind, returning the error only for the casted
/// operation.
#[macro_export]
macro_rules! unwind {
    ($expr:expr) => {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| $expr))
            .map_err(|_| $crate::internals::error_collection::HandlerErrors::PanicRecovery)
    }
}

/// Implements SafeSHM for foreing structs.
/// 
/// ### Disclaimer
/// SafeSHM is a marker trait that tells the compiler the current type is safe to be used inside the 
/// AtomicMatrix. It can be implemented on heap allocated values like Strings, and Vecs, but these types
/// are not SHM safe and should not be used. 
#[macro_export]
macro_rules! safe_shm {
    ($struct_name:ident < $($g:ident),* >) => {
        unsafe impl<$($g: SafeSHM),*> SafeSHM for $struct_name<$($g),*> {}
    };
    ($struct_name:ident) => {
        unsafe impl SafeSHM for $struct_name {}
    };
}

macro_rules! safe_shm_tuple {
    () => {};
    ($first:ident $(, $rest:ident)* $(,)?) => {
        unsafe impl<$first: SafeSHM, $($rest: SafeSHM),*> SafeSHM for ($first, $($rest),*) {}
        safe_shm_tuple!($($rest),*);
    };
}

safe_shm_tuple!(A, B, C, D, E, F, G, H, I, J, K, L);