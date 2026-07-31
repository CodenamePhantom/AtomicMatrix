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
