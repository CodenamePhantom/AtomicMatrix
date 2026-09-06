
/// Rollback is a RAII trigger that accepts a rollback function to be executed on dropping this
/// Struct.
///
/// Methods implement this to perform reparatory actions if anything goes south in the execution of
/// a inner call.
///
/// ### Panic!
///
/// If the rollback function also requires calls that would fail, they invariantly have to panic!
/// the caller if something goes wrong, as the closure does not accept any kind of returns from it.
pub struct Rollback<F: FnOnce()>(Option<F>);

impl<F: FnOnce()> Rollback<F> {
    /// Sets the rollback function to be executed on dropping the Rollback while its still armed.
    ///
    /// ### Params
    /// @f: The rollback function for the RAII guard.
    ///
    /// ### Returns
    /// An instance of self that will be dropped at the end of the scope.
    pub fn set(func: F) -> Self {
        return Self(Some(func))
    }

    pub fn update(&mut self, func: F) {
        self.0 = Some(func);
    }

    /// Disarms the rollback in the RAII guard before dropping the scope.
    ///
    /// This ideally should be called after all breakable calls that are recoverable are completed
    /// to avoid triggering a rollback in a successful execution.
    pub fn disarm(&mut self) {
        self.0 = None
    }
}

/// Drop implementation for Defer. Calls the rollback on being dropped.
impl<F: FnOnce()> Drop for Rollback<F> {
    fn drop(&mut self) {
        if let Some(f) = self.0.take() {
            f();
        }
    }
}