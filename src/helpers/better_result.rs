//! Error handling framework for [`AtomicMatrix`] errors.
//!
//! It works just like [`Result`], but give you monad expression powers on top of error handling,
//! allowing for chained calls for error life cycle stages.
//!
//! - Fail or succeed calls returning either the error, or the value.
//! - then, catch and finally error handling methods.
//! - Opt-out into pattern matching directly from the error struct.
//!
//! ### Usage
//!
//! ```rust
//! failable_call
//!     .then(|v| {//...do some stuff with the value}) 
//!     .catch(|e| return BetterResult::fail(e))
//!     .finally(|v| v); // returns the value to the variable
//!
//! // Additionally, you can take the value directly:
//!
//! let some_value: any;
//! failable_call
//!     .then_capture(|v| some_value = v)
//!     .catch(|e| return BetterResult::fail(e))
//!
//! // Or return it directly:
//!
//! failable_call
//!     .catch(|e| return BetterResult::fail(e))
//!     .finally_return(|v| return BetterResult::succeed(v));
//!
//! // Direct pattern matching:
//!
//! match failable_call {
//!     BetterResult::Val(v) => {//...do some stuff with the value},
//!     BetterResult::Err(e) => {//...directly handles the error},
//! }
//!
//! // Errors can optionally be chained up without the nightly `Try` trait:
//!
//! failable_call
//!     .chain_up::<SomeErrorType>()
//! ```

/// BetterResult error handling enum.
///
/// It optionally holds both the error and the value to be returned. These can, then, be manipulated
/// through monadic chains.
#[derive(Debug)]
pub enum BetterResult<T, K> {
    Val(Option<T>),
    Err(Option<K>),
}

impl<T, K> BetterResult<T, K>
where
    K: std::fmt::Display,
    K: std::error::Error,
{
    /// Fails the call and returns the error value wrapped in [`BetterResult::Err`]
    ///
    /// ### Params:
    /// @err: The error value to be wrapped.
    ///
    /// ### Returns:
    /// A BetterResult::Err instance
    pub fn fail(err: K) -> Self {
        BetterResult::Err(Some(err))
    }

    /// Completes the call and returns the value wrapped in [`BetterResult::Val`]
    ///
    /// ### Params:
    /// @val: The value to be wrapped.
    ///
    /// ### Returns:
    /// A BetterResult::Val instance
    pub fn succeed(val: T) -> Self {
        BetterResult::Val(Some(val))
    }

    /// Chains the result operation to a function with a reference to the value as argument. If the 
    /// variant received is [`BetterResult::Err`] instead, pass the function forward to the `catch()` 
    /// phase.
    ///
    /// ### Params:
    /// @f: The closure to manipulate the result value.
    ///
    /// ### Returns:
    /// An instance of Self
    pub fn then<F>(mut self, f: F) -> Self 
    where
        F: FnOnce(&mut T),
    {
        if let BetterResult::Val(Some(v)) = &mut self {
            f(v);
        }

        return self
    }

    /// Chains the result operation to a function that takes the value from the variant. If the
    /// variant received is [`BetterResult::Err`] instead, pass the function forward to the `catch()` 
    /// phase.
    ///
    /// The result value captured within this function should be consumed, or it will be dropped at
    /// the end of the closure.
    ///
    /// ### Params:
    /// @f: The closure to manipulate the result value.
    ///
    /// ### Returns:
    /// An instance of Self
    pub fn then_capture<F>(mut self, f: F) -> Self
    where
        F: FnOnce(T),
    {
        if let BetterResult::Val(v) = &mut self {
            f(v.take().unwrap());
        }

        return self
    }



    /// Catches the error variant and executes a breakpoint function with the error value that
    /// returns [`BetterResult::Err`]. If the variant is [`BetterResult::Val`] instead, pass the
    /// function to the next consumer in the chain.
    ///
    /// ### Params:
    /// @f: The closure to manipulate and return the error value.
    ///
    /// ### Returns:
    /// An instance of self, in case the value is not an error. Otherwise, the error is returned.
    pub fn catch<F>(self, f: F) -> Self 
    where
        F: FnOnce(K) -> Self,
    {
        match self {
            BetterResult::Err(Some(e)) => f(e),
            val => val,
        }
    }

    /// Executes an inline action with the error value if one is catched.
    ///
    /// This function will execute a closure on an error value without retuning anything. It can be
    /// used to treat error events in-call instead of returning the error.
    ///
    /// ### Params:
    /// @f: The closure to manipulate the error value.
    pub fn catch_contained<F>(self, f: F) 
    where
        F: FnOnce(K)
    {
        match self {
            BetterResult::Err(Some(e)) => f(e),
            _ => {},
        }
    }

    /// Catch an error and returns a different error value in its place.
    ///
    /// ### Params:
    /// @f: The closure to manipulate the error value.
    /// @<NewK>: The type which will be returned by the closure, passed as a typed argument.
    ///
    /// ### Returns:
    /// A new instance of self with the new error typeL.
    pub fn catch_cast<F, NewK>(self, f: F) -> BetterResult<T, NewK>
    where
        F: FnOnce(K) -> BetterResult<T, NewK>,
        NewK: std::fmt::Display + std::error::Error,
    {
        match self {
            BetterResult::Err(Some(e)) => f(e),
            BetterResult::Val(Some(v)) => BetterResult::Val(Some(v)),
            _ => unreachable!(),
        }
    }

    /// Takes the result value into a closure that stores it to a variable.
    ///
    /// Finally functions close the monad chain, so they must be called after catch functions. This
    /// also guarantees that any finally calls had errors checked before calling. This behaviour is 
    /// enforced by the borrow checker on compile time.
    ///
    /// ### Params:
    /// @f: The closure that consumes the result value to return it.
    ///
    /// ### Returns:
    /// The result value.
    pub fn finally<F>(mut self, f: F) -> T
    where
        F: FnOnce(T) -> T,
    {
        if let BetterResult::Val(v) = &mut self {
            f(v.take().unwrap())
        } else {
            unreachable!()
        }
    }

    pub fn finally_as<F, NewT>(mut self, f: F) -> NewT
    where
        F: FnOnce(T) -> NewT,
    {
        if let BetterResult::Val(v) = &mut self {
            f(v.take().unwrap())
        } else {
            unreachable!()
        }
    }

    /// Takes the result value into a closure that completes the call, returning the value. 
    ///
    /// Finally functions close the monad chain, so they must be called after catch functions. This
    /// also guarantees that any finally calls had errors checked before calling. This behaviour is 
    /// enforced by the borrow checker on compile time.
    ///
    /// ### Params:
    /// @f: The closure that consumes the result value to return it.
    ///
    /// ### Returns:
    /// The result value.
    pub fn finally_return<F>(self, f: F) -> Self
    where
        F: FnOnce(T) -> Self,
    {
        match self {
            BetterResult::Val(Some(v)) => f(v),
            _ => unreachable!(),
        }
    }

    /// Takes the result value into a closure that completes the call. The value is then casted into
    /// the new type and returned from the closure.
    ///
    /// Finally functions close the monad chain, so they must be called after catch functions. This
    /// also guarantees that any finally calls had errors checked before calling. This behaviour is 
    /// enforced by the borrow checker on compile time.
    ///
    /// ### Params:
    /// @f: The closure that consumes the result value to return it. \
    /// @<NewT>: The type which the closure will return, passed as a typed argument.
    ///
    /// ### Returns:
    /// The result value.
    pub fn finally_return_as<F, NewT>(self, f: F) -> BetterResult<NewT, K>
    where
        F: FnOnce(T) -> BetterResult<NewT, K>,
    {
        match self {
            BetterResult::Val(Some(v)) => f(v),
            _ => unreachable!(),
        }
    }

    /// Casts the function into the specified error type and chains it up the BetterResult stream.
    ///
    /// This works as the ? keyword from the `Try` trait. Unfortunatelly, the rust std solution for
    /// this is unstable and still on nightly builds, so we implemented our own version of error
    /// chaining.
    ///
    /// ### Params:
    /// @<NewK>: The new error type to be chained into, passed as a typed argument.
    ///
    /// ### Returns:
    /// An instance of self with a casted error type.
    pub fn chain_up<NewK>(self) -> BetterResult<T, NewK>
    where
        NewK: From<K> + std::fmt::Display + std::error::Error,
    {
        match self {
            BetterResult::Err(Some(e)) => BetterResult::fail(NewK::from(e)),
            BetterResult::Val(Some(v)) => BetterResult::Val(Some(v)),
            _ => unreachable!()
        }
    }

    /// Checks if the call returns a value instead of an error.
    ///
    /// ### Returns:
    /// A bool stating if the result is a value instead of an error.
    pub fn is_ok(&self) -> bool {
        matches!(self, BetterResult::Val(_))
    }

    /// Checks if the call returns an error instead of a value.
    ///
    /// ### Returns:
    /// A bool stating if the result is an error instead of a value.
    pub fn is_err(&self) -> bool {
        matches!(self, BetterResult::Err(_))
    }

    /// Tries to extract the value from the result. If an error is present instead of a value, the
    /// function will `panic!()` the thread.
    ///
    /// ### Returns:
    /// The extracted value from the result, provided the thread is still running after the call.
    pub fn unwrap(self) -> T {
        match self {
            BetterResult::Err(Some(e)) => panic!("{}", e),
            BetterResult::Val(Some(v)) => v,
            _ => unreachable!()
        }
    }
}
