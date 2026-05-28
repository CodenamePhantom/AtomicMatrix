//! Atomic operation extensions for atomic primitives.
//!
//! It adds a buch of pre-baked operations to facilitate the manipulation and alteration of atomic
//! primitives such as comparing against a list of numbers to execute the operation, executing only
//! if the current value is not X, and so on.

use std::sync::atomic;
use std::sync::atomic::Ordering;

pub trait AtomicExtensions {
    type Value: Copy + PartialEq;

    /// Swaps the value if its not equal the provided forbidden value.
    ///
    /// If the current value doesn't match, it swaps it and returns an Ok result with the value. If
    /// not, the value is returned wrapped in an Err().
    ///
    /// ### Params:
    /// @forbidden: The value to compare against the current value. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful swap. \
    /// @fail_ordering: Atomic ordering for a failed swap.
    ///
    /// ### Returns:
    /// The old value wraped in a result.
    #[must_use = "If swap result is not used, consider store_if_not instead."]
    fn swap_if_not(
        &self, 
        forbidden: Self::Value, 
        new_value: Self::Value, 
        success_ordering: Ordering,
        fail_ordering: Ordering
    ) -> Result<Self::Value, Self::Value>;

    /// Swaps the value if its equal to the provided target value.
    ///
    /// If the current value matches, it swaps and return an Ok result with the value. If not, the
    /// value is returned wrapped in an Err().
    ///
    /// ### Params:
    /// @target: The value to compare against the current value. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful swap. \
    /// @fail_ordering: Atomic ordering for a failed swap.
    ///
    /// ### Returns:
    /// The old value wrapped in a result.
    #[must_use = "If swap result is not used, consider store_if instead."]
    fn swap_if(
        &self, 
        target: Self::Value, 
        new_value: Self::Value,
        success_ordering: Ordering,
        fail_ordering: Ordering,
    ) -> Result<Self::Value, Self::Value>;

    /// Swaps the value if it is not contained inside a provided list of forbidden values.
    ///
    /// If the value doesn't exist inside the array list, it swaps and returns an Ok result with the
    /// value. If not, the value is returned in an Err()
    ///
    /// ### Params:
    /// @forbidden_list: An array containing the list of forbidden values. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful swap. \
    /// @fail_ordering: Atomic ordering for a failed swap.
    ///
    /// ### Returns:
    /// The old value wrapped in a result.
    #[must_use = "If swap result is not used, consider store_if_not_any instead."]
    fn swap_if_not_any(
        &self,
        forbidden_list: &[Self::Value],
        new_value: Self::Value,
        success_ordering: Ordering,
        fail_ordering: Ordering,
    ) -> Result<Self::Value, Self::Value>;

    /// Swaps the value if it exists inside a provided list of targeted values.
    ///
    /// If the value exists inside the array list, it swaps and returns an Ok result with the value.
    /// If not, the value is returned in an Err()
    ///
    /// ### Params:
    /// @target_list: An array containing the list of targeted values. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful swap. \
    /// @fail_ordering: Atomic ordering for a failed swap. \
    #[must_use = "If swap result is not used, consider store_if_any instead."]
    fn swap_if_any(
        &self,
        target_list: &[Self::Value],
        new_value: Self::Value,
        success_ordering: Ordering,
        fail_ordering: Ordering,
    ) -> Result<Self::Value, Self::Value>;

    /// Stores the value if its not equal the provided forbidden value.
    ///
    /// If the current value doesn't match, it stores it and returns true. If not, false is returned 
    /// instead.
    ///
    /// ### Params:
    /// @forbidden: The value to compare against the current value. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful store. \
    /// @fail_ordering: Atomic ordering for a failed stores.
    ///
    /// ### Returns:
    /// The bool stating if the operation was executed successfully.
    #[must_use = "Atomic store condition result should be checked."]
    fn store_if_not(
        &self,
        forbidden: Self::Value,
        new_value: Self::Value,
        success_ordering: Ordering,
        fail_ordering: Ordering
    ) -> bool;

    /// Stores the value if its equal to the provided target value.
    ///
    /// If the current value matches, it stores it and return true. If not, false is returned
    /// instead.
    ///
    /// ### Params:
    /// @target: The value to compare against the current value. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful store. \
    /// @fail_ordering: Atomic ordering for a failed store.
    ///
    /// ### Returns:
    /// The bool stating if the operation was executed successfully.
    #[must_use = "Atomic store condition result should be checked."]
    fn store_if(
        &self,
        target: Self::Value,
        new_value: Self::Value,
        success_ordering: Ordering,
        fail_ordering: Ordering,
    ) -> bool;

    /// Stores the value if it is not contained inside a provided list of forbidden values.
    ///
    /// If the value doesn't exist inside the array list, it stores it and returns true . If not, 
    /// false is returned instead.
    ///
    /// ### Params:
    /// @forbidden_list: An array containing the list of forbidden values. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful store. \
    /// @fail_ordering: Atomic ordering for a failed store.
    ///
    /// ### Returns:
    /// The bool stating if the operation was executed successfully.
    #[must_use = "Atomic store condition result should be checked."]
    fn store_if_not_any(
        &self,
        forbidden_list: &[Self::Value],
        new_value: Self::Value,
        success_ordering: Ordering,
        fail_ordering: Ordering,
    ) -> bool;

    /// Stores the value if it exists inside a provided list of target values.
    ///
    /// If the value exists inside the array list, it stores it and returns true. If not, false is
    /// returned instead.
    ///
    /// ### Params:
    /// @forbidden_list: An array containing the list of forbidden values. \
    /// @new_value: The value to replace if the expression matches. \
    /// @success_ordering: Atomic ordering for a successful store. \
    /// @fail_ordering: Atomic ordering for a failed store.
    ///
    /// ### Returns:
    /// The bool stating if the operation was executed successfully.
    #[must_use = "Atomic store condition result should be checked."]
    fn store_if_any(
        &self,
        target_list: &[Self::Value],
        new_value: Self::Value,
        success_ordering: Ordering,
        fail_ordering: Ordering
    ) -> bool;
}

macro_rules! impl_atomic_extensions {
    ($atomic:ty, $value:ty) => {
        impl AtomicExtensions for $atomic {
            type Value = $value;

            fn swap_if_not(
                &self,
                forbidden: $value,
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering,
            ) -> Result<$value, $value> {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (cur != forbidden).then_some(new_value)
                })
            }

            fn swap_if(
                &self,
                target: $value,
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering,
            ) -> Result<$value, $value> {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (cur == target).then_some(new_value)
                })
            }

            fn swap_if_not_any(
                &self,
                forbidden_list: &[$value],
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering,
            ) -> Result<$value, $value> {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (!forbidden_list.contains(&cur)).then_some(new_value)
                })
            }

            fn swap_if_any(
                &self,
                target_list: &[$value],
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering,
            ) -> Result<$value, $value> {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (target_list.contains(&cur)).then_some(new_value)
                })
            }

            fn store_if_not(
                &self,
                forbidden: $value,
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering,
            ) -> bool {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (cur != forbidden).then_some(new_value)
                })
                .is_ok()
            }

            fn store_if(
                &self,
                target: $value,
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering,
            ) -> bool {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (cur == target).then_some(new_value)
                })
                .is_ok()
            }

            fn store_if_not_any(
                &self,
                forbidden_list: &[$value],
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering
            ) -> bool {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (!forbidden_list.contains(&cur)).then_some(new_value)
                })
                .is_ok()
            }

            fn store_if_any(
                &self,
                target_list: &[$value],
                new_value: $value,
                success_ordering: Ordering,
                fail_ordering: Ordering,
            ) -> bool {
                self.fetch_update(success_ordering, fail_ordering, |cur| {
                    (target_list.contains(&cur)).then_some(new_value)
                })
                .is_ok()
            }
        }
    };
}

impl_atomic_extensions!(atomic::AtomicI8, i8);
impl_atomic_extensions!(atomic::AtomicI16, i16);
impl_atomic_extensions!(atomic::AtomicI32, i32);
impl_atomic_extensions!(atomic::AtomicI64, i64);
impl_atomic_extensions!(atomic::AtomicU8, u8);
impl_atomic_extensions!(atomic::AtomicU16, u16);
impl_atomic_extensions!(atomic::AtomicU32, u32);
impl_atomic_extensions!(atomic::AtomicU64, u64);
impl_atomic_extensions!(atomic::AtomicUsize, usize);
impl_atomic_extensions!(atomic::AtomicIsize, isize);
