// SPDX-License-Identifier: GPL-2.0

//! Reset controller consumer abstractions.

use crate::{
    bindings,
    device::{Bound, Device},
    error::{from_err_ptr, to_result},
    prelude::*,
};
use core::ptr;

/// A device-managed optional exclusive reset control.
pub struct OptionalExclusive {
    ptr: *mut bindings::reset_control,
}

impl OptionalExclusive {
    /// Obtain an optional exclusive reset control by name.
    pub fn get(dev: &Device<Bound>, name: &CStr) -> Result<Self> {
        let ptr = from_err_ptr(unsafe {
            bindings::devm_reset_control_get_optional_exclusive(dev.as_raw(), name.as_char_ptr())
        })?;

        Ok(Self { ptr })
    }

    /// Assert the reset line when present.
    pub fn assert(&self) -> Result {
        if self.ptr == ptr::null_mut() {
            return Ok(());
        }
        to_result(unsafe { bindings::reset_control_assert(self.ptr) })
    }

    /// Deassert the reset line when present.
    pub fn deassert(&self) -> Result {
        if self.ptr == ptr::null_mut() {
            return Ok(());
        }
        to_result(unsafe { bindings::reset_control_deassert(self.ptr) })
    }
}

// SAFETY: The reset controller core serializes consumer operations.
unsafe impl Send for OptionalExclusive {}
unsafe impl Sync for OptionalExclusive {}
