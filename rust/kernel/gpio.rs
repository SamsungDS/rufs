// SPDX-License-Identifier: GPL-2.0

//! GPIO consumer abstractions.

use crate::{
    bindings,
    device::{Bound, Device},
    error::{from_err_ptr, to_result},
    prelude::*,
};

/// A device-managed optional GPIO configured as a logical-low output.
pub struct OptionalOutput {
    ptr: *mut bindings::gpio_desc,
}

impl OptionalOutput {
    /// Obtain an optional output GPIO with an initial logical value.
    pub fn get(dev: &Device<Bound>, name: &CStr, initial: bool) -> Result<Self> {
        // SAFETY: `dev` is bound, so `dev.as_raw()` is a valid device pointer, and
        // `name.as_char_ptr()` is a valid C string pointer.
        let ptr = from_err_ptr(unsafe {
            bindings::devm_gpiod_get_optional_output(dev.as_raw(), name.as_char_ptr(), initial)
        })?;

        Ok(Self { ptr })
    }

    /// Return whether firmware supplied the GPIO.
    pub fn is_present(&self) -> bool {
        !self.ptr.is_null()
    }

    /// Set the logical output value when the GPIO is present.
    pub fn set_value(&self, value: bool) -> Result {
        if !self.is_present() {
            return Ok(());
        }

        // SAFETY: `self.ptr` is non-null as checked above, and by the C API contract of
        // `devm_gpiod_get_optional`, it points to a valid GPIO descriptor.
        to_result(unsafe { bindings::gpiod_set_value_cansleep(self.ptr, value.into()) })
    }
}

// SAFETY: GPIO descriptors may be used from different sleepable contexts.
unsafe impl Send for OptionalOutput {}

// SAFETY: GPIO descriptors may be used from different sleepable contexts.
unsafe impl Sync for OptionalOutput {}
