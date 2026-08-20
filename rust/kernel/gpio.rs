// SPDX-License-Identifier: GPL-2.0

//! GPIO consumer abstractions.

use crate::{
    bindings,
    device::{Bound, Device},
    error::{from_err_ptr, to_result},
    prelude::*,
};
use core::ptr;

/// A device-managed optional GPIO configured as a logical-low output.
pub struct OptionalOutput {
    ptr: *mut bindings::gpio_desc,
}

impl OptionalOutput {
    /// Obtain an optional output GPIO by connection name.
    pub fn get(dev: &Device<Bound>, name: &CStr) -> Result<Self> {
        let ptr = from_err_ptr(unsafe {
            bindings::devm_gpiod_get_optional_output_low(dev.as_raw(), name.as_char_ptr())
        })?;

        Ok(Self { ptr })
    }

    /// Return whether firmware supplied the GPIO.
    pub fn is_present(&self) -> bool {
        self.ptr != ptr::null_mut()
    }

    /// Set the logical output value when the GPIO is present.
    pub fn set_value(&self, value: bool) -> Result {
        if !self.is_present() {
            return Ok(());
        }

        to_result(unsafe { bindings::gpiod_set_value_cansleep(self.ptr, value.into()) })
    }
}

// SAFETY: GPIO descriptors may be used from different sleepable contexts.
unsafe impl Send for OptionalOutput {}
unsafe impl Sync for OptionalOutput {}
