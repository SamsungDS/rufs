// SPDX-License-Identifier: GPL-2.0

//! Interconnect consumer abstractions.

use crate::{
    bindings,
    device::{Bound, Device},
    error::{from_err_ptr, to_result},
    prelude::*,
};

/// A device-managed interconnect path.
pub struct Path {
    ptr: *mut bindings::icc_path,
}

impl Path {
    /// Obtain an interconnect path by connection name.
    pub fn get(dev: &Device<Bound>, name: &CStr) -> Result<Self> {
        // SAFETY: `dev` is bound, so `dev.as_raw()` is a valid device pointer, and
        // `name.as_char_ptr()` is a valid C string pointer.
        let ptr =
            from_err_ptr(unsafe { bindings::devm_of_icc_get(dev.as_raw(), name.as_char_ptr()) })?;

        Ok(Self { ptr })
    }

    /// Set average and peak bandwidth in interconnect units.
    pub fn set_bw(&self, average: u32, peak: u32) -> Result {
        // SAFETY: By the C API contract of `devm_of_icc_get`, `self.ptr` is either null or
        // points to a valid interconnect path. `icc_set_bw` accepts a null path.
        to_result(unsafe { bindings::icc_set_bw(self.ptr, average, peak) })
    }
}

impl Drop for Path {
    fn drop(&mut self) {
        // Ignore teardown failures: devres still releases the path.
        let _ = self.set_bw(0, 0);
    }
}

// SAFETY: The interconnect core serializes path vote updates.
unsafe impl Send for Path {}

// SAFETY: The interconnect core serializes path vote updates.
unsafe impl Sync for Path {}
