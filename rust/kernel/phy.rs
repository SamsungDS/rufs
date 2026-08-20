// SPDX-License-Identifier: GPL-2.0

//! Generic PHY consumer abstractions.

use crate::{
    bindings,
    device::{Bound, Device},
    error::{from_err_ptr, to_result},
    prelude::*,
};
use core::sync::atomic::{AtomicBool, Ordering};

/// A UFS high-speed PHY mode.
#[derive(Clone, Copy)]
pub enum UfsMode {
    /// High-speed series A.
    HighSpeedA,
    /// High-speed series B.
    HighSpeedB,
}

/// A device-managed generic PHY consumer handle.
pub struct Phy {
    ptr: *mut bindings::phy,
    initialized: AtomicBool,
    powered: AtomicBool,
}

impl Phy {
    /// Obtain a device-managed PHY by connection name.
    pub fn get(dev: &Device<Bound>, name: &CStr) -> Result<Self> {
        let ptr =
            from_err_ptr(unsafe { bindings::devm_phy_get(dev.as_raw(), name.as_char_ptr()) })?;

        Ok(Self {
            ptr,
            initialized: AtomicBool::new(false),
            powered: AtomicBool::new(false),
        })
    }

    /// Initialize the PHY.
    pub fn init(&self) -> Result {
        to_result(unsafe { bindings::phy_init(self.ptr) })?;
        self.initialized.store(true, Ordering::Release);
        Ok(())
    }

    /// Select a UFS high-speed mode and gear.
    pub fn set_ufs_mode(&self, mode: UfsMode, gear: i32) -> Result {
        let rate_b = matches!(mode, UfsMode::HighSpeedB);

        to_result(unsafe { bindings::phy_set_ufs_mode(self.ptr, rate_b, gear) })
    }

    /// Power on the PHY.
    pub fn power_on(&self) -> Result {
        to_result(unsafe { bindings::phy_power_on(self.ptr) })?;
        self.powered.store(true, Ordering::Release);
        Ok(())
    }

    /// Calibrate the PHY.
    pub fn calibrate(&self) -> Result {
        to_result(unsafe { bindings::phy_calibrate(self.ptr) })
    }

    /// Power off and exit the PHY if this handle initialized it.
    pub fn shutdown(&self) {
        if self.powered.swap(false, Ordering::AcqRel) {
            unsafe { bindings::phy_power_off(self.ptr) };
        }
        if self.initialized.swap(false, Ordering::AcqRel) {
            unsafe { bindings::phy_exit(self.ptr) };
        }
    }
}

impl Drop for Phy {
    fn drop(&mut self) {
        self.shutdown();
    }
}

// SAFETY: The PHY core serializes consumer operations. Users must still
// serialize lifecycle transitions performed through one `Phy` instance.
unsafe impl Send for Phy {}
unsafe impl Sync for Phy {}
