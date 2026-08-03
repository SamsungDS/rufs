// SPDX-License-Identifier: GPL-2.0

//! Host-controller variant operations.

use crate::reg::{McqRegisterLayout, UfsReg};
use crate::uic::UfsPaLayerAttr;
use kernel::prelude::*;

#[derive(Clone, Copy)]
pub(crate) enum NotifyPhase {
    Pre,
    Post,
}

pub(crate) trait UfsVariantOps: Send + Sync {
    /// Prepare controller-specific resources before common host initialization.
    fn initialize(&self, _reg: &UfsReg) -> Result<()> {
        Ok(())
    }

    /// Release controller-specific resources after the common host is stopped.
    fn shutdown(&self, _reg: &UfsReg) {}

    fn mcq_register_layout(&self, reg: &UfsReg) -> Result<McqRegisterLayout> {
        reg.standard_mcq_register_layout()
    }

    fn hce_enable_notify(&self, _reg: &UfsReg, _phase: NotifyPhase) -> Result<()> {
        Ok(())
    }

    fn link_startup_notify(&self, _reg: &UfsReg, _phase: NotifyPhase) -> Result<()> {
        Ok(())
    }

    fn constrain_power_mode(&self, desired: UfsPaLayerAttr) -> Result<UfsPaLayerAttr> {
        Ok(desired)
    }

    fn power_mode_notify(
        &self,
        _reg: &UfsReg,
        _mode: UfsPaLayerAttr,
        _phase: NotifyPhase,
    ) -> Result<()> {
        Ok(())
    }
}
