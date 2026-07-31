// SPDX-License-Identifier: GPL-2.0

//! Host-controller variant operations.

use crate::reg::UfsReg;
use crate::uic::UfsPaLayerAttr;
use kernel::prelude::*;

#[derive(Clone, Copy)]
pub(crate) enum NotifyPhase {
    Pre,
    Post,
}

pub(crate) trait UfsVariantOps: Send + Sync {
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
