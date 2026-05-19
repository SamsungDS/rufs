// SPDX-License-Identifier: GPL-2.0

//! UfsHost: Top-level host controller manager.

#![allow(dead_code)]

use kernel::{prelude::*, new_spinlock};
use kernel::sync::{Arc, SpinLock};

use crate::ufs_reg::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum HostState {
    Reset,
    Operational,
    EhNonFatal,
    EhFatal,
    Error,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct HostCaps {
    pub nutrs: u16,
    pub nutmrs: u8,
    pub autoh8: bool,
}

impl HostCaps {
    #[inline]
    fn from_cap_lo(cap_lo: u32) -> Self {
        let nutrs = decode_nutrs(cap_lo);
        let nutmrs = decode_nutmrs(cap_lo);
        let autoh8 = decode_autoh8(cap_lo);
        Self { nutrs, nutmrs, autoh8 }
    }

    #[inline]
    fn from_reg(reg: &Arc<UfsReg>) -> Self {
        Self::from_cap_lo(reg.read_cap_lo())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct HostVersion {
    pub raw: u32,
    pub major: u8,
    pub minor: u8,
    pub step: u8,
}

impl HostVersion {
    #[inline]
    fn from_raw(raw: u32 ) -> Self {
        let major = ((raw >> 24) & 0xff) as u8;
        let minor = ((raw >> 16) & 0xff) as u8;
        let step = ((raw >> 8) & 0xff) as u8;
        Self { raw, major, minor, step }
    }

    #[inline]
    fn from_reg(reg: &Arc<UfsReg>) -> Self {
        Self::from_raw(reg.read_version())
    }
}

#[pin_data]
pub(crate) struct UfsHost {
    reg: Arc<UfsReg>,

    caps: HostCaps,
    version: HostVersion,

    max_hw_queues: u16,
    max_prdt_entries: u16,

    #[pin]
    state: SpinLock<HostState>,
}

impl UfsHost {
    pub(crate) fn new(reg: Arc<UfsReg>) -> Result<Arc<Self>> {
        let caps = HostCaps::from_reg(&reg);
        let version = HostVersion::from_reg(&reg);

        let host = Arc::pin_init(
            pin_init!(Self {
                reg,
                state <- new_spinlock!(HostState::Reset),
                caps,
                version,
                max_hw_queues: 1,
                max_prdt_entries: 256,
            }), GFP_KERNEL,
        )?;

        Ok(host)
    }

    // getter
    #[inline] pub(crate) fn caps(&self) -> &HostCaps { &self.caps }
    #[inline] pub(crate) fn version(&self) -> &HostVersion { &self.version }
    #[inline] pub(crate) fn state(&self) -> HostState { *self.state.lock() }

    // state
    #[inline] fn set_state(&self, state: HostState) { *self.state.lock() = state; }
    #[inline] fn enter_eh_nonfatal(&self) { *self.state.lock() = HostState::EhNonFatal; }
    #[inline] fn enter_eh_fatal(&self) { *self.state.lock() = HostState::EhFatal; }
    #[inline] fn enter_error(&self) { *self.state.lock() = HostState::Error; }
    #[inline] fn promote_to_operational(&self) { *self.state.lock() = HostState::Operational; }
    #[inline] fn fallback_to_reset(&self) { *self.state.lock() = HostState::Reset; }

    // bring-up
    pub(crate) fn bring_up_controller(&self) -> Result<()> {
        if *self.state.lock() != HostState::Reset {
            return Err(EINVAL);
        }

        self.reg.clear_all_interrupts();
        self.reg.ctrl_enable();

        Ok(())
    }
}
