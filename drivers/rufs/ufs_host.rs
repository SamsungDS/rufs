// SPDX-License-Identifier: GPL-2.0

//! UfsHost: Top-level host controller manager.

#![allow(dead_code)]

use kernel::{device::Core, pci, prelude::*, new_spinlock};
use kernel::sync::{Arc, SpinLock};

use crate::ufs_reg::*;
use crate::ufs_dma::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum HostState {
    Reset,
    Operational,
    EhNonFatal,
    EhFatal,
    Error,
}

#[pin_data]
pub(crate) struct UfsHost {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,

    max_hw_queues: u16,
    max_prdt_entries: u16,

    #[pin]
    state: SpinLock<HostState>,
}

impl UfsHost {
    pub(crate) fn new(pdev: &pci::Device<Core>) -> Result<Arc<Self>> {
        let reg = UfsReg::new(pdev)?;
        let dma = UfsDma::new(pdev, reg.clone())?;

        reg.clear_all_interrupts();
        reg.disable_interrupts();

        let host = Arc::pin_init(
            pin_init!(Self {
                reg,
                dma,
                state <- new_spinlock!(HostState::Reset),
                max_hw_queues: 1,
                max_prdt_entries: 256,
            }), GFP_KERNEL,
        )?;

        Ok(host)
    }

    // getter
    #[inline] pub(crate) fn state(&self) -> HostState { *self.state.lock() }

    // state
    #[inline] fn set_state(&self, state: HostState) { *self.state.lock() = state; }
    #[inline] fn enter_eh_nonfatal(&self) { *self.state.lock() = HostState::EhNonFatal; }
    #[inline] fn enter_eh_fatal(&self) { *self.state.lock() = HostState::EhFatal; }
    #[inline] fn enter_error(&self) { *self.state.lock() = HostState::Error; }
    #[inline] fn promote_to_operational(&self) { *self.state.lock() = HostState::Operational; }
    #[inline] fn fallback_to_reset(&self) { *self.state.lock() = HostState::Reset; }
}
