// SPDX-License-Identifier: GPL-2.0

//! UfsHost: Top-level host controller manager.

#![allow(dead_code)]

use kernel::{device::Core, pci, prelude::*, new_spinlock};
use kernel::time::{Delta, delay::*};
use kernel::sync::{Arc, SpinLock};

use crate::ufs_reg::*;
use crate::ufs_dma::*;
use crate::ufs_irq::*;
use crate::ufs_uic::*;
use crate::ufs_queue::*;
use crate::ufs_dev::*;

const HBA_ENABLE_DELAY_US: i64 = 1000;

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
    irq: Arc<UfsIrq>,
    uic: Arc<UfsUic>,
    queue: Arc<UfsQueue>,
    dev: Arc<UfsDev>,

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

        let irq = UfsIrq::new()?;
        let uic = UfsUic::new(reg.clone(), irq.clone())?;
        let queue = UfsQueue::new(
            reg.clone(),
            irq.clone(),
            dma.clone(),
        )?;
        let dev = UfsDev::new(queue.clone())?;
        let host = Arc::pin_init(
            pin_init!(Self {
                reg,
                dma,
                irq,
                uic,
                queue,
                dev,
                state <- new_spinlock!(HostState::Reset),
                max_hw_queues: 1,
                max_prdt_entries: 256,
            }), GFP_KERNEL,
        )?;

        host.reg.ctrl_enable();
        fsleep(Delta::from_micros(HBA_ENABLE_DELAY_US));
        host.reg.wait_for_ctrl_enable(1000, 50)?;

        let irq_vectors = pdev.alloc_irq_vectors(1, 1, pci::IrqTypes::all())?;
        let irq_vector = *irq_vectors.start();

        /* ufshcd_link_startup() */
        host.irq.request_uic_irq(
            pdev,
            irq_vector,
            host.reg.clone(),
            host.uic.clone(),
        )?;
        host.uic.link_startup()?;
        host.dma.make_hba_operational()?;

        /* ufshcd_verify_dev_init */
        host.irq.request_queue_irq(
            pdev,
            irq_vector,
            host.reg.clone(),
            host.queue.clone(),
        )?;
        host.dev.verify_dev_init()?;
        host.dev.complete_dev_init()?;
        host.dev.device_params_init()?;
        host.dev.alloc_tmf_queue(host.reg.nutmrs())?;

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
