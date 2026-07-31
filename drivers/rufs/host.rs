// SPDX-License-Identifier: GPL-2.0

//! UfsHost: Top-level host controller manager.

#![allow(dead_code)]

use kernel::sync::{Arc, Mutex, SpinLock};
use kernel::time::{delay::*, Delta};
use kernel::types::ScopeGuard;
use kernel::{irq, new_mutex, new_spinlock, prelude::*};
use pin_init::pin_init_scope;

use crate::device::*;
use crate::dma::*;
use crate::irq::*;
use crate::lu::*;
use crate::queue::*;
use crate::reg::*;
use crate::resource::HostResources;
use crate::transport::UfsTransferConfig;
use crate::uic::*;
use crate::variant::NotifyPhase;

const HBA_ENABLE_DELAY_US: i64 = 1000;

fn stop_hba_controller(reg: &UfsReg) {
    if !reg.ctrl_enabled() {
        return;
    }

    reg.disable_interrupts();
    reg.clear_all_interrupts();
    reg.disable_run_stop();
    reg.ctrl_disable();
    if let Err(e) = reg.wait_for_ctrl_disable(10, 1) {
        pr_err!(
            "[RUFS] ufs_host: controller disable failed errno={}\n",
            e.to_errno()
        );
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum HostState {
    Reset,
    Operational,
    EhNonFatal,
    EhFatal,
    Error,
}

#[pin_data(PinnedDrop)]
pub(crate) struct UfsHost {
    resources: Arc<HostResources>,
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    irq: Arc<UfsIrq>,
    uic: Arc<UfsUic>,
    queue: Arc<UfsQueue>,
    dev: Arc<UfsDev>,

    #[pin]
    luns: Mutex<KVec<Arc<UfsLu>>>,

    max_hw_queues: u16,
    max_prdt_entries: u16,

    #[pin]
    state: SpinLock<HostState>,
}

impl UfsHost {
    pub(crate) fn new<'a>(
        resources: Arc<HostResources>,
        controller_irq: irq::IrqRequest<'a>,
    ) -> impl PinInit<Self, Error> + 'a {
        pin_init_scope(move || {
            let reg = UfsReg::new(resources.clone())?;
            let transfer_config = UfsTransferConfig::new(&reg)?;
            let dma = UfsDma::new(
                resources.device(),
                reg.clone(),
                transfer_config.tag_count(),
            )?;

            reg.clear_all_interrupts();
            reg.disable_interrupts();

            let irq = UfsIrq::new()?;
            let uic = UfsUic::new(reg.clone())?;
            let interrupt_policy = match transfer_config {
                UfsTransferConfig::Sdb { .. } => UfsInterruptPolicy::EagerAck,
                UfsTransferConfig::Mcq(_) => UfsInterruptPolicy::ThreadedAck,
            };

            let cleanup_reg = reg.clone();
            let init_guard = ScopeGuard::new(move || stop_hba_controller(&cleanup_reg));

            if reg.ctrl_enabled() {
                pr_info!("[RUFS] ufs_host: controller is active, stop before enable\n");
                stop_hba_controller(&reg);
            }

            resources
                .variant()
                .hce_enable_notify(&reg, NotifyPhase::Pre)?;
            reg.ctrl_enable();
            fsleep(Delta::from_micros(HBA_ENABLE_DELAY_US));
            reg.wait_for_ctrl_enable(1000, 50)?;
            resources
                .variant()
                .hce_enable_notify(&reg, NotifyPhase::Post)?;

            /* ufshcd_link_startup() */
            irq.request_controller_irq(
                controller_irq,
                reg.clone(),
                uic.clone(),
                interrupt_policy,
            )?;
            resources
                .variant()
                .link_startup_notify(&reg, NotifyPhase::Pre)?;
            uic.link_startup()?;
            resources
                .variant()
                .link_startup_notify(&reg, NotifyPhase::Post)?;
            dma.make_hba_operational()?;

            let ufs_queue = UfsQueue::new(transfer_config, reg.clone(), dma.clone())?;
            let dev = UfsDev::new(ufs_queue.clone())?;
            let host = try_pin_init!(Self {
                resources,
                reg,
                dma,
                irq,
                uic,
                queue: ufs_queue,
                dev,
                luns <- new_mutex!(KVec::new()),
                state <- new_spinlock!(HostState::Reset),
                max_hw_queues: 1,
                max_prdt_entries: 256,
            })
            .pin_chain(move |host| {
                init_guard.dismiss();

                /* ufshcd_verify_dev_init */
                host.irq.attach_queue(host.queue.clone())?;
                host.dev.verify_dev_init()?;
                host.dev.complete_dev_init()?;
                host.dev.device_params_init()?;
                if let Err(e) = host.configure_power_mode() {
                    pr_warn!(
                        "[RUFS] ufs_host: power mode configuration failed errno={}, continue with current mode\n",
                        e.to_errno(),
                    );
                }
                host.alloc_luns()?;
                host.dev.alloc_tmf_queue(host.reg.nutmrs())?;
                Ok(())
            });

            Ok(host)
        })
    }

    fn configure_power_mode(&self) -> Result<()> {
        let variant = self.resources.variant();
        let mode = variant.constrain_power_mode(self.uic.max_power_mode()?)?;

        variant.power_mode_notify(&self.reg, mode, NotifyPhase::Pre)?;
        self.uic.change_power_mode(mode)?;
        variant.power_mode_notify(&self.reg, mode, NotifyPhase::Post)
    }

    fn alloc_luns(&self) -> Result<()> {
        let num_lu = self.dev.num_lu();
        let mut luns = self.luns.lock();
        let mut lun = 0;

        while lun < num_lu {
            let lun_id = u8::try_from(lun).map_err(|_| EOVERFLOW)?;
            let desc = self.dev.read_unit_desc(lun_id)?;

            if !desc.enabled() {
                lun = lun.checked_add(1).ok_or(EOVERFLOW)?;
                continue;
            }

            let geometry = UfsLuGeometry::from_logical_block_shift(
                desc.logical_block_shift(),
                desc.logical_block_count(),
            )?;
            let queue_depth = match desc.lu_queue_depth() {
                0 => self.queue.tags.queue_depth(),
                depth => core::cmp::min(depth as u32, self.queue.tags.queue_depth()),
            };
            let lu = UfsLu::new(
                self.queue.clone(),
                lun_id,
                geometry,
                queue_depth,
            )?;
            lu.init_disk()?;

            pr_info!(
                "[RUFS] ufs_host: allocated LU {} capacity={} logical_block_size={} queue_depth={}",
                lun,
                geometry.capacity_blocks(),
                geometry.logical_block_size(),
                queue_depth,
            );

            luns.push(lu, GFP_KERNEL)?;
            lun = lun.checked_add(1).ok_or(EOVERFLOW)?;
        }

        Ok(())
    }

    pub(crate) fn remove(&self) {
        self.remove_luns();
        self.hba_stop();
    }

    fn remove_luns(&self) {
        let mut luns = self.luns.lock();
        for lu in luns.iter() {
            lu.remove_disk();
        }
        luns.clear();
    }

    fn hba_stop(&self) {
        stop_hba_controller(&self.reg);
        self.fallback_to_reset();
    }

    // getter
    #[inline]
    pub(crate) fn state(&self) -> HostState {
        *self.state.lock()
    }

    // state
    #[inline]
    fn set_state(&self, state: HostState) {
        *self.state.lock() = state;
    }
    #[inline]
    fn enter_eh_nonfatal(&self) {
        *self.state.lock() = HostState::EhNonFatal;
    }
    #[inline]
    fn enter_eh_fatal(&self) {
        *self.state.lock() = HostState::EhFatal;
    }
    #[inline]
    fn enter_error(&self) {
        *self.state.lock() = HostState::Error;
    }
    #[inline]
    fn promote_to_operational(&self) {
        *self.state.lock() = HostState::Operational;
    }
    #[inline]
    fn fallback_to_reset(&self) {
        *self.state.lock() = HostState::Reset;
    }
}

#[pinned_drop]
impl PinnedDrop for UfsHost {
    fn drop(self: Pin<&mut Self>) {
        self.remove();
    }
}
