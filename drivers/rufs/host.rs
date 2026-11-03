// SPDX-License-Identifier: GPL-2.0

//! UfsHost: Top-level host controller manager.

#![allow(dead_code)]

use kernel::io::poll::read_poll_timeout;
use kernel::sync::{Arc, Mutex, MutexGuard, SpinLock};
use kernel::time::{delay::*, Delta};
use kernel::types::ScopeGuard;
use kernel::workqueue::{self, impl_has_work, new_work, Work, WorkItem};
use kernel::{irq, new_mutex, new_spinlock, prelude::*};
use pin_init::pin_init_scope;

use crate::device::*;
use crate::dma::*;
use crate::irq::*;
use crate::lu::*;
use crate::protocol::query::UfsDevCmd;
use crate::queue::*;
use crate::reg::*;
use crate::resource::HostResources;
use crate::task_management::TaskManagement;
use crate::transport::UfsTransferConfig;
use crate::uic::*;
use crate::variant::{NotifyPhase, UfsVariantOps};

const HBA_ENABLE_DELAY_US: i64 = 1000;
const LINK_STARTUP_RETRIES: usize = 3;
const SHUTDOWN_DRAIN_INTERVAL_MS: i64 = 1;
const SHUTDOWN_DRAIN_TIMEOUT_SECS: i64 = 30;

#[derive(Clone, Copy, Debug)]
pub(crate) enum RecoveryReason {
    Timeout,
    Driver(&'static str),
    Uic(UicErrorStatus),
}

impl RecoveryReason {
    pub(crate) fn name(&self) -> &'static str {
        match *self {
            Self::Timeout => "request timeout",
            Self::Driver(reason) => reason,
            Self::Uic(_) => "fatal UIC error",
        }
    }

    pub(crate) fn uic_errors(&self) -> Option<UicErrorStatus> {
        match *self {
            Self::Uic(errors) => Some(errors),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct RecoveryCause {
    pub(crate) reason: RecoveryReason,
    pub(crate) source_queue: Option<u32>,
    pub(crate) context: usize,
}

impl RecoveryCause {
    fn merge(self, other: Self) -> Self {
        match (self.reason, other.reason) {
            (RecoveryReason::Uic(_), _) => self,
            (_, RecoveryReason::Uic(_)) => other,
            (RecoveryReason::Driver(_), _) => self,
            (_, RecoveryReason::Driver(_)) => other,
            _ => self,
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) enum RecoveryState {
    Idle,
    Requested(RecoveryCause),
    Quiescing(RecoveryCause),
    Recovering(RecoveryCause),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum HostLifecycle {
    Starting,
    Operational,
    Draining,
    Stopping,
    Failed,
    Stopped,
}

pub(crate) struct RecoveryControl {
    pub(crate) lifecycle: HostLifecycle,
    pub(crate) state: RecoveryState,
    pub(crate) pending: Option<RecoveryCause>,
}

pub(crate) enum RecoveryOutcome {
    Pending(RecoveryCause),
    Resume,
    Stop,
}

impl RecoveryControl {
    fn new() -> Self {
        Self {
            lifecycle: HostLifecycle::Starting,
            state: RecoveryState::Idle,
            pending: None,
        }
    }

    fn cause(&self) -> Option<RecoveryCause> {
        match self.state {
            RecoveryState::Idle => None,
            RecoveryState::Requested(cause)
            | RecoveryState::Quiescing(cause)
            | RecoveryState::Recovering(cause) => Some(cause),
        }
    }

    fn request(&mut self, cause: RecoveryCause) -> (bool, bool) {
        if !matches!(
            self.lifecycle,
            HostLifecycle::Operational | HostLifecycle::Draining
        ) {
            return (false, false);
        }
        match self.state {
            RecoveryState::Idle => {
                self.state = RecoveryState::Requested(cause);
                (true, true)
            }
            RecoveryState::Requested(current) => {
                self.state = RecoveryState::Requested(current.merge(cause));
                (true, false)
            }
            RecoveryState::Quiescing(current) => {
                self.state = RecoveryState::Quiescing(current.merge(cause));
                (true, false)
            }
            RecoveryState::Recovering(_) => {
                self.pending = Some(match self.pending {
                    Some(current) => current.merge(cause),
                    None => cause,
                });
                (true, false)
            }
        }
    }

    pub(crate) fn begin(&mut self) -> Option<RecoveryCause> {
        let RecoveryState::Requested(cause) = self.state else {
            return None;
        };
        self.state = RecoveryState::Quiescing(cause);
        Some(cause)
    }

    pub(crate) fn start_plan(&mut self) -> Option<RecoveryCause> {
        let RecoveryState::Quiescing(cause) = self.state else {
            return None;
        };
        self.state = RecoveryState::Recovering(cause);
        Some(cause)
    }

    pub(crate) fn refresh_plan(&mut self, cause: RecoveryCause) -> RecoveryCause {
        let cause = match self.pending.take() {
            Some(pending) => cause.merge(pending),
            None => cause,
        };
        if matches!(self.state, RecoveryState::Recovering(_)) {
            self.state = RecoveryState::Recovering(cause);
        }
        cause
    }

    pub(crate) fn finish_plan(&mut self) -> RecoveryOutcome {
        if matches!(self.lifecycle, HostLifecycle::Stopping | HostLifecycle::Stopped) {
            self.pending = None;
            self.state = RecoveryState::Idle;
            return RecoveryOutcome::Stop;
        }
        match self.state {
            RecoveryState::Recovering(current) => match self.pending.take() {
                Some(cause) => {
                    let cause = current.merge(cause);
                    self.state = RecoveryState::Recovering(cause);
                    RecoveryOutcome::Pending(cause)
                }
                None => {
                    self.state = RecoveryState::Idle;
                    RecoveryOutcome::Resume
                }
            },
            _ => RecoveryOutcome::Stop,
        }
    }

    pub(crate) fn fail(&mut self, _cause: RecoveryCause) {
        self.pending = None;
        self.state = RecoveryState::Idle;
        if !matches!(self.lifecycle, HostLifecycle::Stopping | HostLifecycle::Stopped) {
            self.lifecycle = HostLifecycle::Failed;
        }
    }
}

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

fn enable_hba_controller(resources: &HostResources, reg: &UfsReg) -> Result {
    resources.variant().device_reset()?;
    resources
        .variant()
        .hce_enable_notify(reg, NotifyPhase::Pre)?;
    reg.ctrl_enable();
    fsleep(Delta::from_micros(HBA_ENABLE_DELAY_US));
    reg.wait_for_ctrl_enable(1000, 50)?;
    resources
        .variant()
        .hce_enable_notify(reg, NotifyPhase::Post)
}

fn start_link(resources: &HostResources, reg: &UfsReg, uic: &Arc<UfsUic>) -> Result<bool> {
    resources
        .variant()
        .link_startup_notify(reg, uic, NotifyPhase::Pre)?;
    uic.link_startup()?;
    resources
        .variant()
        .link_startup_notify(reg, uic, NotifyPhase::Post)?;
    resources.variant().link_startup_valid(uic)
}

/// Ref-counted portion of `UfsHost` shared with asynchronous callbacks.
///
/// This is not an independent recovery controller: it carries the Host state
/// and schedules Host-owned recovery work. QueueData and IRQ handlers may
/// retain it, while `UfsQueue` never references it, avoiding a Host/Queue
/// ownership cycle.
#[pin_data]
pub(crate) struct UfsHostShared {
    pub(crate) queue: Arc<UfsQueue>,
    pub(crate) resources: Arc<HostResources>,
    pub(crate) reg: Arc<UfsReg>,
    pub(crate) dma: Arc<UfsDma>,
    pub(crate) uic: Arc<UfsUic>,
    pub(crate) task_management: Arc<TaskManagement>,
    #[pin]
    pub(crate) control_lock: Mutex<()>,
    #[pin]
    pub(crate) recovery: SpinLock<RecoveryControl>,
    #[pin]
    pub(crate) recovery_requests: Mutex<KVec<RecoveryRequest>>,
    #[pin]
    recovery_work: Work<UfsHostShared>,
}

impl_has_work! {
    impl HasWork<Self> for UfsHostShared { self.recovery_work }
}

impl UfsHostShared {
    fn new(
        queue: Arc<UfsQueue>,
        resources: Arc<HostResources>,
        reg: Arc<UfsReg>,
        dma: Arc<UfsDma>,
        uic: Arc<UfsUic>,
        task_management: Arc<TaskManagement>,
    ) -> Result<Arc<Self>> {
        let recovery_requests =
            KVec::with_capacity(queue.tags.queue_depth() as usize, GFP_KERNEL)?;
        Arc::pin_init(
            try_pin_init!(Self {
                queue,
                resources,
                reg,
                dma,
                uic,
                task_management,
                control_lock <- new_mutex!(()),
                recovery <- new_spinlock!(RecoveryControl::new()),
                recovery_requests <- new_mutex!(recovery_requests),
                recovery_work <- new_work!("UfsHostShared::work"),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn recovery_required(&self) -> bool {
        self.recovery.lock().cause().is_some()
    }

    fn promote_to_operational(&self) -> Result<()> {
        let mut control = self.recovery.lock();
        if control.lifecycle != HostLifecycle::Starting
            || !matches!(control.state, RecoveryState::Idle)
        {
            return Err(EINVAL);
        }
        control.lifecycle = HostLifecycle::Operational;
        Ok(())
    }

    fn begin_draining(&self) -> bool {
        let mut control = self.recovery.lock();
        match control.lifecycle {
            HostLifecycle::Starting | HostLifecycle::Operational => {
                control.lifecycle = HostLifecycle::Draining;
                true
            }
            HostLifecycle::Draining
            | HostLifecycle::Stopping
            | HostLifecycle::Failed
            | HostLifecycle::Stopped => false,
        }
    }

    pub(crate) fn require_recovery(
        self: &Arc<Self>,
        reason: &'static str,
        tag: usize,
    ) {
        self.request_recovery(RecoveryCause {
            reason: RecoveryReason::Driver(reason),
            source_queue: None,
            context: tag,
        });
    }

    pub(crate) fn require_timeout_recovery(self: &Arc<Self>, tag: usize) {
        self.request_recovery(RecoveryCause {
            reason: RecoveryReason::Timeout,
            source_queue: None,
            context: tag,
        });
    }

    pub(crate) fn require_uic_recovery(
        self: &Arc<Self>,
        errors: UicErrorStatus,
    ) {
        self.request_recovery(RecoveryCause {
            reason: RecoveryReason::Uic(errors),
            source_queue: None,
            context: 0,
        });
    }

    pub(crate) fn require_mcq_recovery(
        self: &Arc<Self>,
        reason: &'static str,
        queue_id: u32,
        context: usize,
    ) {
        self.request_recovery(RecoveryCause {
            reason: RecoveryReason::Driver(reason),
            source_queue: Some(queue_id),
            context,
        });
    }

    fn request_recovery(self: &Arc<Self>, cause: RecoveryCause) {
        let (accepted, schedule) = {
            let mut state = self.recovery.lock();
            state.request(cause)
        };

        if accepted {
            pr_err!(
                "[RUFS] ufs_host: recovery required reason={} queue={:?} tag={}\n",
                cause.reason.name(),
                cause.source_queue,
                cause.context,
            );
        }
        if schedule {
            let _ = workqueue::system_default().enqueue(self.clone());
        }
    }

    pub(crate) fn lock_control(&self) -> MutexGuard<'_, ()> {
        self.control_lock.lock()
    }

    pub(crate) fn begin_shutdown(&self) {
        let mut control = self.recovery.lock();
        control.lifecycle = HostLifecycle::Stopping;
        control.pending = None;
        if !matches!(control.state, RecoveryState::Recovering(_)) {
            control.state = RecoveryState::Idle;
        }
        drop(control);
        self.queue.tags.quiesce();
    }

    fn finish_shutdown(&self) {
        let mut control = self.recovery.lock();
        control.state = RecoveryState::Idle;
        control.pending = None;
        control.lifecycle = HostLifecycle::Stopped;
    }

    pub(crate) fn flush_work(self: &Arc<Self>) {
        // SAFETY: This object was pin-initialized in an Arc and remains pinned
        // while this reference is alive.
        let work = unsafe { Pin::new_unchecked(&self.recovery_work) };
        work.flush();
    }

    fn stop_controller(&self) -> Result<DmaAccessEnded> {
        self.reg.disable_interrupts();
        self.reg.clear_all_interrupts();
        self.reg.disable_run_stop();
        if self.reg.ctrl_enabled() {
            self.reg.ctrl_disable();
            self.reg.wait_for_ctrl_disable(10, 10)?;
        }
        Ok(DmaAccessEnded)
    }

    pub(crate) fn reset_controller(&self) -> ControllerResetResult {
        let dma_ended = match self.stop_controller() {
            Ok(dma_ended) => dma_ended,
            Err(stop_error) => {
                return ControllerResetResult::UnsafeStop {
                    reset_error: None,
                    stop_error,
                }
            }
        };

        let result = (|| {
            let variant = self.resources.variant();
            variant.device_reset()?;
            variant.hce_enable_notify(&self.reg, NotifyPhase::Pre)?;
            self.reg.ctrl_enable();
            fsleep(Delta::from_micros(HBA_ENABLE_DELAY_US));
            self.reg.wait_for_ctrl_enable(1000, 50)?;
            variant.hce_enable_notify(&self.reg, NotifyPhase::Post)?;

            variant.link_startup_notify(&self.reg, &self.uic, NotifyPhase::Pre)?;
            self.uic.link_startup()?;
            variant.link_startup_notify(&self.reg, &self.uic, NotifyPhase::Post)?;

            // Restore common descriptor state and the active transfer backend
            // before accepting transfer interrupts again.
            self.dma.make_hba_operational()?;
            let recovery_transfer = self.queue.prepare_recovery_transfer(dma_ended)?;
            self.task_management.reset_after_controller_reset()?;
            let (recovery_transfer, _) =
                recovery_transfer.execute_device_command(UfsDevCmd::Nop)?;
            self.queue.enable_interrupts();

            if let Err(e) = self.restore_power_mode(variant) {
                pr_warn!(
                    "[RUFS] ufs_host: recovery power mode restore failed errno={}, continue\n",
                    e.to_errno(),
                );
            }
            Ok(recovery_transfer.finish())
        })();

        match result {
            Ok(dma_ended) => ControllerResetResult::Restored(dma_ended),
            Err(error) => match self.stop_controller() {
                Ok(dma_ended) => ControllerResetResult::Stopped { dma_ended, error },
                Err(stop_error) => ControllerResetResult::UnsafeStop {
                    reset_error: Some(error),
                    stop_error,
                },
            },
        }
    }

    fn restore_power_mode(&self, variant: &dyn UfsVariantOps) -> Result<()> {
        let mode = variant.constrain_power_mode(self.uic.max_power_mode()?)?;
        variant.power_mode_notify(&self.reg, &self.uic, mode, NotifyPhase::Pre)?;
        self.uic.change_power_mode(mode)?;
        variant.power_mode_notify(&self.reg, &self.uic, mode, NotifyPhase::Post)
    }
}

impl WorkItem for UfsHostShared {
    type Pointer = Arc<Self>;

    fn run(this: Arc<Self>) {
        UfsHostShared::run_recovery(this);
    }
}

#[pin_data(PinnedDrop)]
pub(crate) struct UfsHost<'a> {
    resources: Arc<HostResources>,
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    irq: Arc<UfsIrq<'a>>,
    uic: Arc<UfsUic>,
    task_management: Arc<TaskManagement>,
    queue: Arc<UfsQueue>,
    shared: Arc<UfsHostShared>,
    dev: Arc<UfsDev>,

    #[pin]
    luns: Mutex<KVec<Arc<UfsLu>>>,

    max_hw_queues: u16,
    max_prdt_entries: u16,

}

impl<'a> UfsHost<'a> {
    pub(crate) fn new(
        resources: Arc<HostResources>,
        uic_irq: irq::IrqRequest<'a>,
        queue_irq: irq::IrqRequest<'a>,
    ) -> impl PinInit<Self, Error> + 'a {
        pin_init_scope(move || {
            let reg = UfsReg::new(resources.clone())?;
            resources.variant().initialize(&reg)?;

            let cleanup_resources = resources.clone();
            let cleanup_reg = reg.clone();
            let init_guard = ScopeGuard::new(move || {
                stop_hba_controller(&cleanup_reg);
                cleanup_resources.variant().shutdown(&cleanup_reg);
            });

            let transfer_config = UfsTransferConfig::new(&reg)?;
            let dma = UfsDma::new(
                resources.device(),
                reg.clone(),
                transfer_config.queue_depth(),
            )?;

            reg.clear_all_interrupts();
            reg.disable_interrupts();

            let irq = UfsIrq::new()?;
            let uic = UfsUic::new(reg.clone())?;
            let interrupt_policy = match transfer_config {
                UfsTransferConfig::Sdb { .. } => UfsInterruptPolicy::EagerAck,
                UfsTransferConfig::Mcq(_) => UfsInterruptPolicy::ThreadedAck,
            };

            if reg.ctrl_enabled() {
                stop_hba_controller(&reg);
            }

            enable_hba_controller(&resources, &reg)?;

            /* ufshcd_link_startup() */
            irq.request_uic_irq(uic_irq, reg.clone(), uic.clone(), interrupt_policy)?;
            let mut retries = LINK_STARTUP_RETRIES;
            loop {
                match start_link(&resources, &reg, &uic)? {
                    true => break,
                    false if retries > 0 => {
                        pr_warn!(
                            "[RUFS] ufs_host: retry degraded link, retries_left={}\n",
                            retries,
                        );
                        retries -= 1;
                        stop_hba_controller(&reg);
                        enable_hba_controller(&resources, &reg)?;
                    }
                    false => return Err(EIO),
                }
            }
            dma.make_hba_operational()?;
            let task_management = TaskManagement::new(reg.clone(), dma.clone())?;

            let ufs_queue = UfsQueue::new(
                transfer_config,
                reg.clone(),
                dma.clone(),
            )?;
            let shared = UfsHostShared::new(
                ufs_queue.clone(),
                resources.clone(),
                reg.clone(),
                dma.clone(),
                uic.clone(),
                task_management.clone(),
            )?;
            let dev = UfsDev::new(ufs_queue.clone(), shared.clone())?;
            let host = try_pin_init!(Self {
                resources,
                reg,
                dma,
                irq,
                uic,
                task_management,
                queue: ufs_queue,
                shared,
                dev,
                luns <- new_mutex!(KVec::new()),
                max_hw_queues: 1,
                max_prdt_entries: 256,
            })
            .pin_chain(move |host| {
                init_guard.dismiss();

                /* ufshcd_verify_dev_init */
                host.irq.request_queue_irq(
                    queue_irq,
                    host.reg.clone(),
                    host.queue.clone(),
                    host.shared.clone(),
                    host.task_management.clone(),
                    interrupt_policy,
                )?;
                // Transfer and backend completion interrupts must not become
                // visible before their handler owns a live queue.
                host.queue.enable_interrupts();
                host.dev.verify_dev_init()?;
                host.dev.complete_dev_init()?;
                host.dev.device_params_init()?;
                if let Err(e) = host.configure_power_mode() {
                    pr_warn!("rufs: power mode configuration failed: {}\n", e.to_errno(),);
                }
                host.alloc_luns()?;
                host.shared.promote_to_operational()?;
                Ok(())
            });

            Ok(host)
        })
    }

    fn configure_power_mode(&self) -> Result<()> {
        let variant = self.resources.variant();
        let mode = variant.constrain_power_mode(self.uic.max_power_mode()?)?;

        variant.power_mode_notify(&self.reg, &self.uic, mode, NotifyPhase::Pre)?;
        self.uic.change_power_mode(mode)?;
        variant.power_mode_notify(&self.reg, &self.uic, mode, NotifyPhase::Post)
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
                self.shared.clone(),
                lun_id,
                geometry,
                queue_depth,
            )?;
            lu.init_disk()?;

            pr_info!(
                "rufs: LU {} capacity={} block_size={} queue_depth={}\n",
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

    pub(crate) fn request_mcq_queue_irqs<F>(&self, request: F) -> Result<()>
    where
        F: FnMut() -> Result<irq::IrqRequest<'a>>,
    {
        self.irq
            .request_mcq_queue_irqs(
                self.queue.clone(),
                self.shared.clone(),
                self.queue.interrupt_queues(),
                request,
            )
    }

    pub(crate) fn shutdown(&self) {
        if !self.shared.begin_draining() {
            return;
        }

        // Removing a disk may submit final filesystem writeback. Keep the
        // request queues and completion path operational until every disk has
        // been marked dead and its final I/O has completed.
        self.remove_luns();

        self.shared.begin_shutdown();
        self.shared.flush_work();
        let _control = self.shared.lock_control();
        if let Err(e) = read_poll_timeout(
            || Ok(self.queue.busy_requests()),
            |active| *active == 0,
            Delta::from_millis(SHUTDOWN_DRAIN_INTERVAL_MS),
            Delta::from_secs(SHUTDOWN_DRAIN_TIMEOUT_SECS),
        ) {
            pr_err!(
                "[RUFS] ufs_host: timed out draining {} commands errno={}\n",
                self.queue.busy_requests(),
                e.to_errno(),
            );
        }
        self.queue.wait_completed_requests();

        self.reg.disable_interrupts();
        self.irq.shutdown();
        stop_hba_controller(&self.reg);
        self.resources.variant().shutdown(&self.reg);
        self.shared.finish_shutdown();
    }

    fn remove_luns(&self) {
        let mut luns = self.luns.lock();
        for lu in luns.iter() {
            lu.remove_disk();
        }
        luns.clear();
    }

}

#[pinned_drop]
impl PinnedDrop for UfsHost<'_> {
    fn drop(self: Pin<&mut Self>) {
        self.shutdown();
    }
}
