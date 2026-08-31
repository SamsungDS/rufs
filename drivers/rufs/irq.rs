// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use crate::queue::*;
use crate::reg::*;
use crate::transport::McqInterruptQueue;
use crate::uic::*;
use kernel::device::{Bound, Device};
use kernel::irq::{self, Flags, IrqRequest, IrqReturn, ThreadedIrqReturn};
use kernel::sync::atomic::{Acquire, Atomic, Relaxed, Release};
use kernel::sync::{Arc, Mutex};
use kernel::{c_str, new_mutex, prelude::*};

#[derive(Clone, Copy)]
pub(crate) enum UfsInterruptPolicy {
    EagerAck,
    ThreadedAck,
}

#[pin_data]
struct UfsUicHandler {
    reg: Arc<UfsReg>,
    uic: Arc<UfsUic>,
    policy: UfsInterruptPolicy,
    pending_interrupts: Atomic<u32>,
}

#[pin_data]
struct UfsQueueHandler {
    reg: Arc<UfsReg>,
    queue: Arc<UfsQueue>,
    policy: UfsInterruptPolicy,
    pending_interrupts: Atomic<u32>,
    per_queue_irqs_active: Atomic<bool>,
}

#[pin_data]
struct UfsMcqQueueHandler {
    queue: Arc<UfsQueue>,
    interrupt_queue: McqInterruptQueue,
}

fn record_pending_interrupts(pending_interrupts: &Atomic<u32>, interrupt_status: u32) {
    let mut pending = pending_interrupts.load(Relaxed);
    loop {
        match pending_interrupts.cmpxchg(pending, pending | interrupt_status, Release) {
            Ok(_) => break,
            Err(current) => pending = current,
        }
    }
}

fn registration_flags(policy: UfsInterruptPolicy) -> Flags {
    match policy {
        UfsInterruptPolicy::EagerAck => Flags::SHARED,
        UfsInterruptPolicy::ThreadedAck => Flags::SHARED | Flags::ONESHOT,
    }
}

fn warn_mcq_irq_fallback(stage: &str, error: Error) {
    pr_warn!(
        "[RUFS] ufs_irq: per-CQ IRQ {} failed errno={}, use global completion handler\n",
        stage,
        error.to_errno(),
    );
}

impl irq::ThreadedHandler for UfsUicHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let interrupt_status = self.reg.read_uic_interrupts();
        if interrupt_status == 0 {
            return ThreadedIrqReturn::None;
        }

        record_pending_interrupts(&self.pending_interrupts, interrupt_status);

        if matches!(self.policy, UfsInterruptPolicy::EagerAck) {
            self.reg.confirm_uic_interrupts(interrupt_status);
        }

        ThreadedIrqReturn::WakeThread
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        loop {
            let interrupt_status = self.pending_interrupts.xchg(0, Acquire);
            if interrupt_status == 0 {
                break;
            }

            if matches!(self.policy, UfsInterruptPolicy::ThreadedAck) {
                self.reg.confirm_uic_interrupts(interrupt_status);
            }

            if self.uic.handle_uic_completion(interrupt_status) {
                self.uic.complete_uic_cmd();
            }
        }

        IrqReturn::Handled
    }
}

impl irq::ThreadedHandler for UfsQueueHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let interrupt_status = self.reg.read_transfer_interrupts();
        if interrupt_status == 0 {
            return ThreadedIrqReturn::None;
        }

        match self.policy {
            UfsInterruptPolicy::EagerAck => {
                // SDB completion identity remains available in the doorbell
                // and outstanding bitmap after global status is cleared.
                self.reg.confirm_transfer_interrupts(interrupt_status);
            }
            UfsInterruptPolicy::ThreadedAck => {
                // Latch and clear the global MCQ event before any threaded
                // action runs. A later CQ event can then assert a new IRQ.
                self.reg.confirm_mcq_cq_events(interrupt_status);

                // Per-CQ handlers own normal MCQ completion processing after
                // activation. Keep the global action handled for controllers
                // that also report CQES for poll queues, but do not wake its
                // thread unless a deferred transfer or error event needs it.
                if self.per_queue_irqs_active.load(Acquire)
                    && !has_deferred_transfer_interrupts(interrupt_status)
                {
                    return ThreadedIrqReturn::Handled;
                }
            }
        }

        record_pending_interrupts(&self.pending_interrupts, interrupt_status);

        if is_error_interrupt(interrupt_status) {
            pr_warn!(
                "[RUFS] ufs_irq: transfer/error interrupt status=0x{:x}\n",
                interrupt_status
            );
        }

        ThreadedIrqReturn::WakeThread
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        loop {
            let interrupt_status = self.pending_interrupts.xchg(0, Acquire);
            if interrupt_status == 0 {
                break;
            }

            let uic_errors = if is_uic_error_interrupt(interrupt_status) {
                Some(self.reg.read_uic_errors())
            } else {
                None
            };

            if matches!(self.policy, UfsInterruptPolicy::ThreadedAck) {
                self.reg
                    .confirm_deferred_transfer_interrupts(interrupt_status);
            }

            if let Some(errors) = uic_errors {
                pr_warn!(
                    "[RUFS] ufs_irq: UIC error phy=0x{:08x} dl=0x{:08x} nl=0x{:08x} tl=0x{:08x} dme=0x{:08x}\n",
                    errors.phy,
                    errors.data_link,
                    errors.network,
                    errors.transport,
                    errors.dme,
                );
                if errors.requires_recovery() {
                    self.queue.require_uic_recovery(errors);
                }
            }
            if is_transfer_recovery_interrupt(interrupt_status) {
                self.queue.require_recovery("transfer error interrupt", 0);
            }
            if !self.per_queue_irqs_active.load(Acquire) {
                self.queue.complete();
            }
        }

        IrqReturn::Handled
    }
}

impl irq::ThreadedHandler for UfsMcqQueueHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        match self.interrupt_queue.acknowledge_completion() {
            Ok(true) | Err(_) => ThreadedIrqReturn::WakeThread,
            Ok(false) => ThreadedIrqReturn::None,
        }
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        self.queue.complete_queue(&self.interrupt_queue);
        IrqReturn::Handled
    }
}

#[pin_data]
pub(crate) struct UfsIrq {
    #[pin]
    uic: Mutex<Option<Arc<irq::ThreadedRegistration<UfsUicHandler>>>>,
    #[pin]
    queue: Mutex<Option<Arc<irq::ThreadedRegistration<UfsQueueHandler>>>>,
    #[pin]
    mcq_queues: Mutex<KVec<Arc<irq::ThreadedRegistration<UfsMcqQueueHandler>>>>,
}

impl UfsIrq {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                uic <- new_mutex!(None),
                queue <- new_mutex!(None),
                mcq_queues <- new_mutex!(KVec::new()),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn request_uic_irq<'a>(
        &self,
        request: IrqRequest<'a>,
        reg: Arc<UfsReg>,
        uic: Arc<UfsUic>,
        policy: UfsInterruptPolicy,
    ) -> Result<()> {
        let handler = try_pin_init!(UfsUicHandler {
            reg,
            uic,
            policy,
            pending_interrupts: Atomic::new(0),
        });

        let irq = irq::ThreadedRegistration::new(
            request,
            registration_flags(policy),
            c_str!("rufs-uic"),
            handler,
        );

        let reg = Arc::pin_init(irq, GFP_KERNEL)?;
        self.uic.lock().replace(reg);

        Ok(())
    }

    pub(crate) fn request_queue_irq<'a>(
        &self,
        request: IrqRequest<'a>,
        reg: Arc<UfsReg>,
        queue: Arc<UfsQueue>,
        policy: UfsInterruptPolicy,
    ) -> Result<()> {
        let handler = try_pin_init!(UfsQueueHandler {
            reg,
            queue,
            policy,
            pending_interrupts: Atomic::new(0),
            per_queue_irqs_active: Atomic::new(false),
        });

        let irq = irq::ThreadedRegistration::new(
            request,
            registration_flags(policy),
            c_str!("rufs-queue"),
            handler,
        );

        let reg = Arc::pin_init(irq, GFP_KERNEL)?;
        self.queue.lock().replace(reg);

        Ok(())
    }

    pub(crate) fn request_mcq_queue_irqs<'a, F>(
        &self,
        queue: Arc<UfsQueue>,
        interrupt_queues: &[McqInterruptQueue],
        mut request: F,
    ) -> Result<()>
    where
        F: FnMut() -> Result<IrqRequest<'a>>,
    {
        if interrupt_queues.is_empty() {
            return Ok(());
        }

        let queue_irq = self.queue.lock().as_ref().cloned().ok_or(ENODEV)?;
        if !self.mcq_queues.lock().is_empty() {
            return Err(EBUSY);
        }
        let mut registrations = KVec::new();

        for interrupt_queue in interrupt_queues {
            let handler_queue = queue.clone();
            let handler = try_pin_init!(UfsMcqQueueHandler {
                queue: handler_queue,
                interrupt_queue: interrupt_queue.clone(),
            });
            let request = match request() {
                Ok(request) => request,
                Err(e) => {
                    warn_mcq_irq_fallback("setup", e);
                    return Ok(());
                }
            };
            let irq = irq::ThreadedRegistration::new(
                request,
                Flags::SHARED | Flags::ONESHOT,
                c_str!("rufs-mcq-cq"),
                handler,
            );
            let registration = match Arc::pin_init(irq, GFP_KERNEL) {
                Ok(registration) => registration,
                Err(e) => {
                    warn_mcq_irq_fallback("setup", e);
                    return Ok(());
                }
            };
            if let Err(e) = registrations.push(registration, GFP_KERNEL) {
                warn_mcq_irq_fallback("setup", e.into());
                return Ok(());
            }
        }

        {
            let mut mcq_queues = self.mcq_queues.lock();
            if !mcq_queues.is_empty() {
                return Err(EBUSY);
            }
            *mcq_queues = registrations;
        }
        queue_irq.handler().per_queue_irqs_active.store(true, Release);

        // Close the transition race with completions that became pending
        // before their per-CQ action was visible on the shared IRQ.
        let wake_result = {
            let mcq_queues = self.mcq_queues.lock();
            let mut result = Ok(());
            for registration in mcq_queues.iter() {
                if let Err(e) = registration.wake_thread() {
                    result = Err(e);
                    break;
                }
            }
            result
        };
        if let Err(e) = wake_result {
            queue_irq.handler().per_queue_irqs_active.store(false, Release);
            let registrations =
                core::mem::replace(&mut *self.mcq_queues.lock(), KVec::new());
            drop(registrations);
            warn_mcq_irq_fallback("activation", e);
            return Ok(());
        }

        pr_info!(
            "[RUFS] ufs_irq: registered {} per-CQ handlers on shared IRQ\n",
            interrupt_queues.len(),
        );
        Ok(())
    }

    pub(crate) fn shutdown(&self) {
        // Drop outside the registration locks because `free_irq()` waits for
        // the corresponding primary and threaded handlers to finish.
        let mcq_queues = core::mem::replace(&mut *self.mcq_queues.lock(), KVec::new());
        let queue = self.queue.lock().take();
        let uic = self.uic.lock().take();
        drop(mcq_queues);
        drop(queue);
        drop(uic);
    }
}
