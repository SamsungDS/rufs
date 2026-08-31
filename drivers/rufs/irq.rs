// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use crate::queue::*;
use crate::reg::*;
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

        record_pending_interrupts(&self.pending_interrupts, interrupt_status);

        if matches!(self.policy, UfsInterruptPolicy::EagerAck) {
            // SDB completion identity remains available in the doorbell and
            // outstanding bitmap after the global status is cleared. Allow
            // the next completion to interrupt while this one is finalized.
            self.reg.confirm_transfer_interrupts(interrupt_status);
        }

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
                // A global MCQ event can remain asserted until the thread
                // drains the CQ and clears its per-queue status. Keep the
                // oneshot IRQ masked and acknowledge the global status here.
                self.reg.confirm_transfer_interrupts(interrupt_status);
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
            self.queue.complete();
        }

        IrqReturn::Handled
    }
}

#[pin_data]
pub(crate) struct UfsIrq {
    #[pin]
    uic: Mutex<Option<Arc<irq::ThreadedRegistration<UfsUicHandler>>>>,
    #[pin]
    queue: Mutex<Option<Arc<irq::ThreadedRegistration<UfsQueueHandler>>>>,
}

impl UfsIrq {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                uic <- new_mutex!(None),
                queue <- new_mutex!(None),
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

    pub(crate) fn shutdown(&self) {
        // Drop outside the registration locks because `free_irq()` waits for
        // the corresponding primary and threaded handlers to finish.
        let queue = self.queue.lock().take();
        let uic = self.uic.lock().take();
        drop(queue);
        drop(uic);
    }
}
