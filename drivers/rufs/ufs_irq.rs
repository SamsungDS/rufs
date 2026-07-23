// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use crate::ufs_queue::*;
use crate::ufs_reg::*;
use crate::ufs_uic::*;
use kernel::device::{Bound, Core, Device};
use kernel::irq::{self, Flags, IrqReturn, ThreadedIrqReturn};
use kernel::sync::atomic::{Acquire, Atomic, Relaxed, Release};
use kernel::sync::{Arc, Mutex};
use kernel::{c_str, new_mutex, pci, prelude::*};

#[derive(Clone, Copy)]
pub(crate) enum UfsInterruptPolicy {
    EagerAck,
    ThreadedAck,
}

#[pin_data]
struct UfsControllerHandler {
    reg: Arc<UfsReg>,
    uic: Arc<UfsUic>,
    queue: Arc<Mutex<Option<Arc<UfsQueue>>>>,
    policy: UfsInterruptPolicy,
    pending_interrupts: Atomic<u32>,
}

impl UfsControllerHandler {
    fn acknowledge(&self, interrupt_status: u32) {
        self.reg
            .confirm_uic_interrupts(UfsReg::uic_interrupts(interrupt_status));
        self.reg
            .confirm_transfer_interrupts(UfsReg::transfer_interrupts(interrupt_status));
    }
}

impl irq::ThreadedHandler for UfsControllerHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let interrupt_status = self.reg.read_uic_interrupts() | self.reg.read_transfer_interrupts();
        if interrupt_status == 0 {
            return ThreadedIrqReturn::None;
        }

        let mut pending = self.pending_interrupts.load(Relaxed);
        loop {
            match self
                .pending_interrupts
                .cmpxchg(pending, pending | interrupt_status, Release)
            {
                Ok(_) => break,
                Err(current) => pending = current,
            }
        }

        if matches!(self.policy, UfsInterruptPolicy::EagerAck) {
            // SDB completion identity remains available in the doorbell and
            // outstanding bitmap after the global status is cleared. Allow
            // the next completion to interrupt while this one is finalized.
            self.acknowledge(interrupt_status);
        }

        if is_error_interrupt(interrupt_status) {
            pr_warn!(
                "[RUFS] ufs_irq: controller error interrupt status=0x{:x}\n",
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

            let uic_status = UfsReg::uic_interrupts(interrupt_status);
            let transfer_status = UfsReg::transfer_interrupts(interrupt_status);
            let uic_errors = if is_uic_error_interrupt(transfer_status) {
                Some(self.reg.read_uic_errors())
            } else {
                None
            };

            if matches!(self.policy, UfsInterruptPolicy::ThreadedAck) {
                // A global MCQ event can remain asserted until the thread
                // drains the CQ and clears its per-queue status. Keep the
                // oneshot IRQ masked and acknowledge the global status here.
                self.acknowledge(interrupt_status);
            }

            if self.uic.handle_uic_completion(uic_status) {
                self.uic.complete_uic_cmd();
            }
            let queue = self.queue.lock().clone();
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
                    if let Some(queue) = &queue {
                        queue.require_uic_recovery(errors);
                    }
                }
            }
            if let Some(queue) = queue {
                if is_transfer_recovery_interrupt(transfer_status) {
                    queue.require_recovery("transfer error interrupt", 0);
                }
                if transfer_status != 0 {
                    queue.complete();
                }
            }
        }

        IrqReturn::Handled
    }
}

#[pin_data]
pub(crate) struct UfsIrq {
    #[pin]
    registration: Mutex<Option<Arc<irq::ThreadedRegistration<UfsControllerHandler>>>>,
    #[pin]
    queue: Arc<Mutex<Option<Arc<UfsQueue>>>>,
}

impl UfsIrq {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                registration <- new_mutex!(None),
                queue: Arc::pin_init(new_mutex!(None), GFP_KERNEL)?,
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn request_controller_irq(
        &self,
        pdev: &pci::Device<Core<'_>>,
        vector: pci::IrqVector<'_>,
        reg: Arc<UfsReg>,
        uic: Arc<UfsUic>,
        policy: UfsInterruptPolicy,
    ) -> Result<()> {
        let handler = try_pin_init!(UfsControllerHandler {
            reg,
            uic,
            queue: self.queue.clone(),
            policy,
            pending_interrupts: Atomic::new(0),
        });

        let flags = match policy {
            UfsInterruptPolicy::EagerAck => Flags::SHARED,
            UfsInterruptPolicy::ThreadedAck => Flags::SHARED | Flags::ONESHOT,
        };
        let irq = pdev.request_threaded_irq(vector, flags, c_str!("rufs-controller"), handler);

        let reg = Arc::pin_init(irq, GFP_KERNEL)?;
        self.registration.lock().replace(reg);

        Ok(())
    }

    pub(crate) fn attach_queue(&self, queue: Arc<UfsQueue>) -> Result<()> {
        let mut slot = self.queue.lock();
        if slot.is_some() {
            return Err(EBUSY);
        }
        *slot = Some(queue);
        Ok(())
    }
}
