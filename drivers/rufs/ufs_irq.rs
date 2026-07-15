// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use crate::ufs_queue::*;
use crate::ufs_reg::*;
use crate::ufs_uic::*;
use kernel::device::{Bound, Core, Device};
use kernel::irq::{self, Flags, IrqReturn, ThreadedIrqReturn};
use kernel::sync::atomic::{Acquire, Atomic, Release};
use kernel::sync::{Arc, Mutex};
use kernel::{c_str, new_mutex, pci, prelude::*};

#[pin_data]
struct UfsControllerHandler {
    reg: Arc<UfsReg>,
    uic: Arc<UfsUic>,
    queue: Arc<Mutex<Option<Arc<UfsQueue>>>>,
    interrupt_status: Atomic<u32>,
}

impl irq::ThreadedHandler for UfsControllerHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let interrupt_status =
            self.reg.read_uic_interrupts() | self.reg.read_transfer_interrupts();
        if interrupt_status == 0 {
            return ThreadedIrqReturn::None;
        }

        self.interrupt_status.store(interrupt_status, Release);
        if is_error_interrupt(interrupt_status) {
            pr_warn!(
                "[RUFS] ufs_irq: controller error interrupt status=0x{:x}\n",
                interrupt_status
            );
        }

        ThreadedIrqReturn::WakeThread
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        let interrupt_status = self.interrupt_status.load(Acquire);
        let uic_status = UfsReg::uic_interrupts(interrupt_status);
        let transfer_status = UfsReg::transfer_interrupts(interrupt_status);
        let uic_errors = if is_uic_error_interrupt(transfer_status) {
            Some(self.reg.read_uic_errors())
        } else {
            None
        };

        self.reg.confirm_uic_interrupts(uic_status);
        self.reg.confirm_transfer_interrupts(transfer_status);

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
    ) -> Result<()> {
        let handler = try_pin_init!(UfsControllerHandler {
            reg,
            uic,
            queue: self.queue.clone(),
            interrupt_status: Atomic::new(0),
        });

        let flags = Flags::SHARED | Flags::ONESHOT;
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
