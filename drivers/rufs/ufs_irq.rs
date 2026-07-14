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
struct UfsUicHandler {
    reg: Arc<UfsReg>,
    uic: Arc<UfsUic>,
    interrupt_status: Atomic<u32>,
}

impl irq::ThreadedHandler for UfsUicHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let interrupt_status = self.reg.read_uic_interrupts();
        if interrupt_status == 0 {
            return ThreadedIrqReturn::None;
        }

        self.interrupt_status.store(interrupt_status, Release);

        ThreadedIrqReturn::WakeThread
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        let interrupt_status = self.interrupt_status.load(Acquire);
        self.reg.confirm_uic_interrupts(interrupt_status);
        if self.uic.handle_uic_completion(interrupt_status) {
            self.uic.complete_uic_cmd();
        }

        IrqReturn::Handled
    }
}

#[pin_data]
pub(crate) struct UfsQueueHandler {
    reg: Arc<UfsReg>,
    queue: Arc<UfsQueue>,
    interrupt_status: Atomic<u32>,
}

impl irq::ThreadedHandler for UfsQueueHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let interrupt_status = self.reg.read_transfer_interrupts();
        if interrupt_status == 0 {
            return ThreadedIrqReturn::None;
        }

        self.interrupt_status.store(interrupt_status, Release);
        if is_error_interrupt(interrupt_status) {
            pr_warn!(
                "[RUFS] ufs_irq: transfer/error interrupt status=0x{:x}\n",
                interrupt_status
            );
        }

        ThreadedIrqReturn::WakeThread
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        let interrupt_status = self.interrupt_status.load(Acquire);
        self.reg.confirm_transfer_interrupts(interrupt_status);
        self.queue.complete();
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

    pub(crate) fn request_uic_irq(
        &self,
        pdev: &pci::Device<Core<'_>>,
        vector: pci::IrqVector<'_>,
        reg: Arc<UfsReg>,
        uic: Arc<UfsUic>,
    ) -> Result<()> {
        let handler = try_pin_init!(UfsUicHandler {
            reg,
            uic,
            interrupt_status: Atomic::new(0),
        });

        let flags = Flags::SHARED | Flags::ONESHOT;
        let irq = pdev.request_threaded_irq(vector, flags, c_str!("ufshcd-uic"), handler);

        let reg = Arc::pin_init(irq, GFP_KERNEL)?;
        self.uic.lock().replace(reg);

        Ok(())
    }

    pub(crate) fn request_queue_irq(
        &self,
        pdev: &pci::Device<Core<'_>>,
        vector: pci::IrqVector<'_>,
        reg: Arc<UfsReg>,
        queue: Arc<UfsQueue>,
    ) -> Result<()> {
        let handler = try_pin_init!(UfsQueueHandler {
            reg,
            queue,
            interrupt_status: Atomic::new(0),
        });

        let flags = Flags::SHARED | Flags::ONESHOT;
        let irq = pdev.request_threaded_irq(vector, flags, c_str!("ufshcd-queue"), handler);

        let irq = Arc::pin_init(irq, GFP_KERNEL)?;
        self.queue.lock().replace(irq);

        Ok(())
    }
}
