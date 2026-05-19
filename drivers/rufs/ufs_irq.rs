// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::{pci, prelude::*, c_str, new_mutex, new_spinlock};
use kernel::device::{Device, Core, Bound};
use kernel::sync::{Arc, Mutex, SpinLock};
use kernel::irq::{self, Flags, IrqReturn};
use crate::ufs_reg::*;
use crate::ufs_uic::*;
use crate::ufs_queue::*;

#[pin_data]
struct UfsUicHandler {
    reg: Arc<UfsReg>,
    uic: Arc<UfsUic>,
    #[pin]
    placeholder: SpinLock<u32>,
}

impl irq::Handler for UfsUicHandler {
    fn handle(&self, _dev: &Device<Bound>) -> IrqReturn {
        let interrupt_status = self.reg.read_uic_interrupts();
        self.reg.confirm_uic_interrupts(interrupt_status);
        self.uic.get_uic_cmd_response(interrupt_status);
        self.uic.complete_uic_cmd();

        IrqReturn::Handled
    }
}

#[pin_data]
pub(crate) struct UfsQueueHandler {
    reg: Arc<UfsReg>,
    queue: Arc<UfsQueue>,

    #[pin]
    placeholder: SpinLock<u32>,
}

impl irq::Handler for UfsQueueHandler {
    fn handle(&self, _dev: &Device<Bound>) -> IrqReturn {
        let interrupt_status = self.reg.read_transfer_interrupts();
        self.reg.confirm_transfer_interrupts(interrupt_status);
        self.queue.complete();

        IrqReturn::Handled
    }
}

#[pin_data]
pub(crate) struct UfsIrq {
    #[pin]
    uic: Mutex<Option<Arc<irq::Registration<UfsUicHandler>>>>,
    #[pin]
    queue: Mutex<Option<Arc<irq::Registration<UfsQueueHandler>>>>,
}

impl UfsIrq {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                uic <- new_mutex!(None),
                queue <- new_mutex!(None),
            }), GFP_KERNEL,
        )
    }

    pub(crate) fn request_uic_irq(
        &self,
        pdev: &pci::Device<Core>,
        vector: pci::IrqVector<'_>,
        reg: Arc<UfsReg>,
        uic: Arc<UfsUic>,
    ) -> Result<()> {
        let handler = try_pin_init!(UfsUicHandler {
            reg,
            uic,
            placeholder <- new_spinlock!(0),
        });

        let irq = pdev.request_irq(
            vector,
            Flags::SHARED,
            c_str!("ufshcd-uic"),
            handler,
        );

        let reg = Arc::pin_init(irq, GFP_KERNEL)?;
        self.uic.lock().replace(reg);

        Ok(())
    }

    pub(crate) fn request_queue_irq(
        &self,
        pdev: &pci::Device<Core>,
        vector: pci::IrqVector<'_>,
        reg: Arc<UfsReg>,
        queue: Arc<UfsQueue>,
    ) -> Result<()> {
        let handler = try_pin_init!(UfsQueueHandler {
            reg,
            queue,
            placeholder <- new_spinlock!(0),
        });

        let irq = pdev.request_irq(
            vector,
            Flags::SHARED,
            c_str!("ufshcd-queue"),
            handler,
        );

        let irq = Arc::pin_init(irq, GFP_KERNEL)?;
        self.queue.lock().replace(irq);

        Ok(())
    }
}
