// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::{pci, prelude::*, c_str, new_mutex, new_spinlock};
use kernel::device::{Device, Core, Bound};
use kernel::sync::{Arc, Mutex, SpinLock};
use kernel::irq::{self, Flags, IrqReturn, ThreadedIrqReturn};
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

impl irq::ThreadedHandler for UfsQueueHandler {
    fn handle(&self, _dev: &Device<Bound>) -> ThreadedIrqReturn {
        let interrupt_status = self.reg.read_transfer_interrupts();
        if interrupt_status == 0 {
            return ThreadedIrqReturn::None;
        }

        self.reg.confirm_transfer_interrupts(interrupt_status);

        ThreadedIrqReturn::WakeThread
    }

    fn handle_threaded(&self, _dev: &Device<Bound>) -> IrqReturn {
        self.queue.complete();
        IrqReturn::Handled
    }
}

#[pin_data]
pub(crate) struct UfsIrq {
    #[pin]
    uic: Mutex<Option<Arc<irq::Registration<UfsUicHandler>>>>,
    #[pin]
    queue: SpinLock<KVec<Arc<irq::ThreadedRegistration<UfsQueueHandler>>>>,
}

impl UfsIrq {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                uic <- new_mutex!(None),
                queue <- new_spinlock!(KVec::new()),
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

    pub(crate) fn request_queue_irqs(
        &self,
        pdev: &pci::Device<Core>,
        first_vector: pci::IrqVector<'_>,
        nr_vectors: usize,
        reg: Arc<UfsReg>,
        queue: Arc<UfsQueue>,
    ) -> Result<()> {
        if nr_vectors == 0 {
            return Err(EINVAL);
        }

        let mut irqs = KVec::new();
        let nr_vectors = u32::try_from(nr_vectors).map_err(|_| EOVERFLOW)?;
        let last_vector = first_vector
            .index()
            .checked_add(nr_vectors)
            .ok_or(EOVERFLOW)?;
        for index in first_vector.index()..last_vector {
            // SAFETY: `UfsHost` passes `nr_vectors` from the range returned by
            // `alloc_irq_vectors()`, so every index in this loop is allocated
            // for `pdev`.
            let vector = unsafe { first_vector.from_allocated_index(index) };
            Self::request_one_queue_irq(&mut irqs, pdev, vector, reg.clone(), queue.clone())?;
        }

        *self.queue.lock() = irqs;
        Ok(())
    }

    fn request_one_queue_irq(
        irqs: &mut KVec<Arc<irq::ThreadedRegistration<UfsQueueHandler>>>,
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

        let irq = pdev.request_threaded_irq(
            vector,
            Flags::SHARED,
            c_str!("ufshcd-queue"),
            handler,
        );

        let irq = Arc::pin_init(irq, GFP_KERNEL)?;
        irqs.push(irq, GFP_KERNEL)?;

        Ok(())
    }

    pub(crate) fn wake_queue_thread(&self) {
        let Some(irq) = self.queue.lock().first().map(|irq| irq.clone()) else {
            pr_err!("rufs: queue IRQ thread is not registered\n");
            return;
        };

        if irq.wake_thread().is_err() {
            pr_err!("rufs: failed to wake queue IRQ thread\n");
        }
    }
}
