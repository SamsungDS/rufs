// SPDX-License-Identifier: GPL-2.0

//! UfsHost: Top-level host controller manager.

#![allow(dead_code)]

use kernel::{
    block::mq::TagSet,
    device::Core,
    pci,
    prelude::*,
    new_mutex,
    new_spinlock,
};
use kernel::time::{Delta, delay::*};
use kernel::sync::{Arc, Mutex, SpinLock};

use crate::ufs_reg::*;
use crate::ufs_dma::*;
use crate::ufs_irq::*;
use crate::ufs_uic::*;
use crate::ufs_queue::*;
use crate::ufs_dev::*;
use crate::ufs_lu::*;

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

    #[pin]
    luns: Mutex<KVec<Arc<UfsLu>>>,

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

        let requested_irq_vectors = match ufs_mcq_interrupt_queue_count(&reg)
            .and_then(|queues| u32::try_from(queues).map_err(|_| EOVERFLOW))
        {
            Ok(queues) => queues,
            Err(e) => {
                pr_warn!(
                    "[RUFS] ufs_host: failed to plan MCQ IRQ vectors errno={}, fallback to single shared IRQ\n",
                    e.to_errno(),
                );
                1
            },
        };
        let msi_irq_types = pci::IrqTypes::default()
            .with(pci::IrqType::MsiX)
            .with(pci::IrqType::Msi);
        let irq_vectors = if requested_irq_vectors > 1 {
            match pdev.alloc_irq_vectors(requested_irq_vectors, requested_irq_vectors, msi_irq_types) {
                Ok(irq_vectors) => irq_vectors,
                Err(e) => {
                    pr_warn!(
                        "[RUFS] ufs_host: failed to allocate {} MSI/MSI-X IRQ vectors errno={}, fallback to single shared IRQ\n",
                        requested_irq_vectors,
                        e.to_errno(),
                    );
                    pdev.alloc_irq_vectors(1, 1, pci::IrqTypes::all())?
                },
            }
        } else {
            pdev.alloc_irq_vectors(1, 1, pci::IrqTypes::all())?
        };
        let first_irq_vector = *irq_vectors.start();
        let allocated_irq_vectors = irq_vectors
            .end()
            .index()
            .checked_sub(first_irq_vector.index())
            .ok_or(EINVAL)?
            + 1;
        pr_info!(
            "[RUFS] ufs_host: IRQ vectors requested={} allocated={}\n",
            requested_irq_vectors,
            allocated_irq_vectors,
        );

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
                luns <- new_mutex!(KVec::new()),
                state <- new_spinlock!(HostState::Reset),
                max_hw_queues: 1,
                max_prdt_entries: 256,
            }), GFP_KERNEL,
        )?;

        host.reg.ctrl_enable();
        fsleep(Delta::from_micros(HBA_ENABLE_DELAY_US));
        host.reg.wait_for_ctrl_enable(1000, 50)?;

        /* ufshcd_link_startup() */
        host.irq.request_uic_irq(
            pdev,
            first_irq_vector,
            host.reg.clone(),
            host.uic.clone(),
        )?;
        host.uic.link_startup()?;
        host.dma.make_hba_operational()?;

        /* ufshcd_verify_dev_init */
        host.irq.request_queue_irqs(
            pdev,
            first_irq_vector,
            allocated_irq_vectors as usize,
            host.reg.clone(),
            host.queue.clone(),
        )?;
        if allocated_irq_vectors < requested_irq_vectors {
            pr_info!(
                "[RUFS] ufs_host: allocated fewer IRQ vectors than requested {}/{}\n",
                allocated_irq_vectors,
                requested_irq_vectors,
            );
        }
        if host.reg.mcq_supported() {
            match host.queue.enable_mcq_backend(host.reg.clone(), host.dma.clone()) {
                Ok(()) => {},
                Err(e) => {
                    pr_warn!(
                        "[RUFS] ufs_host: MCQ setup failed errno={}, keep SDB backend\n",
                        e.to_errno(),
                    );
                },
            }
        }
        host.dev.verify_dev_init()?;
        host.dev.complete_dev_init()?;
        host.dev.device_params_init()?;
        host.alloc_luns()?;
        host.dev.alloc_tmf_queue(host.reg.nutmrs())?;

        Ok(host)
    }

    fn alloc_luns(&self) -> Result<()> {
        let num_lu = self.dev.num_lu();
        let queue_map = self.queue.queue_map()?;
        let nr_hw_queues = queue_map.nr_hw_queues();
        let total_block_tags = self.queue.queue_depth().checked_sub(1).ok_or(EINVAL)?;
        if total_block_tags == 0 {
            return Err(EINVAL);
        }
        let hw_queue_depth = total_block_tags / nr_hw_queues;
        if hw_queue_depth == 0 {
            return Err(EINVAL);
        }
        let num_maps = queue_map.num_maps();
        let tagset = Arc::pin_init(
            TagSet::<UfsLuBlockOps>::new(
                u32::try_from(nr_hw_queues).map_err(|_| EOVERFLOW)?,
                KBox::new(queue_map, GFP_KERNEL)?,
                u32::try_from(hw_queue_depth).map_err(|_| EOVERFLOW)?,
                num_maps,
                kernel::alloc::NumaNode::NO_NODE,
                kernel::block::mq::tag_set::Flags::default(),
            ),
            GFP_KERNEL,
        )?;
        let mut luns = self.luns.lock();

        for lun in 0..num_lu {
            let desc = self.dev.read_unit_desc(lun as u8)?;

            if !desc.enabled() {
                continue;
            }

            let geometry = UfsLuGeometry::from_logical_block_shift(
                desc.logical_block_shift(),
                desc.logical_block_count(),
            )?;
            let lu = UfsLu::new(self.queue.clone(), lun as u8, geometry, hw_queue_depth)?;
            lu.init_disk(tagset.clone())?;

            pr_info!(
                "[RUFS] ufs_host: allocated LU {} capacity={} logical_block_size={}",
                lun,
                geometry.capacity_blocks(),
                geometry.logical_block_size(),
            );

            luns.push(lu, GFP_KERNEL)?;
        }

        Ok(())
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
