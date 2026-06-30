// SPDX-License-Identifier: GPL-2.0

//! UfsHost: Top-level host controller manager.

#![allow(dead_code)]

use kernel::sync::{Arc, Mutex, SpinLock};
use kernel::time::{delay::*, Delta};
use kernel::{block::mq::TagSet, device::Core, new_mutex, new_spinlock, pci, prelude::*};
use pin_init::pin_init_scope;

use crate::ufs_dev::*;
use crate::ufs_dma::*;
use crate::ufs_irq::*;
use crate::ufs_lu::*;
use crate::ufs_queue::*;
use crate::ufs_reg::*;
use crate::ufs_uic::*;

const HBA_ENABLE_DELAY_US: i64 = 1000;

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
    pub(crate) fn new<'a>(pdev: &'a pci::Device<Core<'a>>) -> impl PinInit<Self, Error> + 'a {
        pin_init_scope(move || {
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
                }
            };
            let msi_irq_types = pci::IrqTypes::default()
                .with(pci::IrqType::MsiX)
                .with(pci::IrqType::Msi);
            let irq_vectors = if requested_irq_vectors > 1 {
                match pdev.alloc_irq_vectors(
                    requested_irq_vectors,
                    requested_irq_vectors,
                    msi_irq_types,
                ) {
                    Ok(irq_vectors) => irq_vectors,
                    Err(e) => {
                        pr_warn!(
                        "[RUFS] ufs_host: failed to allocate {} MSI/MSI-X IRQ vectors errno={}, fallback to single shared IRQ\n",
                        requested_irq_vectors,
                        e.to_errno(),
                    );
                        pdev.alloc_irq_vectors(1, 1, pci::IrqTypes::all())?
                    }
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
            let queue = UfsQueue::new(reg.clone(), irq.clone(), dma.clone())?;
            let dev = UfsDev::new(queue.clone())?;
            let host = try_pin_init!(Self {
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
            })
            .pin_chain(move |host| {
                if host.reg.ctrl_enabled() {
                    pr_info!("[RUFS] ufs_host: controller is active, stop before enable\n");
                    host.hba_stop();
                }

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
                    pr_info!("MCQ supported\n");
                    match host
                        .queue
                        .enable_mcq_backend(host.reg.clone(), host.dma.clone())
                    {
                        Ok(()) => {}
                        Err(e) => {
                            pr_warn!(
                                "[RUFS] ufs_host: MCQ setup failed errno={}, keep SDB backend\n",
                                e.to_errno(),
                            );
                        }
                    }
                } else {
                    pr_info!("MCQ not supported, using SDB mode!\n");
                }

                host.dev.alloc_dev_request()?;
                host.dev.verify_dev_init()?;
                host.dev.complete_dev_init()?;
                host.dev.device_params_init()?;
                host.uic.configure_max_power_mode()?;
                host.alloc_luns()?;
                host.dev.alloc_tmf_queue(host.reg.nutmrs())?;
                Ok(())
            });

            Ok(host)
        })
    }

    fn alloc_luns(&self) -> Result<()> {
        let num_lu = self.dev.num_lu();
        let queue_map = self.queue.queue_map()?;
        let nr_hw_queues = queue_map.nr_hw_queues();
        let total_block_tags = self.queue.queue_depth().checked_sub(1).ok_or(EINVAL)?;
        if total_block_tags == 0 || nr_hw_queues == 0 {
            return Err(EINVAL);
        }
        let hw_queue_depth = total_block_tags.checked_div(nr_hw_queues).ok_or(EINVAL)?;
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
            let lu_queue_depth = match desc.lu_queue_depth() {
                0 => total_block_tags,
                depth => core::cmp::min(depth, total_block_tags),
            };
            let lu = UfsLu::new(
                self.queue.clone(),
                lun_id,
                geometry,
                hw_queue_depth,
                lu_queue_depth,
            )?;
            lu.init_disk(tagset.clone())?;

            pr_info!(
                "[RUFS] ufs_host: allocated LU {} capacity={} logical_block_size={} queue_depth={}",
                lun,
                geometry.capacity_blocks(),
                geometry.logical_block_size(),
                lu_queue_depth,
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
        self.reg.disable_interrupts();
        self.reg.clear_all_interrupts();
        self.reg.disable_run_stop();
        self.reg.ctrl_disable();
        if let Err(e) = self.reg.wait_for_ctrl_disable(10, 1) {
            pr_err!(
                "[RUFS] ufs_host: controller disable failed errno={}\n",
                e.to_errno()
            );
        }
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
