// SPDX-License-Identifier: GPL-2.0

//! UFSHCI multi-circular queue transport.

use crate::dma::UfsDma;
use crate::hci::descriptor::{CqEntry, SqEntry};
use crate::reg::{McqRegisterLayout, UfsMcqOprRegion, UfsMcqOprSet, UfsReg};
use crate::transport::{
    CompletedRequests, McqConfig, SubmissionOutcome, UfsHwQueue, UfsTransferOps,
};
use kernel::io::{io_project, Io};
use kernel::sync::{barrier, Arc, SpinLock};
use kernel::{
    device::{self, Bound},
    dma, new_spinlock,
    prelude::*,
};

struct UfsMcqQueue {
    descriptor: UfsMcqQueueDescriptor,
    submission: UfsMcqSubmissionQueue,
    completion: UfsMcqCompletionQueue,
}

#[derive(Clone, Copy)]
struct UfsMcqQueueDescriptor {
    id: u32,
    max_entries: u32,
    oprs: UfsMcqOprSet,
}

struct UfsMcqSubmissionQueue {
    sqe: dma::Coherent<[SqEntry]>,
    sq_tail_slot: u32,
}

struct UfsMcqCompletionQueue {
    cqe: dma::Coherent<[CqEntry]>,
    cq_tail_slot: u32,
    cq_head_slot: u32,
}

impl UfsMcqQueue {
    fn new(
        dev: &device::Device<Bound>,
        id: u32,
        max_entries: u32,
        oprs: UfsMcqOprSet,
    ) -> Result<Self> {
        if max_entries == 0 {
            return Err(EINVAL);
        }

        let entries = max_entries as usize;
        Ok(Self {
            descriptor: UfsMcqQueueDescriptor {
                id,
                max_entries,
                oprs,
            },
            submission: UfsMcqSubmissionQueue {
                sqe: dma::Coherent::<SqEntry>::zeroed_slice(dev, entries, GFP_KERNEL)?,
                sq_tail_slot: 0,
            },
            completion: UfsMcqCompletionQueue {
                cqe: dma::Coherent::<CqEntry>::zeroed_slice(dev, entries, GFP_KERNEL)?,
                cq_tail_slot: 0,
                cq_head_slot: 0,
            },
        })
    }

    fn into_parts(
        self,
    ) -> (
        UfsMcqQueueDescriptor,
        UfsMcqSubmissionQueue,
        UfsMcqCompletionQueue,
    ) {
        (self.descriptor, self.submission, self.completion)
    }
}

impl UfsMcqQueueDescriptor {
    fn id(&self) -> u32 {
        self.id
    }

    fn max_entries(&self) -> u32 {
        self.max_entries
    }

    fn oprs(&self) -> &UfsMcqOprSet {
        &self.oprs
    }

    fn acknowledge_completion_events(&self, reg: &UfsReg) -> Result<bool> {
        let status = reg.read_mcq_cqis(self.oprs(), self.id() as usize)?;
        if status != 0 {
            reg.write_mcq_cqis(self.oprs(), self.id() as usize, status)?;
        }

        Ok(status != 0)
    }

    fn offset_to_slot(&self, offset: u32, entry_size: u32) -> Result<u32> {
        if offset % entry_size != 0 {
            return Err(EINVAL);
        }

        let slot = offset / entry_size;
        if slot >= self.max_entries {
            return Err(EINVAL);
        }

        Ok(slot)
    }
}

impl UfsMcqSubmissionQueue {
    fn dma_addr(&self) -> dma::DmaAddress {
        self.sqe.dma_handle()
    }

    fn sq_tail_slot(&self) -> u32 {
        self.sq_tail_slot
    }

    fn set_sq_tail_slot(&mut self, slot: u32) {
        self.sq_tail_slot = slot;
    }

    fn sq_tail_index(&self, descriptor: &UfsMcqQueueDescriptor) -> Result<usize> {
        let index = self.sq_tail_slot as usize;
        if index >= descriptor.max_entries as usize {
            return Err(EINVAL);
        }

        Ok(index)
    }

    fn sq_slot_offset(slot: u32) -> u32 {
        slot * core::mem::size_of::<SqEntry>() as u32
    }

    fn next_sq_tail_slot(&self, descriptor: &UfsMcqQueueDescriptor) -> u32 {
        let next = self.sq_tail_slot + 1;
        if next == descriptor.max_entries {
            0
        } else {
            next
        }
    }

    fn is_full(&self, reg: &UfsReg, descriptor: &UfsMcqQueueDescriptor) -> Result<bool> {
        let head = descriptor.offset_to_slot(
            reg.read_mcq_sq_head(descriptor.oprs(), descriptor.id() as usize)?,
            core::mem::size_of::<SqEntry>() as u32,
        )?;
        Ok(self.next_sq_tail_slot(descriptor) == head)
    }

    fn reset(&mut self) {
        self.sq_tail_slot = 0;
    }

    fn write_entry(&mut self, descriptor: &UfsMcqQueueDescriptor, entry: SqEntry) -> Result<u32> {
        let index = self.sq_tail_index(descriptor)?;
        io_project!(self.sqe, [try: index]).copy_write(entry);

        self.sq_tail_slot = self.next_sq_tail_slot(descriptor);

        Ok(Self::sq_slot_offset(self.sq_tail_slot))
    }
}

impl UfsMcqCompletionQueue {
    fn dma_addr(&self) -> dma::DmaAddress {
        self.cqe.dma_handle()
    }

    fn tail_slot(&self) -> u32 {
        self.cq_tail_slot
    }

    fn head_slot(&self) -> u32 {
        self.cq_head_slot
    }

    fn update_tail(&mut self, reg: &UfsReg, descriptor: &UfsMcqQueueDescriptor) -> Result<()> {
        self.cq_tail_slot = descriptor.offset_to_slot(
            reg.read_mcq_cq_tail(descriptor.oprs(), descriptor.id() as usize)?,
            core::mem::size_of::<CqEntry>() as u32,
        )?;
        Ok(())
    }

    fn is_empty(&self) -> bool {
        self.cq_head_slot == self.cq_tail_slot
    }

    fn reset(&mut self) {
        self.cq_tail_slot = 0;
        self.cq_head_slot = 0;
    }

    fn consume_entry(&mut self, descriptor: &UfsMcqQueueDescriptor) -> Result<Option<CqEntry>> {
        let index = self.cq_head_slot as usize;
        if index >= descriptor.max_entries as usize {
            return Err(EINVAL);
        }

        let cqe = io_project!(self.cqe, [try: index]).copy_read();
        if cqe.is_empty() {
            return Ok(None);
        }

        io_project!(self.cqe, [try: index]).copy_write(CqEntry::default());

        self.cq_head_slot += 1;
        if self.cq_head_slot == descriptor.max_entries {
            self.cq_head_slot = 0;
        }

        Ok(Some(cqe))
    }

    fn commit_head(&self, reg: &UfsReg, descriptor: &UfsMcqQueueDescriptor) -> Result<()> {
        reg.write_mcq_cq_head(
            descriptor.oprs(),
            descriptor.id() as usize,
            self.cq_head_slot * core::mem::size_of::<CqEntry>() as u32,
        )
    }
}

#[pin_data]
struct McqHardwareQueue {
    descriptor: UfsMcqQueueDescriptor,
    #[pin]
    submission: SpinLock<UfsMcqSubmissionQueue>,
    #[pin]
    completion: SpinLock<UfsMcqCompletionQueue>,
}

impl McqHardwareQueue {
    fn new(queue: UfsMcqQueue) -> Result<Arc<Self>> {
        let (descriptor, submission, completion) = queue.into_parts();
        Arc::pin_init(
            pin_init!(Self {
                descriptor,
                submission <- new_spinlock!(submission),
                completion <- new_spinlock!(completion),
            }),
            GFP_KERNEL,
        )
    }

    fn submit<F>(&self, reg: &UfsReg, dma: &UfsDma, tag: u32, publish: F) -> SubmissionOutcome
    where
        F: FnOnce() -> Result<()>,
    {
        let mut submission = self.submission.lock();
        match submission.is_full(reg, &self.descriptor) {
            Ok(true) => return SubmissionOutcome::NotSubmitted(EBUSY),
            Err(e) => return SubmissionOutcome::NotSubmitted(e),
            Ok(false) => {}
        }

        let sqe = match dma.transfer_request_desc(tag as usize) {
            Ok(sqe) => sqe,
            Err(e) => return SubmissionOutcome::NotSubmitted(e),
        };
        let previous_tail = submission.sq_tail_slot();
        let tail = match submission.write_entry(&self.descriptor, sqe) {
            Ok(tail) => tail,
            Err(e) => return SubmissionOutcome::NotSubmitted(e),
        };
        if let Err(e) = publish() {
            submission.set_sq_tail_slot(previous_tail);
            return SubmissionOutcome::NotSubmitted(e);
        }

        barrier::dma_mb(barrier::Write);
        match reg.write_mcq_sq_tail(self.descriptor.oprs(), self.descriptor.id() as usize, tail) {
            Ok(()) => SubmissionOutcome::Submitted,
            Err(e) => SubmissionOutcome::PublishFailed(e),
        }
    }

    fn collect_completions(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        completed_requests: &mut CompletedRequests,
    ) -> Result<()> {
        let mut completion = self.completion.lock();
        self.descriptor.acknowledge_completion_events(reg)?;
        completion.update_tail(reg, &self.descriptor)?;
        barrier::dma_mb(barrier::Read);
        let mut consumed = false;
        let result = (|| {
            while !completion.is_empty() && !completed_requests.is_full() {
                let Some(cqe) = completion.consume_entry(&self.descriptor)? else {
                    completed_requests.record_fault(
                        "empty MCQ completion entry",
                        completion.head_slot() as usize,
                        Some(self.descriptor.id()),
                    );
                    break;
                };

                consumed = true;
                match dma.validate_cq_entry(&cqe, self.descriptor.id()) {
                    Ok(()) => completed_requests.insert_mcq(self.descriptor.id(), cqe)?,
                    Err(_) => completed_requests.record_fault(
                        "invalid MCQ completion descriptor",
                        usize::from(cqe.task_tag()),
                        Some(self.descriptor.id()),
                    ),
                }
            }
            Ok(())
        })();
        if consumed {
            completion.commit_head(reg, &self.descriptor)?;
        }

        result
    }
}

#[derive(Clone)]
pub(crate) struct McqHwQueue {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    queue: Arc<McqHardwareQueue>,
    poll: bool,
}

impl McqHwQueue {
    pub(crate) fn id(&self) -> u32 {
        self.queue.descriptor.id()
    }

    pub(crate) fn submit<F>(&self, tag: u32, publish: F) -> SubmissionOutcome
    where
        F: FnOnce() -> Result<()>,
    {
        self.queue.submit(&self.reg, &self.dma, tag, publish)
    }

    pub(crate) fn poll(&self, completed: &mut CompletedRequests) -> Result<()> {
        if !self.poll {
            return Err(EINVAL);
        }
        self.queue
            .collect_completions(&self.reg, &self.dma, completed)
    }
}

#[derive(Clone)]
pub(crate) struct McqInterruptQueue {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    queue: Arc<McqHardwareQueue>,
}

impl McqInterruptQueue {
    pub(crate) fn id(&self) -> u32 {
        self.queue.descriptor.id()
    }

    pub(crate) fn acknowledge_completion(&self) -> Result<bool> {
        self.queue
            .descriptor
            .acknowledge_completion_events(&self.reg)
    }

    pub(crate) fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        self.queue
            .collect_completions(&self.reg, &self.dma, completed)
    }
}

#[pin_data]
struct McqQueueSet {
    queues: KVec<Arc<McqHardwareQueue>>,
}

impl McqQueueSet {
    fn new(queues: KVec<Arc<McqHardwareQueue>>) -> impl PinInit<Self> {
        pin_init!(Self { queues })
    }

    fn len(&self) -> usize {
        self.queues.len()
    }

    fn poll_completions(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        nr_queues: usize,
        completed_requests: &mut CompletedRequests,
    ) -> Result<()> {
        for queue in self.queues.iter().take(nr_queues) {
            queue.collect_completions(reg, dma, completed_requests)?;
            if completed_requests.is_full() {
                break;
            }
        }

        Ok(())
    }

    fn dump_state(&self, reg: &UfsReg, tag: usize, reason: &str) {
        if self.queues.is_empty() {
            pr_err!(
                "[RUFS] ufs_queue: MCQ dump reason={} tag={} queues=unallocated\n",
                reason,
                tag,
            );
            return;
        }

        for queue in self.queues.iter() {
            let descriptor = &queue.descriptor;
            let id = descriptor.id() as usize;
            let sq_head = reg
                .read_mcq_sq_head(descriptor.oprs(), id)
                .unwrap_or(u32::MAX);
            let sq_tail = reg
                .read_mcq_sq_tail(descriptor.oprs(), id)
                .unwrap_or(u32::MAX);
            let cq_head = reg
                .read_mcq_cq_head(descriptor.oprs(), id)
                .unwrap_or(u32::MAX);
            let cq_tail = reg
                .read_mcq_cq_tail(descriptor.oprs(), id)
                .unwrap_or(u32::MAX);
            let cqis = reg.read_mcq_cqis(descriptor.oprs(), id).unwrap_or(u32::MAX);
            let sq_tail_slot = queue.submission.lock().sq_tail_slot();
            let completion = queue.completion.lock();

            pr_err!(
                "[RUFS] ufs_queue: MCQ state reason={} tag={} q={} sqhp={} sqtp={} cqhp={} cqtp={} cqis={:#x} sw_sq_tail={} sw_cq_head={} sw_cq_tail={}\n",
                reason,
                tag,
                id,
                sq_head,
                sq_tail,
                cq_head,
                cq_tail,
                cqis,
                sq_tail_slot,
                completion.head_slot(),
                completion.tail_slot(),
            );
        }
    }

    fn configure_registers_with_interrupt_queues(
        &self,
        reg: &UfsReg,
        layout: &McqRegisterLayout,
        interrupt_queues: usize,
    ) -> Result<()> {
        if interrupt_queues > self.queues.len() {
            return Err(EINVAL);
        }

        for queue in self.queues.iter() {
            let descriptor = &queue.descriptor;
            let id = descriptor.id() as usize;
            let mut submission = queue.submission.lock();
            let mut completion = queue.completion.lock();
            let sq_dma_addr = submission.dma_addr() as u64;
            let cq_dma_addr = completion.dma_addr() as u64;

            reg.set_mcq_sq_base_addr(layout, id, sq_dma_addr)?;
            reg.write_mcq_sqdao(
                layout,
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Sqd, id),
            )?;
            reg.write_mcq_sqisao(
                layout,
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Sqis, id),
            )?;

            reg.set_mcq_cq_base_addr(layout, id, cq_dma_addr)?;
            reg.write_mcq_cqdao(
                layout,
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Cqd, id),
            )?;
            reg.write_mcq_cqisao(
                layout,
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Cqis, id),
            )?;

            submission.reset();
            completion.reset();
            if id < interrupt_queues {
                reg.enable_mcq_cq_tail_push_intr(descriptor.oprs(), id)?;
            }
            reg.enable_mcq_cq(layout, id, descriptor.max_entries() as usize)?;
            reg.enable_mcq_sq(layout, id, descriptor.max_entries() as usize, id)?;
        }

        Ok(())
    }
}

pub(crate) struct McqTransferBackend {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    config: McqConfig,
    register_layout: McqRegisterLayout,
    queues: Arc<McqQueueSet>,
}

impl McqTransferBackend {
    pub(crate) fn new(config: McqConfig, reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        let register_layout = reg.mcq_register_layout()?;
        let oprs = register_layout.oprs();
        let mut hardware_queues = KVec::new();
        let ring_entries = u32::try_from(config.ring_entries).map_err(|_| EOVERFLOW)?;
        for id in 0..config.total_queues {
            let queue = UfsMcqQueue::new(
                dma.dev(),
                u32::try_from(id).map_err(|_| EOVERFLOW)?,
                ring_entries,
                oprs,
            )?;
            hardware_queues.push(McqHardwareQueue::new(queue)?, GFP_KERNEL)?;
        }
        let queues = Arc::pin_init(McqQueueSet::new(hardware_queues), GFP_KERNEL)?;

        Ok(Self {
            reg,
            dma,
            config,
            register_layout,
            queues,
        })
    }

    pub(crate) fn queue_depth(&self) -> usize {
        self.config.queue_depth
    }

    pub(crate) fn allocated_queues(&self) -> usize {
        self.queues.len()
    }

    pub(crate) fn interrupt_queues(&self) -> Result<KVec<McqInterruptQueue>> {
        let mut interrupt_queues = KVec::new();
        for queue in self.queues.queues.iter().take(self.config.interrupt_queues) {
            interrupt_queues.push(
                McqInterruptQueue {
                    reg: self.reg.clone(),
                    dma: self.dma.clone(),
                    queue: queue.clone(),
                },
                GFP_KERNEL,
            )?;
        }
        if interrupt_queues.len() != self.config.interrupt_queues {
            return Err(EINVAL);
        }

        Ok(interrupt_queues)
    }

    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        let mut hw_queues = KVec::new();
        for queue in self.queues.queues.iter() {
            let id = queue.descriptor.id() as usize;
            hw_queues.push(
                UfsHwQueue::mcq(McqHwQueue {
                    reg: self.reg.clone(),
                    dma: self.dma.clone(),
                    queue: queue.clone(),
                    poll: self.config.is_poll_queue(id),
                }),
                GFP_KERNEL,
            )?;
        }
        Ok(hw_queues)
    }

    fn prepare(&self) -> Result<()> {
        self.queues.configure_registers_with_interrupt_queues(
            &self.reg,
            &self.register_layout,
            self.config.interrupt_queues,
        )
    }

    fn enable(&self) {
        self.reg.enable_mcq_mode()
    }

    pub(crate) fn activate(&self) -> Result<()> {
        self.prepare()?;
        self.reg.config_mcq_max_active_cmds(
            u32::try_from(self.queue_depth()).map_err(|_| EOVERFLOW)?,
        )?;
        self.enable();
        Ok(())
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        self.queues.dump_state(&self.reg, tag, reason);
    }

    // MCQ CQE consumption is destructive because the software CQ head advances.
    // Snapshot each CQE before returning its tag so request finalization can
    // decode the consumed CQE after the backend lock is released.
    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        self.queues.poll_completions(
            &self.reg,
            &self.dma,
            self.config.interrupt_queues,
            completed,
        )
    }
}

impl UfsTransferOps for McqTransferBackend {
    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        McqTransferBackend::hw_queues(self)
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        McqTransferBackend::dump_state(self, tag, reason);
    }

    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        McqTransferBackend::collect_completions(self, completed)
    }

    fn enable_interrupts(&self) {
        self.reg.enable_transfer_interrupts();
        self.reg.enable_mcq_interrupts()
    }

    fn reset(&self) -> Result<()> {
        self.activate()
    }
}
