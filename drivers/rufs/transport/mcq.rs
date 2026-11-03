// SPDX-License-Identifier: GPL-2.0

//! UFSHCI multi-circular queue transport.

use crate::command::{TaskTag, TASK_TAG_COUNT};
use crate::dma::UfsDma;
use crate::hci::descriptor::{CqEntry, SqEntry, UtpOcs};
use crate::reg::{McqRegisterLayout, UfsMcqOprRegion, UfsMcqOprSet, UfsReg};
use crate::transport::{
    CompletionBatch, McqConfig, RequestDmaAccessEnded, SubmissionOutcome,
    UfsHwQueue, UfsTransferOps,
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
    recovery: KVec<McqRecoveryCompletion>,
}

struct McqRecoveryCompletion {
    task_tag: TaskTag,
    cqe: Option<CqEntry>,
}

#[derive(Clone, Copy)]
pub(crate) struct McqLogicalResetTarget {
    queue_id: u32,
    task_tag: TaskTag,
    task_claimed: bool,
    unfetched: bool,
}

impl McqLogicalResetTarget {
    pub(crate) fn new(queue_id: u32, task_tag: TaskTag, task_claimed: bool) -> Self {
        Self {
            queue_id,
            task_tag,
            task_claimed,
            unfetched: false,
        }
    }

    pub(crate) fn task_claimed(&self) -> bool {
        self.task_claimed
    }
}

const MCQ_TASK_TAG_WORDS: usize = TASK_TAG_COUNT / u64::BITS as usize;

pub(crate) struct McqRequestsDmaAccessEnded {
    task_tags: [u64; MCQ_TASK_TAG_WORDS],
}

impl McqRequestsDmaAccessEnded {
    fn insert(&mut self, task_tag: TaskTag) -> Result<()> {
        let index = task_tag.index() / u64::BITS as usize;
        let bit = task_tag.index() % u64::BITS as usize;
        let mask = 1u64 << bit;
        if self.task_tags[index] & mask != 0 {
            return Err(EEXIST);
        }
        self.task_tags[index] |= mask;
        Ok(())
    }

    pub(crate) fn contains(&self, task_tag: TaskTag) -> bool {
        let index = task_tag.index() / u64::BITS as usize;
        let bit = task_tag.index() % u64::BITS as usize;
        self.task_tags[index] & (1u64 << bit) != 0
    }

    pub(crate) fn take(&mut self, task_tag: TaskTag) -> Result<RequestDmaAccessEnded> {
        let index = task_tag.index() / u64::BITS as usize;
        let bit = task_tag.index() % u64::BITS as usize;
        let mask = 1u64 << bit;
        if self.task_tags[index] & mask == 0 {
            return Err(ENOENT);
        }
        self.task_tags[index] &= !mask;
        Ok(RequestDmaAccessEnded::new(task_tag))
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.task_tags.iter().all(|word| *word == 0)
    }

    pub(crate) fn len(&self) -> usize {
        self.task_tags
            .iter()
            .map(|word| word.count_ones() as usize)
            .sum()
    }
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
        let recovery = KVec::with_capacity(entries, GFP_KERNEL)?;
        Ok(Self {
            descriptor: UfsMcqQueueDescriptor {
                id,
                max_entries,
                oprs,
            },
            submission: UfsMcqSubmissionQueue {
                sqe: dma::Coherent::<SqEntry>::zeroed_slice_zerocopy(dev, entries, GFP_KERNEL)?,
                sq_tail_slot: 0,
            },
            completion: UfsMcqCompletionQueue {
                cqe: dma::Coherent::<CqEntry>::zeroed_slice_zerocopy(dev, entries, GFP_KERNEL)?,
                cq_tail_slot: 0,
                cq_head_slot: 0,
                recovery,
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
        self.sqe.dma_address()
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

    fn find_task_slot(
        &self,
        reg: &UfsReg,
        descriptor: &UfsMcqQueueDescriptor,
        target: &SqEntry,
    ) -> Result<Option<u32>> {
        let mut slot = descriptor.offset_to_slot(
            reg.read_mcq_sq_head(descriptor.oprs(), descriptor.id() as usize)?,
            core::mem::size_of::<SqEntry>() as u32,
        )?;

        while slot != self.sq_tail_slot {
            let sqe = io_project!(self.sqe, [try: slot as usize]).copy_read();
            if sqe.matches_command_descriptor(target) {
                return Ok(Some(slot));
            }
            slot = self.next_slot(descriptor, slot);
        }

        Ok(None)
    }

    fn next_slot(&self, descriptor: &UfsMcqQueueDescriptor, slot: u32) -> u32 {
        let next = slot + 1;
        if next == descriptor.max_entries {
            0
        } else {
            next
        }
    }

    fn nullify_slot(&mut self, slot: u32) -> Result<()> {
        let mut sqe = io_project!(self.sqe, [try: slot as usize]).copy_read();
        sqe.nullify();
        io_project!(self.sqe, [try: slot as usize]).copy_write(sqe);
        Ok(())
    }
}

impl UfsMcqCompletionQueue {
    fn dma_addr(&self) -> dma::DmaAddress {
        self.cqe.dma_address()
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
        self.recovery.clear();
    }

    fn contains_task_tag(
        &self,
        descriptor: &UfsMcqQueueDescriptor,
        task_tag: TaskTag,
    ) -> Result<bool> {
        let mut slot = self.cq_head_slot;
        while slot != self.cq_tail_slot {
            let cqe = io_project!(self.cqe, [try: slot as usize]).copy_read();
            if cqe.is_empty() {
                return Err(EIO);
            }
            if cqe.task_tag() == task_tag.value() {
                return Ok(true);
            }
            slot += 1;
            if slot == descriptor.max_entries {
                slot = 0;
            }
        }
        Ok(false)
    }

    fn commit_task_recovery(
        &mut self,
        descriptor: &UfsMcqQueueDescriptor,
        task_tag: TaskTag,
    ) -> Result<()> {
        if self
            .recovery
            .iter()
            .any(|recovery| recovery.task_tag == task_tag)
        {
            return Err(EBUSY);
        }
        if self.contains_task_tag(descriptor, task_tag)? {
            return Err(EAGAIN);
        }
        self.recovery
            .push_within_capacity(McqRecoveryCompletion {
                task_tag,
                cqe: None,
            })
            .map_err(|_| EOVERFLOW)?;
        Ok(())
    }

    fn store_recovery_completion(&mut self, cqe: CqEntry) -> Result<bool> {
        let Some(recovery) = self
            .recovery
            .iter_mut()
            .find(|recovery| cqe.task_tag() == recovery.task_tag.value())
        else {
            return Ok(false);
        };
        if recovery.cqe.replace(cqe).is_some() {
            return Err(EIO);
        }
        Ok(true)
    }

    fn take_recovery_completion(&mut self, task_tag: TaskTag) -> Result<CqEntry> {
        let Some(index) = self
            .recovery
            .iter()
            .position(|recovery| recovery.task_tag == task_tag)
        else {
            return Err(ENOENT);
        };
        if self.recovery[index].cqe.is_none() {
            return Err(EAGAIN);
        }
        let recovery = self.recovery.remove(index).map_err(|_| EIO)?;
        recovery.cqe.ok_or(EIO)
    }

    fn has_recovery_completion(&self, task_tag: TaskTag) -> Result<bool> {
        self.recovery
            .iter()
            .find(|recovery| recovery.task_tag == task_tag)
            .map(|recovery| recovery.cqe.is_some())
            .ok_or(ENOENT)
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
        completed_requests: &mut CompletionBatch,
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
                    Ok(()) if completion.store_recovery_completion(cqe)? => {}
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

    pub(crate) fn supports_polling(&self) -> bool {
        self.poll
    }

    pub(crate) fn submit<F>(&self, tag: u32, publish: F) -> SubmissionOutcome
    where
        F: FnOnce() -> Result<()>,
    {
        self.queue.submit(&self.reg, &self.dma, tag, publish)
    }

    pub(crate) fn poll(&self, completed: &mut CompletionBatch) -> Result<()> {
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

    pub(crate) fn collect_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
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
        completed_requests: &mut CompletionBatch,
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
            let sq_dma_addr = submission.dma_addr();
            let cq_dma_addr = completion.dma_addr();

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
    fn collect_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        self.queues.poll_completions(
            &self.reg,
            &self.dma,
            self.config.interrupt_queues,
            completed,
        )
    }

    fn collect_recovery_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        self.queues
            .poll_completions(&self.reg, &self.dma, self.queues.len(), completed)
    }

    fn claim_task(&self, queue_id: u32, task_tag: TaskTag) -> Result<()> {
        let queue_id = usize::try_from(queue_id).map_err(|_| EINVAL)?;
        let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
        let _submission = queue.submission.lock();

        self.reg
            .stop_mcq_sq(queue.descriptor.oprs(), queue_id)?;

        let commit = {
            let mut completion = queue.completion.lock();
            completion.update_tail(&self.reg, &queue.descriptor)?;
            barrier::dma_mb(barrier::Read);
            completion.commit_task_recovery(&queue.descriptor, task_tag)
        };
        if let Err(error) = commit {
            self.reg
                .start_mcq_sq(queue.descriptor.oprs(), queue_id)?;
            return Err(error);
        }

        Ok(())
    }

    fn begin_task_cleanup(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
        lun: u8,
    ) -> Result<()> {
        let queue_id = usize::try_from(queue_id).map_err(|_| EINVAL)?;
        let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
        self.reg.write_mcq_sq_cleanup_target(
            queue.descriptor.oprs(),
            queue_id,
            lun,
            task_tag.value(),
        )?;
        self.reg
            .initiate_mcq_sq_cleanup(queue.descriptor.oprs(), queue_id)?;
        if self
            .reg
            .mcq_sq_cleanup_error_code(queue.descriptor.oprs(), queue_id)?
            != 0
        {
            return Err(EIO);
        }
        Ok(())
    }

    fn claim_sqe_nullification(&self, queue_id: u32, task_tag: TaskTag) -> Result<()> {
        let queue_id = usize::try_from(queue_id).map_err(|_| EINVAL)?;
        let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
        let mut submission = queue.submission.lock();

        self.reg
            .stop_mcq_sq(queue.descriptor.oprs(), queue_id)?;

        let mut claimed = false;
        let result = (|| {
            let target = self.dma.transfer_request_desc(task_tag.index())?;
            let mut completion = queue.completion.lock();
            completion.update_tail(&self.reg, &queue.descriptor)?;
            barrier::dma_mb(barrier::Read);
            if completion.contains_task_tag(&queue.descriptor, task_tag)? {
                return Err(EAGAIN);
            }

            let Some(slot) = submission.find_task_slot(
                &self.reg,
                &queue.descriptor,
                &target,
            )? else {
                return Err(ENOENT);
            };

            completion.commit_task_recovery(&queue.descriptor, task_tag)?;
            claimed = true;
            submission.nullify_slot(slot)?;
            barrier::dma_mb(barrier::Write);
            Ok(())
        })();

        if let Err(error) = result {
            if claimed {
                return Err(EIO);
            }
            self.reg
                .start_mcq_sq(queue.descriptor.oprs(), queue_id)?;
            return Err(error);
        }

        if let Err(error) = self
            .reg
            .start_mcq_sq(queue.descriptor.oprs(), queue_id)
        {
            return Err(error);
        }

        Ok(())
    }

    fn finish_sqe_nullification(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<RequestDmaAccessEnded> {
        let queue_id = usize::try_from(queue_id).map_err(|_| EINVAL)?;
        let queue = self
            .queues
            .queues
            .get(queue_id)
            .ok_or(EINVAL)?;
        let cqe = queue
            .completion
            .lock()
            .take_recovery_completion(task_tag)?;
        if !matches!(cqe.overall_status().into(), UtpOcs::Aborted) {
            return Err(EIO);
        }
        Ok(RequestDmaAccessEnded::new(task_tag))
    }

    fn finish_task(&self, queue_id: u32, task_tag: TaskTag) -> Result<RequestDmaAccessEnded> {
        let queue_id = usize::try_from(queue_id).map_err(|_| EINVAL)?;
        let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
        let cqe = queue
            .completion
            .lock()
            .take_recovery_completion(task_tag)?;
        if !matches!(cqe.overall_status().into(), UtpOcs::Aborted) {
            return Err(EIO);
        }
        self.reg
            .start_mcq_sq(queue.descriptor.oprs(), queue_id)?;
        Ok(RequestDmaAccessEnded::new(task_tag))
    }

    fn restart_logical_unit_queues(
        &self,
        targets: &[McqLogicalResetTarget],
    ) -> Result<()> {
        for (index, target) in targets.iter().enumerate() {
            if targets[..index]
                .iter()
                .any(|previous| previous.queue_id == target.queue_id)
            {
                continue;
            }
            let queue_id = usize::try_from(target.queue_id).map_err(|_| EINVAL)?;
            let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
            self.reg
                .start_mcq_sq(queue.descriptor.oprs(), queue_id)?;
        }
        Ok(())
    }

    fn restart_new_logical_unit_queues(
        &self,
        targets: &[McqLogicalResetTarget],
        count: usize,
    ) -> Result<()> {
        for (index, target) in targets[..count].iter().enumerate() {
            if targets[..index]
                .iter()
                .any(|previous| previous.queue_id == target.queue_id)
                || targets
                    .iter()
                    .any(|candidate| candidate.queue_id == target.queue_id && candidate.task_claimed)
            {
                continue;
            }
            let queue_id = usize::try_from(target.queue_id).map_err(|_| EINVAL)?;
            let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
            self.reg
                .start_mcq_sq(queue.descriptor.oprs(), queue_id)?;
        }
        Ok(())
    }

    fn claim_logical_unit(
        &self,
        targets: &mut [McqLogicalResetTarget],
    ) -> Result<()> {
        if targets.is_empty() {
            return Err(EINVAL);
        }

        let mut task_claimed = 0usize;
        for (index, target) in targets.iter().enumerate() {
            let queue_id = usize::try_from(target.queue_id).map_err(|_| EINVAL)?;
            if self.queues.queues.get(queue_id).is_none()
                || targets[..index]
                    .iter()
                    .any(|previous| previous.task_tag == target.task_tag)
            {
                return Err(EINVAL);
            }
            if target.task_claimed {
                task_claimed += 1;
                self.queues.queues[queue_id]
                    .completion
                    .lock()
                    .has_recovery_completion(target.task_tag)?;
            }
        }
        if task_claimed > 1 {
            return Err(EINVAL);
        }

        for (index, target) in targets.iter().enumerate() {
            if targets[..index]
                .iter()
                .any(|previous| previous.queue_id == target.queue_id)
                || targets
                    .iter()
                    .any(|candidate| candidate.queue_id == target.queue_id && candidate.task_claimed)
            {
                continue;
            }
            let queue_id = usize::try_from(target.queue_id).map_err(|_| EINVAL)?;
            let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
            if let Err(error) = self
                .reg
                .stop_mcq_sq(queue.descriptor.oprs(), queue_id)
            {
                let _ = self.restart_new_logical_unit_queues(targets, index);
                return Err(error);
            }
        }

        let mut claimed = task_claimed != 0;
        for index in 0..targets.len() {
            let result = (|| {
                let queue_id =
                    usize::try_from(targets[index].queue_id).map_err(|_| EINVAL)?;
                let task_tag = targets[index].task_tag;
                let task_claimed = targets[index].task_claimed;
                let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
                let mut submission = queue.submission.lock();
                let target = self.dma.transfer_request_desc(task_tag.index())?;
                let mut completion = queue.completion.lock();
                completion.update_tail(&self.reg, &queue.descriptor)?;
                barrier::dma_mb(barrier::Read);

                if !task_claimed {
                    completion.commit_task_recovery(&queue.descriptor, task_tag)?;
                    claimed = true;
                }
                let slot = submission.find_task_slot(
                    &self.reg,
                    &queue.descriptor,
                    &target,
                )?;
                if let Some(slot) = slot {
                    submission.nullify_slot(slot)?;
                    targets[index].unfetched = true;
                }
                Ok(())
            })();
            if let Err(error) = result {
                if claimed {
                    barrier::dma_mb(barrier::Write);
                    return Err(EIO);
                }
                let _ = self.restart_new_logical_unit_queues(targets, targets.len());
                return Err(error);
            }
        }
        barrier::dma_mb(barrier::Write);

        Ok(())
    }

    fn begin_logical_unit_cleanup(
        &self,
        targets: &[McqLogicalResetTarget],
        lun: u8,
    ) -> Result<()> {
        for target in targets.iter().filter(|target| !target.unfetched) {
            let queue_id = usize::try_from(target.queue_id).map_err(|_| EINVAL)?;
            let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
            if queue
                .completion
                .lock()
                .has_recovery_completion(target.task_tag)?
            {
                continue;
            }
            self.reg.write_mcq_sq_cleanup_target(
                queue.descriptor.oprs(),
                queue_id,
                lun,
                target.task_tag.value(),
            )?;
            self.reg
                .initiate_mcq_sq_cleanup(queue.descriptor.oprs(), queue_id)?;
            let cleanup_status = self.reg.mcq_sq_cleanup_error_code(
                queue.descriptor.oprs(),
                queue_id,
            )?;
            if cleanup_status > 1 {
                return Err(EIO);
            }
            // A status of one means that the SQ no longer contains the
            // target. The request may have completed after the reset was
            // committed; the bounded CQ retirement below decides whether DMA
            // really ended. Other cleanup errors make that proof impossible.
        }

        self.restart_logical_unit_queues(targets)
    }

    fn finish_logical_unit(
        &self,
        targets: &[McqLogicalResetTarget],
    ) -> Result<McqRequestsDmaAccessEnded> {
        for target in targets.iter() {
            let queue_id = usize::try_from(target.queue_id).map_err(|_| EINVAL)?;
            let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
            if !queue
                .completion
                .lock()
                .has_recovery_completion(target.task_tag)?
            {
                return Err(EAGAIN);
            }
        }

        let mut proof = McqRequestsDmaAccessEnded {
            task_tags: [0; MCQ_TASK_TAG_WORDS],
        };
        for target in targets.iter() {
            let queue_id = usize::try_from(target.queue_id).map_err(|_| EINVAL)?;
            let queue = self.queues.queues.get(queue_id).ok_or(EINVAL)?;
            queue
                .completion
                .lock()
                .take_recovery_completion(target.task_tag)?;
            proof.insert(target.task_tag)?;
        }
        Ok(proof)
    }
}

impl UfsTransferOps for McqTransferBackend {
    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        McqTransferBackend::hw_queues(self)
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        McqTransferBackend::dump_state(self, tag, reason);
    }

    fn collect_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        McqTransferBackend::collect_completions(self, completed)
    }

    fn collect_recovery_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        McqTransferBackend::collect_recovery_completions(self, completed)
    }

    fn enable_interrupts(&self) {
        self.reg.enable_transfer_interrupts();
        self.reg.enable_mcq_interrupts()
    }

    fn reset(&self) -> Result<()> {
        self.activate()
    }

    fn supports_mcq_task_abort(&self) -> bool {
        true
    }

    fn claim_mcq_task(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<()> {
        self.claim_task(queue_id, task_tag)
    }

    fn begin_mcq_task_cleanup(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
        lun: u8,
    ) -> Result<()> {
        self.begin_task_cleanup(queue_id, task_tag, lun)
    }

    fn finish_mcq_task(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<RequestDmaAccessEnded> {
        self.finish_task(queue_id, task_tag)
    }

    fn claim_mcq_sqe_nullification(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<()> {
        self.claim_sqe_nullification(queue_id, task_tag)
    }

    fn finish_mcq_sqe_nullification(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<RequestDmaAccessEnded> {
        self.finish_sqe_nullification(queue_id, task_tag)
    }

    fn supports_mcq_logical_reset(&self) -> bool {
        true
    }

    fn claim_mcq_logical_unit(
        &self,
        targets: &mut [McqLogicalResetTarget],
    ) -> Result<()> {
        self.claim_logical_unit(targets)
    }

    fn begin_mcq_logical_unit_cleanup(
        &self,
        targets: &[McqLogicalResetTarget],
        lun: u8,
    ) -> Result<()> {
        self.begin_logical_unit_cleanup(targets, lun)
    }

    fn finish_mcq_logical_unit(
        &self,
        targets: &[McqLogicalResetTarget],
    ) -> Result<McqRequestsDmaAccessEnded> {
        self.finish_logical_unit(targets)
    }
}
