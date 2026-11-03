// SPDX-License-Identifier: GPL-2.0

//! UFS transfer transports.

use crate::command::{TaskTag, TASK_TAG_COUNT};
use crate::dma::UfsDma;
use crate::hci::descriptor::CqEntry;
use crate::reg::UfsReg;
use kernel::cpu;
use kernel::prelude::*;
use kernel::sync::Arc;

pub(crate) mod mcq;
pub(crate) mod sdb;

pub(crate) use mcq::{
    McqInterruptQueue, McqLogicalResetTarget, McqRequestsDmaAccessEnded,
};
use mcq::{
    McqHwQueue, McqTransferBackend,
};
pub(crate) use sdb::SdbRequestsDmaAccessEnded;
use sdb::{SdbHwQueue, SdbTransferBackend};

pub(crate) struct RequestDmaAccessEnded {
    task_tag: TaskTag,
}

impl RequestDmaAccessEnded {
    fn new(task_tag: TaskTag) -> Self {
        Self { task_tag }
    }

    pub(crate) fn task_tag(&self) -> TaskTag {
        self.task_tag
    }
}

const UFS_MCQ_DEFAULT_READ_QUEUES: usize = 0;
const UFS_MCQ_DEFAULT_POLL_QUEUES: usize = 1;
const COMPLETION_BATCH_SIZE: usize = 16;

fn possible_cpus() -> usize {
    (cpu::nr_cpu_ids() as usize).max(1)
}

#[derive(Copy, Clone)]
pub(crate) struct McqConfig {
    pub(crate) max_queues: usize,
    pub(crate) total_queues: usize,
    pub(crate) default_queues: usize,
    pub(crate) read_queues: usize,
    pub(crate) interrupt_queues: usize,
    pub(crate) poll_queues: usize,
    pub(crate) queue_depth: usize,
    pub(crate) ring_entries: usize,
}

impl McqConfig {
    pub(crate) fn is_poll_queue(&self, queue: usize) -> bool {
        queue >= self.interrupt_queues && queue < self.total_queues
    }
}

#[derive(Copy, Clone)]
pub(crate) enum UfsTransferConfig {
    Sdb { tag_count: usize },
    Mcq(McqConfig),
}

impl UfsTransferConfig {
    pub(crate) fn new(reg: &UfsReg) -> Result<Self> {
        let hardware_mcq = reg.mcq_hardware_supported();
        let variant_mcq = reg.mcq_variant_enabled();

        if !hardware_mcq || !variant_mcq {
            pr_info!(
                "[RUFS] ufs_queue: SDB selected CAP.MCQ={} variant.MCQ={}\n",
                hardware_mcq,
                variant_mcq,
            );
            let tag_count = reg.nutrs();
            if tag_count == 0 || tag_count > u32::BITS as usize {
                return Err(EINVAL);
            }
            return Ok(Self::Sdb { tag_count });
        }

        let max_queues = reg.mcq_max_queues();
        let read_queues = UFS_MCQ_DEFAULT_READ_QUEUES;
        let poll_queues = UFS_MCQ_DEFAULT_POLL_QUEUES;
        let reserved_queues = read_queues.checked_add(poll_queues).ok_or(EOVERFLOW)?;
        if max_queues <= reserved_queues {
            return Err(ENOTSUPP);
        }

        let default_queues = core::cmp::min(max_queues - reserved_queues, possible_cpus());
        let interrupt_queues = default_queues.checked_add(read_queues).ok_or(EOVERFLOW)?;
        let total_queues = interrupt_queues.checked_add(poll_queues).ok_or(EOVERFLOW)?;
        let queue_depth =
            reg.constrain_mcq_active_commands(core::cmp::min(reg.nutrs_mcq(), TASK_TAG_COUNT));
        let ring_entries = queue_depth.checked_add(1).ok_or(EOVERFLOW)?;
        if interrupt_queues == 0 || queue_depth == 0 {
            return Err(EINVAL);
        }

        Ok(Self::Mcq(McqConfig {
            max_queues,
            total_queues,
            default_queues,
            read_queues,
            interrupt_queues,
            poll_queues,
            queue_depth,
            ring_entries,
        }))
    }

    pub(crate) fn queue_depth(&self) -> usize {
        match self {
            Self::Sdb { tag_count } => *tag_count,
            Self::Mcq(config) => config.queue_depth,
        }
    }
}

pub(crate) enum CompletionPayload {
    Sdb,
    Mcq(CqEntry),
}

pub(crate) struct BackendCompletion {
    task_tag: TaskTag,
    queue_id: u32,
    payload: CompletionPayload,
}

impl BackendCompletion {
    fn sdb(task_tag: TaskTag) -> Self {
        Self {
            task_tag,
            queue_id: 0,
            payload: CompletionPayload::Sdb,
        }
    }

    fn mcq(queue_id: u32, cqe: CqEntry) -> Self {
        Self {
            task_tag: TaskTag::from_value(cqe.task_tag()),
            queue_id,
            payload: CompletionPayload::Mcq(cqe),
        }
    }

    pub(crate) fn task_tag(&self) -> TaskTag {
        self.task_tag
    }

    pub(crate) fn queue_id(&self) -> u32 {
        self.queue_id
    }

    pub(crate) fn into_parts(self) -> (TaskTag, u32, CompletionPayload) {
        (self.task_tag, self.queue_id, self.payload)
    }
}

#[derive(Clone, Copy)]
pub(crate) struct CompletionFault {
    pub(crate) reason: &'static str,
    pub(crate) tag: usize,
    pub(crate) queue_id: Option<u32>,
}

pub(crate) struct CompletionBatch {
    completions: [Option<BackendCompletion>; COMPLETION_BATCH_SIZE],
    len: usize,
    pos: usize,
    fault: Option<CompletionFault>,
}

impl CompletionBatch {
    pub(crate) const fn capacity() -> usize {
        COMPLETION_BATCH_SIZE
    }

    pub(crate) fn new() -> Self {
        Self {
            completions: [const { None }; COMPLETION_BATCH_SIZE],
            len: 0,
            pos: 0,
            fault: None,
        }
    }

    fn insert(&mut self, completion: BackendCompletion) -> Result<()> {
        if self.len == self.completions.len() {
            return Err(ENOMEM);
        }

        self.completions[self.len] = Some(completion);
        self.len += 1;
        Ok(())
    }

    pub(crate) fn insert_mcq(&mut self, queue_id: u32, cqe: CqEntry) -> Result<()> {
        self.insert(BackendCompletion::mcq(queue_id, cqe))
    }

    pub(crate) fn insert_sdb_mask(&mut self, mut mask: u32) -> Result<u32> {
        let mut inserted = 0;

        while mask != 0 && !self.is_full() {
            let tag = mask.trailing_zeros();
            let tag_mask = 1u32 << tag;
            mask &= !tag_mask;
            self.insert(BackendCompletion::sdb(TaskTag::new(tag)?))?;
            inserted |= tag_mask;
        }

        Ok(inserted)
    }

    pub(crate) fn is_full(&self) -> bool {
        self.len == self.completions.len()
    }

    pub(crate) fn record_fault(&mut self, reason: &'static str, tag: usize, queue_id: Option<u32>) {
        if self.fault.is_none() {
            self.fault = Some(CompletionFault {
                reason,
                tag,
                queue_id,
            });
        }
    }

    pub(crate) fn take_fault(&mut self) -> Option<CompletionFault> {
        self.fault.take()
    }

    pub(crate) fn take_next(&mut self) -> Option<BackendCompletion> {
        if self.pos == self.len {
            return None;
        }

        let completion = self.completions[self.pos].take();
        self.pos += 1;
        completion
    }
}

pub(crate) enum SubmissionOutcome {
    Submitted,
    NotSubmitted(Error),
    PublishFailed(Error),
}

#[derive(Clone)]
pub(crate) struct UfsHwQueue {
    inner: UfsHwQueueKind,
}

#[derive(Clone)]
enum UfsHwQueueKind {
    Sdb(SdbHwQueue),
    Mcq(McqHwQueue),
}

impl UfsHwQueue {
    pub(crate) fn sdb(queue: SdbHwQueue) -> Self {
        Self {
            inner: UfsHwQueueKind::Sdb(queue),
        }
    }

    pub(crate) fn mcq(queue: McqHwQueue) -> Self {
        Self {
            inner: UfsHwQueueKind::Mcq(queue),
        }
    }

    pub(crate) fn id(&self) -> u32 {
        match &self.inner {
            UfsHwQueueKind::Sdb(queue) => queue.id(),
            UfsHwQueueKind::Mcq(queue) => queue.id(),
        }
    }

    pub(crate) fn submit<F>(&self, tag: u32, polled: bool, publish: F) -> SubmissionOutcome
    where
        F: FnOnce() -> Result<()>,
    {
        match &self.inner {
            UfsHwQueueKind::Sdb(queue) => queue.submit(tag, polled, publish),
            UfsHwQueueKind::Mcq(queue) => queue.submit(tag, publish),
        }
    }

    pub(crate) fn poll(&self, completed: &mut CompletionBatch) -> Result<()> {
        match &self.inner {
            UfsHwQueueKind::Sdb(queue) => queue.poll(completed),
            UfsHwQueueKind::Mcq(queue) => queue.poll(completed),
        }
    }
}

pub(crate) trait UfsTransferOps: Send + Sync {
    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>>;
    fn dump_state(&self, tag: usize, reason: &str);
    fn collect_completions(&self, completed: &mut CompletionBatch) -> Result<()>;
    fn collect_recovery_completions(&self, completed: &mut CompletionBatch) -> Result<()>;
    fn enable_interrupts(&self);
    fn reset(&self) -> Result<()>;
    fn supports_sdb_task_abort(&self) -> bool {
        false
    }
    fn claim_sdb_task(&self, _task_tag: TaskTag) -> Result<()> {
        Err(EOPNOTSUPP)
    }
    fn finish_sdb_task(&self, _task_tag: TaskTag) -> Result<RequestDmaAccessEnded> {
        Err(EOPNOTSUPP)
    }
    fn supports_sdb_logical_reset(&self) -> bool {
        false
    }
    fn claim_sdb_logical_unit(
        &self,
        _task_mask: u32,
        _task_claimed_mask: u32,
    ) -> Result<()> {
        Err(EOPNOTSUPP)
    }
    fn finish_sdb_logical_unit(
        &self,
        _task_mask: u32,
    ) -> Result<SdbRequestsDmaAccessEnded> {
        Err(EOPNOTSUPP)
    }
    fn supports_mcq_task_abort(&self) -> bool {
        false
    }
    fn claim_mcq_task(
        &self,
        _queue_id: u32,
        _task_tag: TaskTag,
    ) -> Result<()> {
        Err(EOPNOTSUPP)
    }
    fn begin_mcq_task_cleanup(
        &self,
        _queue_id: u32,
        _task_tag: TaskTag,
        _lun: u8,
    ) -> Result<()> {
        Err(EOPNOTSUPP)
    }
    fn finish_mcq_task(
        &self,
        _queue_id: u32,
        _task_tag: TaskTag,
    ) -> Result<RequestDmaAccessEnded> {
        Err(EOPNOTSUPP)
    }
    fn claim_mcq_sqe_nullification(
        &self,
        _queue_id: u32,
        _task_tag: TaskTag,
    ) -> Result<()> {
        Err(EOPNOTSUPP)
    }
    fn finish_mcq_sqe_nullification(
        &self,
        _queue_id: u32,
        _task_tag: TaskTag,
    ) -> Result<RequestDmaAccessEnded> {
        Err(EOPNOTSUPP)
    }
    fn supports_mcq_logical_reset(&self) -> bool {
        false
    }
    fn claim_mcq_logical_unit(
        &self,
        _targets: &mut [McqLogicalResetTarget],
    ) -> Result<()> {
        Err(EOPNOTSUPP)
    }
    fn begin_mcq_logical_unit_cleanup(
        &self,
        _targets: &[McqLogicalResetTarget],
        _lun: u8,
    ) -> Result<()> {
        Err(EOPNOTSUPP)
    }
    fn finish_mcq_logical_unit(
        &self,
        _targets: &[McqLogicalResetTarget],
    ) -> Result<McqRequestsDmaAccessEnded> {
        Err(EOPNOTSUPP)
    }
}

pub(crate) struct UfsTransferBackend {
    ops: KBox<dyn UfsTransferOps>,
    interrupt_queues: KVec<McqInterruptQueue>,
}

impl UfsTransferBackend {
    pub(crate) fn new(
        config: UfsTransferConfig,
        reg: Arc<UfsReg>,
        dma: Arc<UfsDma>,
    ) -> Result<Self> {
        let (ops, interrupt_queues) = match config {
            UfsTransferConfig::Sdb { .. } => (
                KBox::new(SdbTransferBackend::new(reg)?, GFP_KERNEL)? as KBox<dyn UfsTransferOps>,
                KVec::new(),
            ),
            UfsTransferConfig::Mcq(config) => {
                let backend = KBox::new(McqTransferBackend::new(config, reg, dma)?, GFP_KERNEL)?;
                let interrupt_queues = backend.interrupt_queues()?;
                backend.activate()?;
                pr_info!(
                    "[RUFS] ufs_queue: MCQ backend enabled queues={}/{} interrupt={} poll={} allocated={} depth={} ring_entries={}\n",
                    config.total_queues,
                    config.max_queues,
                    config.interrupt_queues,
                    config.poll_queues,
                    backend.allocated_queues(),
                    backend.queue_depth(),
                    config.ring_entries,
                );
                (backend as KBox<dyn UfsTransferOps>, interrupt_queues)
            }
        };

        Ok(Self {
            ops,
            interrupt_queues,
        })
    }

    pub(crate) fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        self.ops.hw_queues()
    }

    pub(crate) fn collect_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        self.ops.collect_completions(completed)
    }

    pub(crate) fn collect_recovery_completions(
        &self,
        completed: &mut CompletionBatch,
    ) -> Result<()> {
        self.ops.collect_recovery_completions(completed)
    }

    pub(crate) fn interrupt_queues(&self) -> &[McqInterruptQueue] {
        &self.interrupt_queues
    }

    pub(crate) fn enable_interrupts(&self) {
        self.ops.enable_interrupts()
    }

    pub(crate) fn dump_state(&self, tag: usize, reason: &str) {
        self.ops.dump_state(tag, reason)
    }

    pub(crate) fn reset(&self) -> Result<()> {
        self.ops.reset()
    }

    pub(crate) fn supports_sdb_task_abort(&self) -> bool {
        self.ops.supports_sdb_task_abort()
    }

    pub(crate) fn claim_sdb_task(&self, task_tag: TaskTag) -> Result<()> {
        self.ops.claim_sdb_task(task_tag)
    }

    pub(crate) fn finish_sdb_task(&self, task_tag: TaskTag) -> Result<RequestDmaAccessEnded> {
        self.ops.finish_sdb_task(task_tag)
    }

    pub(crate) fn supports_sdb_logical_reset(&self) -> bool {
        self.ops.supports_sdb_logical_reset()
    }

    pub(crate) fn claim_sdb_logical_unit(
        &self,
        task_mask: u32,
        task_claimed_mask: u32,
    ) -> Result<()> {
        self.ops
            .claim_sdb_logical_unit(task_mask, task_claimed_mask)
    }

    pub(crate) fn finish_sdb_logical_unit(
        &self,
        task_mask: u32,
    ) -> Result<SdbRequestsDmaAccessEnded> {
        self.ops.finish_sdb_logical_unit(task_mask)
    }

    pub(crate) fn supports_mcq_task_abort(&self) -> bool {
        self.ops.supports_mcq_task_abort()
    }

    pub(crate) fn claim_mcq_task(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<()> {
        self.ops.claim_mcq_task(queue_id, task_tag)
    }

    pub(crate) fn begin_mcq_task_cleanup(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
        lun: u8,
    ) -> Result<()> {
        self.ops.begin_mcq_task_cleanup(queue_id, task_tag, lun)
    }

    pub(crate) fn finish_mcq_task(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<RequestDmaAccessEnded> {
        self.ops.finish_mcq_task(queue_id, task_tag)
    }

    pub(crate) fn claim_mcq_sqe_nullification(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<()> {
        self.ops
            .claim_mcq_sqe_nullification(queue_id, task_tag)
    }

    pub(crate) fn finish_mcq_sqe_nullification(
        &self,
        queue_id: u32,
        task_tag: TaskTag,
    ) -> Result<RequestDmaAccessEnded> {
        self.ops
            .finish_mcq_sqe_nullification(queue_id, task_tag)
    }

    pub(crate) fn supports_mcq_logical_reset(&self) -> bool {
        self.ops.supports_mcq_logical_reset()
    }

    pub(crate) fn claim_mcq_logical_unit(
        &self,
        targets: &mut [McqLogicalResetTarget],
    ) -> Result<()> {
        self.ops.claim_mcq_logical_unit(targets)
    }

    pub(crate) fn begin_mcq_logical_unit_cleanup(
        &self,
        targets: &[McqLogicalResetTarget],
        lun: u8,
    ) -> Result<()> {
        self.ops.begin_mcq_logical_unit_cleanup(targets, lun)
    }

    pub(crate) fn finish_mcq_logical_unit(
        &self,
        targets: &[McqLogicalResetTarget],
    ) -> Result<McqRequestsDmaAccessEnded> {
        self.ops.finish_mcq_logical_unit(targets)
    }
}
