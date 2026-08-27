// SPDX-License-Identifier: GPL-2.0

use crate::command::{CommandOwner, CommandPool, TaskTag, TASK_TAG_COUNT};
use crate::dma::*;
use crate::lu::{TagSetData, UfsLuBlockOps, UfsRequestData};
use crate::protocol::scsi::*;
use crate::protocol::{query::UfsDevCmd, UfsCmd};
use crate::reg::*;
use crate::resource::HostResources;
use crate::transport::*;
use crate::uic::UfsUic;
use crate::variant::{NotifyPhase, UfsVariantOps};
use kernel::alloc::mempool::MemPool;
use kernel::block::mq;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::block::mq::TagSet;
use kernel::sync::{aref::ARef, Arc, SpinLock};
use kernel::time::{delay::fsleep, Delta};
use kernel::types::OwnableRefCounted;
use kernel::workqueue::{self, impl_has_work, new_work, Work, WorkItem};
use kernel::{bindings, new_spinlock, prelude::*};

const HBA_ENABLE_DELAY_US: i64 = 1000;

fn retryable_check_condition(sense: Option<&ScsiSense>) -> bool {
    matches!(sense, Some(sense) if sense.is_unit_attention())
}

fn should_requeue_scsi(completion: UfsScsiCompletion, sense: Option<&ScsiSense>) -> bool {
    matches!(
        completion,
        UfsScsiCompletion::Busy | UfsScsiCompletion::TaskSetFull | UfsScsiCompletion::Requeue
    ) || (matches!(completion, UfsScsiCompletion::CheckCondition)
        && retryable_check_condition(sense))
}

impl McqConfig {
    fn queue_map(&self) -> Result<UfsQueueMap> {
        UfsQueueMap::new(
            self.total_queues,
            self.default_queues,
            self.read_queues,
            self.poll_queues,
        )
    }
}

impl UfsTransferConfig {
    fn queue_map(&self) -> Result<UfsQueueMap> {
        match self {
            Self::Sdb { .. } => Ok(UfsQueueMap::sdb()),
            Self::Mcq(config) => config.queue_map(),
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct UfsQueueRange {
    offset: usize,
    count: usize,
}

#[derive(Copy, Clone)]
pub(crate) struct UfsQueueMap {
    nr_hw_queues: usize,
    default: UfsQueueRange,
    read: UfsQueueRange,
    poll: UfsQueueRange,
}

impl UfsQueueMap {
    fn sdb() -> Self {
        Self {
            nr_hw_queues: 1,
            default: UfsQueueRange {
                offset: 0,
                count: 1,
            },
            read: UfsQueueRange {
                offset: 0,
                count: 0,
            },
            poll: UfsQueueRange {
                offset: 0,
                count: 1,
            },
        }
    }

    fn new(
        nr_hw_queues: usize,
        default_queues: usize,
        read_queues: usize,
        poll_queues: usize,
    ) -> Result<Self> {
        let mapped_queues = default_queues
            .checked_add(read_queues)
            .and_then(|queues| queues.checked_add(poll_queues))
            .ok_or(EOVERFLOW)?;

        if nr_hw_queues == 0 || mapped_queues != nr_hw_queues {
            return Err(EINVAL);
        }

        let read_offset = default_queues;
        let poll_offset = read_offset.checked_add(read_queues).ok_or(EOVERFLOW)?;

        Ok(Self {
            nr_hw_queues,
            default: UfsQueueRange {
                offset: 0,
                count: default_queues,
            },
            read: UfsQueueRange {
                offset: read_offset,
                count: read_queues,
            },
            poll: UfsQueueRange {
                offset: poll_offset,
                count: poll_queues,
            },
        })
    }

    pub(crate) fn nr_hw_queues(&self) -> usize {
        self.nr_hw_queues
    }

    pub(crate) fn range(&self, kind: mq::QueueType) -> UfsQueueRange {
        match kind {
            mq::QueueType::Default => self.default,
            mq::QueueType::Read => self.read,
            mq::QueueType::Poll => self.poll,
        }
    }

    /// Number of blk-mq queue maps required to express this layout.
    pub(crate) fn num_maps(&self) -> u32 {
        if self.poll.count > 0 {
            3
        } else if self.read.count > 0 {
            2
        } else {
            1
        }
    }
}

impl UfsQueueRange {
    pub(crate) fn offset(&self) -> usize {
        self.offset
    }

    pub(crate) fn count(&self) -> usize {
        self.count
    }
}

enum CompletionTarget<'a> {
    Direct,
    Poll(&'a mut mq::IoCompletionBatch<UfsLuBlockOps>),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum CompletionOutcome {
    Returned,
    RetainedForRecovery,
}

impl CompletionOutcome {
    fn returned(self) -> bool {
        self == Self::Returned
    }
}

pub(crate) struct UfsRequestInner {
    state: UfsRequestState,
}

enum UfsRequestState {
    Idle,
    Prepared {
        cmd: UfsCmd,
        prdt: Option<UfsPrdtMapping>,
    },
    InFlight {
        cmd: UfsCmd,
        prdt: Option<UfsPrdtMapping>,
        queue_id: u32,
    },
    Recovering {
        cmd: UfsCmd,
        prdt: Option<UfsPrdtMapping>,
        queue_id: u32,
    },
    Completing,
    CompletionReady(CompletionDisposition),
    DeviceComplete(UfsDevCmd),
}

pub(crate) enum CompletionDisposition {
    End(u32),
    Requeue,
}

#[derive(Clone, Copy, Debug)]
enum RecoveryReason {
    Driver(&'static str),
    Uic(UicErrorStatus),
    InvalidMcqCompletion,
}

impl RecoveryReason {
    fn name(&self) -> &'static str {
        match *self {
            Self::Driver(reason) => reason,
            Self::Uic(_) => "fatal UIC error",
            Self::InvalidMcqCompletion => "invalid MCQ completion descriptor",
        }
    }

    fn uic_errors(&self) -> Option<UicErrorStatus> {
        match *self {
            Self::Uic(errors) => Some(errors),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum RecoveryScope {
    Controller,
    Queue(u32),
}

impl RecoveryScope {
    fn queue_id(&self) -> Option<u32> {
        match *self {
            Self::Controller => None,
            Self::Queue(queue_id) => Some(queue_id),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct RecoveryCause {
    reason: RecoveryReason,
    scope: RecoveryScope,
    tag: usize,
}

enum RecoveryState {
    Operational,
    Requested(RecoveryCause),
    Quiescing(RecoveryCause),
    Recovering(RecoveryCause),
    Failed(RecoveryCause),
    Shutdown,
}

impl RecoveryState {
    fn cause(&self) -> Option<RecoveryCause> {
        match *self {
            Self::Operational => None,
            Self::Requested(cause)
            | Self::Quiescing(cause)
            | Self::Recovering(cause)
            | Self::Failed(cause) => Some(cause),
            Self::Shutdown => None,
        }
    }
}

enum TimeoutDisposition {
    StartRecovery(UfsCmd),
    Recovering(UfsCmd),
    Pending(UfsCmd),
    Completed,
}

impl Default for UfsRequestInner {
    fn default() -> Self {
        UfsRequestInner {
            state: UfsRequestState::Idle,
        }
    }
}

impl UfsRequestInner {
    pub(crate) fn prepare_device(&mut self, cmd: UfsCmd) -> Result<()> {
        if !matches!(cmd, UfsCmd::Device(_)) || !matches!(self.state, UfsRequestState::Idle) {
            return Err(EINVAL);
        }
        self.state = UfsRequestState::Prepared { cmd, prdt: None };
        Ok(())
    }

    fn prepare_scsi(&mut self, cmd: UfsSCSICmd, prdt: Option<UfsPrdtMapping>) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Idle) {
            return Err(EBUSY);
        }
        self.state = UfsRequestState::Prepared {
            cmd: UfsCmd::SCSI(cmd),
            prdt,
        };
        Ok(())
    }

    fn prepared_command(&self) -> Result<UfsCmd> {
        match self.state {
            UfsRequestState::Prepared { cmd, .. } => Ok(cmd),
            _ => Err(EIO),
        }
    }

    fn mark_in_flight(&mut self, queue_id: u32) -> Result<()> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        self.state = match state {
            UfsRequestState::Prepared { cmd, prdt } => UfsRequestState::InFlight {
                cmd,
                prdt,
                queue_id,
            },
            state => {
                self.state = state;
                return Err(EIO);
            }
        };
        Ok(())
    }

    fn begin_completion(&mut self, queue_id: u32) -> Result<(UfsCmd, Option<UfsPrdtMapping>)> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::InFlight {
                cmd,
                prdt,
                queue_id: submitted_queue,
            }
            | UfsRequestState::Recovering {
                cmd,
                prdt,
                queue_id: submitted_queue,
            } if submitted_queue == queue_id => {
                self.state = UfsRequestState::Completing;
                Ok((cmd, prdt))
            }
            state => {
                self.state = state;
                Err(EIO)
            }
        }
    }

    fn timeout(&mut self) -> TimeoutDisposition {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::Prepared { cmd, prdt } => {
                self.state = UfsRequestState::Prepared { cmd, prdt };
                TimeoutDisposition::Pending(cmd)
            }
            UfsRequestState::InFlight {
                cmd,
                prdt,
                queue_id,
            } => {
                self.state = UfsRequestState::Recovering {
                    cmd,
                    prdt,
                    queue_id,
                };
                TimeoutDisposition::StartRecovery(cmd)
            }
            UfsRequestState::Recovering {
                cmd,
                prdt,
                queue_id,
            } => {
                self.state = UfsRequestState::Recovering {
                    cmd,
                    prdt,
                    queue_id,
                };
                TimeoutDisposition::Recovering(cmd)
            }
            state => {
                self.state = state;
                TimeoutDisposition::Completed
            }
        }
    }

    fn complete_device(&mut self, cmd: UfsDevCmd) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Completing) {
            return Err(EIO);
        }
        self.state = UfsRequestState::DeviceComplete(cmd);
        Ok(())
    }

    fn schedule_completion(&mut self, disposition: CompletionDisposition) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Completing) {
            return Err(EIO);
        }
        self.state = UfsRequestState::CompletionReady(disposition);
        Ok(())
    }

    pub(crate) fn take_scheduled_completion(&mut self) -> Result<CompletionDisposition> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::CompletionReady(disposition) => Ok(disposition),
            state => {
                self.state = state;
                Err(EIO)
            }
        }
    }

    fn finish_direct_completion(&mut self) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Completing) {
            return Err(EIO);
        }
        self.state = UfsRequestState::Idle;
        Ok(())
    }

    pub(crate) fn take_device_completion(&mut self) -> Result<UfsCmd> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::DeviceComplete(cmd) => Ok(UfsCmd::Device(cmd)),
            state => {
                self.state = state;
                Err(EIO)
            }
        }
    }

    pub(crate) fn reset(&mut self) {
        self.state = UfsRequestState::Idle;
    }

    fn prepare_recovery_disposition(&mut self, requeue: bool) -> Result<()> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::InFlight {
                cmd,
                prdt,
                queue_id: _,
            }
            | UfsRequestState::Recovering {
                cmd,
                prdt,
                queue_id: _,
            } => {
                drop(prdt);
                if requeue && matches!(cmd, UfsCmd::Device(_)) {
                    self.state = UfsRequestState::Prepared { cmd, prdt: None };
                }
                Ok(())
            }
            state => {
                self.state = state;
                Err(EIO)
            }
        }
    }
}

struct ResolvedCompletion {
    rq: ARef<mq::Request<UfsLuBlockOps>>,
    task_tag: TaskTag,
    queue_id: u32,
    completion: TransferCompletion,
}

impl ResolvedCompletion {
    fn complete(self) -> CompletionOutcome {
        UfsRequestData::complete(self.rq, self.task_tag, self.queue_id, self.completion)
    }

    fn complete_polled(
        self,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> CompletionOutcome {
        UfsRequestData::complete_polled(
            self.rq,
            self.task_tag,
            self.queue_id,
            self.completion,
            batch,
        )
    }
}

impl UfsRequestData {
    fn task_tag(rq: &mq::Request<UfsLuBlockOps>) -> Result<TaskTag> {
        TaskTag::new(rq.dispatch_budget().ok_or(EIO)?)
    }

    pub(crate) fn compose_dev_request(
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        hw_queue: &UfsHwQueue,
    ) -> Result<()> {
        if let Some(queue) = rq.queue_data().dev_queue() {
            let cmd = rq
                .data_ref()
                .inner
                .lock()
                .prepared_command()?
                .get_device()?;
            let task_tag = Self::task_tag(rq)?;
            queue.compose_dev(cmd, task_tag)?;
            queue.bind_command(task_tag, hw_queue.id(), rq.tag())?;
            Ok(())
        } else {
            Err(EIO)
        }
    }

    pub(crate) fn compose_scsi_cmd(
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
        hw_queue: &UfsHwQueue,
    ) -> Result<()> {
        let mempool = rq.queue().tag_set().data().dma_vec_mempool.clone();
        let queue = rq.queue_data().queue();
        let task_tag = Self::task_tag(rq)?;
        let prdt = UfsQueue::compose_scsi(rq, cmd, task_tag, &mempool)?;

        rq.data_ref().inner.lock().prepare_scsi(cmd, prdt)?;
        queue.bind_command(task_tag, hw_queue.id(), rq.tag())?;
        Ok(())
    }

    pub(crate) fn submit(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        hw_queue: &UfsHwQueue,
    ) -> core::result::Result<(), (ARef<mq::Request<UfsLuBlockOps>>, Error)> {
        let queue = rq.queue_data().queue_arc().clone();
        let queue_id = hw_queue.id();
        let task_tag = match Self::task_tag(&rq) {
            Ok(task_tag) => task_tag,
            Err(e) => return Err((rq, e)),
        };
        let polled = rq.flags().contains(mq::RequestFlag::Polled);

        if queue.recovery_required() {
            return Err((rq, EBUSY));
        }

        if rq.queue_index() != queue_id {
            return Err((rq, EINVAL));
        }

        let mut rq = Some(rq);
        let outcome = hw_queue.submit(u32::from(task_tag.value()), polled, || {
            let request = rq.as_ref().ok_or(EIO)?;
            request.data_ref().inner.lock().mark_in_flight(queue_id)?;

            // Drop the submit-side reference at the publish boundary. From
            // this point hardware may complete the command immediately and
            // completion must be able to recover unique request ownership.
            drop(rq.take());
            Ok(())
        });

        match outcome {
            SubmissionOutcome::Submitted => Ok(()),
            SubmissionOutcome::NotSubmitted(e) => {
                let Some(rq) = rq else {
                    queue.require_recovery("invalid submission ownership", task_tag.index());
                    return Ok(());
                };
                Err((rq, e))
            }
            SubmissionOutcome::PublishFailed(e) => {
                pr_err!(
                    "[RUFS] ufs_queue: submission publish failed tag={} queue={} errno={}\n",
                    task_tag.value(),
                    queue_id,
                    e.to_errno(),
                );
                queue.require_recovery("submission publish failed", task_tag.index());
                Ok(())
            }
        }
    }

    pub(crate) fn timeout(rq: &mq::Request<UfsLuBlockOps>, tag: u32) -> bool {
        let queue = rq.queue_data().queue_arc().clone();
        let disposition = rq.data_ref().inner.lock().timeout();
        let cmd = match disposition {
            TimeoutDisposition::StartRecovery(cmd) => {
                queue.require_recovery("request timeout", tag as usize);
                Some(cmd)
            }
            TimeoutDisposition::Recovering(cmd) => Some(cmd),
            TimeoutDisposition::Pending(cmd) => Some(cmd),
            TimeoutDisposition::Completed => return true,
        };

        if let Some(UfsCmd::SCSI(cmd)) = cmd {
            let cdb = cmd.cdb();
            pr_err!(
                "[RUFS] ufs_queue: SCSI request timeout tag={} lun={} opcode=0x{:02x}\n",
                tag,
                cmd.lun(),
                cdb[0],
            );
        } else {
            pr_err!("[RUFS] ufs_queue: request timeout tag={}\n", tag);
        }
        // Do not release the tag until recovery has stopped hardware and
        // prevented a late completion from referring to a reused request.
        false
    }

    fn complete(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        completion: TransferCompletion,
    ) -> CompletionOutcome {
        Self::complete_with(rq, task_tag, queue_id, completion, CompletionTarget::Direct)
    }

    fn complete_polled(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        completion: TransferCompletion,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> CompletionOutcome {
        Self::complete_with(
            rq,
            task_tag,
            queue_id,
            completion,
            CompletionTarget::Poll(batch),
        )
    }

    fn complete_with(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        completion: TransferCompletion,
        target: CompletionTarget<'_>,
    ) -> CompletionOutcome {
        let request_queue = rq.queue_data().queue_arc().clone();
        let (cmd, prdt) = match rq.data_ref().inner.lock().begin_completion(queue_id) {
            Ok(state) => state,
            Err(_) => {
                pr_err!(
                    "[RUFS] ufs_queue: completion for inactive request tag={}\n",
                    rq.tag(),
                );
                request_queue
                    .require_recovery("completion for inactive request", rq.tag() as usize);
                return CompletionOutcome::RetainedForRecovery;
            }
        };

        match cmd {
            UfsCmd::Device(cmd) => {
                let Some(queue) = rq.queue_data().dev_queue() else {
                    pr_err!("[RUFS] ufs_queue: device request has invalid context\n");
                    drop(prdt);
                    let status = u32::from(bindings::BLK_STS_IOERR);
                    rq.data_ref().inner.lock().reset();
                    return Self::end_device_request(rq, request_queue, status);
                };
                let result = queue.fetch_dev(cmd, task_tag, completion);
                drop(prdt);
                let status = match result {
                    Ok(UfsCmd::Device(cmd)) => {
                        if rq.data_ref().inner.lock().complete_device(cmd).is_err() {
                            pr_err!("[RUFS] ufs_queue: invalid device completion state\n");
                            rq.data_ref().inner.lock().reset();
                            u32::from(bindings::BLK_STS_IOERR)
                        } else {
                            u32::from(bindings::BLK_STS_OK)
                        }
                    }
                    _ => {
                        pr_err!("[RUFS] ufs_queue: failed to fetch device response\n");
                        rq.data_ref().inner.lock().reset();
                        u32::from(bindings::BLK_STS_IOERR)
                    }
                };
                Self::end_device_request(rq, request_queue, status)
            }
            UfsCmd::SCSI(cmd) => {
                let Some(lu) = rq.queue_data().logical_unit() else {
                    pr_err!("[RUFS] ufs_queue: SCSI request has invalid context\n");
                    drop(prdt);
                    let status = u32::from(bindings::BLK_STS_IOERR);
                    rq.data_ref().inner.lock().reset();
                    return Self::end_device_request(rq, request_queue, status);
                };
                let queue = &lu.queue;
                let result = queue.fetch_scsi_completion(task_tag, completion);
                drop(prdt);

                queue.clone().complete_scsi(cmd, result, rq, target)
            }
        }
    }

    fn end_device_request(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        queue: Arc<UfsQueue>,
        status: u32,
    ) -> CompletionOutcome {
        let tag = rq.tag();
        rq.release_budget_and_run_queue();
        let rq = match OwnableRefCounted::try_from_shared(rq) {
            Ok(rq) => rq,
            Err(_rq) => {
                queue.require_recovery("device completion ownership conflict", tag as usize);
                return CompletionOutcome::RetainedForRecovery;
            }
        };

        rq.end(u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR as u8));
        CompletionOutcome::Returned
    }
}

#[pin_data]
pub(crate) struct UfsQueue {
    pub(crate) tags: Arc<TagSet<UfsLuBlockOps>>,
    resources: Arc<HostResources>,
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    uic: Arc<UfsUic>,
    backend: UfsTransferBackend,
    #[pin]
    recovery: SpinLock<RecoveryState>,
    #[pin]
    recovery_work: Work<UfsQueue>,
    #[pin]
    command_pool: SpinLock<CommandPool>,
}

impl_has_work! {
    impl HasWork<Self> for UfsQueue { self.recovery_work }
}

impl UfsQueue {
    pub(crate) fn new(
        config: UfsTransferConfig,
        resources: Arc<HostResources>,
        reg: Arc<UfsReg>,
        dma: Arc<UfsDma>,
        uic: Arc<UfsUic>,
    ) -> Result<Arc<Self>> {
        let backend = UfsTransferBackend::new(config, reg.clone(), dma.clone())?;
        let hw_queues = backend.hw_queues()?;
        let queue_map = config.queue_map()?;
        let nr_hw_queues = queue_map.nr_hw_queues();
        let task_tag_count = config.tag_count();
        let software_queue_depth = config.software_queue_depth();
        let max_active_commands = config.max_active_commands();
        if task_tag_count == 0
            || software_queue_depth == 0
            || max_active_commands == 0
            || nr_hw_queues == 0
            || hw_queues.len() != nr_hw_queues
        {
            return Err(EINVAL);
        }

        let tagset_data = KBox::new(
            TagSetData {
                queue_map,
                hw_queues,
                // Every active hardware command may retain one detached DMA mapping
                // until completion. Reserve enough vector storage to make
                // that mapping lifetime independent of atomic allocation
                // success under memory pressure.
                dma_vec_mempool: MemPool::new(max_active_commands)?,
            },
            GFP_KERNEL,
        )?;

        let tagset = Arc::pin_init(
            TagSet::<UfsLuBlockOps>::new(
                nr_hw_queues as u32,
                tagset_data,
                u32::try_from(software_queue_depth).map_err(|_| EOVERFLOW)?,
                queue_map.num_maps(),
                kernel::alloc::NumaNode::NO_NODE,
                kernel::block::mq::tag_set::Flags::default(),
            ),
            GFP_KERNEL,
        )?;

        let queue = Arc::pin_init(
            try_pin_init!(Self {
                tags <- tagset,
                resources,
                reg,
                dma,
                uic,
                backend,
                recovery <- new_spinlock!(RecoveryState::Operational),
                recovery_work <- new_work!("UfsQueue::recovery"),
                command_pool <- new_spinlock!(CommandPool::new(
                    task_tag_count,
                    max_active_commands,
                )?),
            }),
            GFP_KERNEL,
        )?;

        Ok(queue)
    }

    pub(crate) fn try_get_budget(&self) -> Option<u32> {
        let mut command_pool = self.command_pool.lock();
        command_pool
            .reserve()
            .map(|task_tag| u32::from(task_tag.value()))
    }

    pub(crate) fn begin_shutdown(&self) {
        *self.recovery.lock() = RecoveryState::Shutdown;
        self.tags.quiesce();
    }

    pub(crate) fn active_commands(&self) -> usize {
        self.command_pool.lock().active()
    }

    pub(crate) fn flush_recovery_work(self: &Arc<Self>) {
        // SAFETY: `UfsQueue` was initialized in an `Arc` with `pin_init!`.
        // The allocation remains stable while this `Arc` is alive, so its
        // structurally pinned recovery work cannot move.
        let work = unsafe { Pin::new_unchecked(&self.recovery_work) };
        work.flush();
    }

    fn bind_command(&self, task_tag: TaskTag, queue_id: u32, blk_tag: u32) -> Result<()> {
        self.command_pool
            .lock()
            .bind(task_tag, CommandOwner { queue_id, blk_tag })
    }

    fn command_owner(&self, task_tag: TaskTag) -> Result<CommandOwner> {
        self.command_pool.lock().owner(task_tag).ok_or(EIO)
    }

    pub(crate) fn put_budget(&self, token: u32) -> bool {
        let Ok(task_tag) = TaskTag::new(token) else {
            pr_warn!("[RUFS] ufs_queue: invalid budget token={}\n", token);
            return false;
        };
        if self.command_pool.lock().release(task_tag).is_err() {
            pr_warn!(
                "[RUFS] ufs_queue: invalid command slot release task_tag={}\n",
                task_tag.value(),
            );
            return false;
        }
        true
    }

    fn completion_pass_limit(&self) -> usize {
        let active = self.command_pool.lock().active();
        core::cmp::max(1, active.div_ceil(CompletedRequests::capacity()))
    }

    fn recovery_required(&self) -> bool {
        self.recovery.lock().cause().is_some()
    }

    fn stop_controller(&self) -> Result<()> {
        self.reg.disable_interrupts();
        self.reg.clear_all_interrupts();
        self.reg.disable_run_stop();
        if self.reg.ctrl_enabled() {
            self.reg.ctrl_disable();
            self.reg.wait_for_ctrl_disable(10, 10)?;
        }
        Ok(())
    }

    fn reset_controller(&self) -> Result<()> {
        self.stop_controller()?;

        let variant = self.resources.variant();
        variant.device_reset()?;
        variant.hce_enable_notify(&self.reg, NotifyPhase::Pre)?;
        self.reg.ctrl_enable();
        fsleep(Delta::from_micros(HBA_ENABLE_DELAY_US));
        self.reg.wait_for_ctrl_enable(1000, 50)?;
        variant.hce_enable_notify(&self.reg, NotifyPhase::Post)?;

        variant.link_startup_notify(&self.reg, &self.uic, NotifyPhase::Pre)?;
        self.uic.link_startup()?;
        variant.link_startup_notify(&self.reg, &self.uic, NotifyPhase::Post)?;

        // Restore the common UTRL/UTMRL state before backend-specific state.
        // In particular, MCQ activation programs queue registers and enables
        // MCQ interrupts on top of an operational controller, matching the
        // order used during initial host bring-up.
        self.dma.make_hba_operational()?;
        self.backend.reset()?;

        if let Err(e) = self.restore_power_mode(variant) {
            pr_warn!(
                "[RUFS] ufs_queue: recovery power mode restore failed errno={}, continue\n",
                e.to_errno(),
            );
        }
        Ok(())
    }

    fn restore_power_mode(&self, variant: &dyn UfsVariantOps) -> Result<()> {
        let mode = variant.constrain_power_mode(self.uic.max_power_mode()?)?;
        variant.power_mode_notify(&self.reg, &self.uic, mode, NotifyPhase::Pre)?;
        self.uic.change_power_mode(mode)?;
        variant.power_mode_notify(&self.reg, &self.uic, mode, NotifyPhase::Post)
    }

    fn dispose_recovery_request(
        &self,
        owner: CommandOwner,
        requeue: bool,
    ) -> Result<()> {
        let rq = self
            .tags
            .try_tag_to_rq(owner.queue_id, owner.blk_tag)?
            .ok_or(EIO)?;
        rq.data_ref()
            .inner
            .lock()
            .prepare_recovery_disposition(requeue)?;
        let rq = OwnableRefCounted::try_from_shared(rq).map_err(|_| EBUSY)?;

        if requeue {
            rq.requeue(true);
        } else {
            rq.end(bindings::BLK_STS_IOERR as u8);
        }
        Ok(())
    }

    fn dispose_recovery_requests(&self, requeue: bool) -> Result<usize> {
        let mut disposed = 0;
        let mut failed = false;

        for tag in 0..TASK_TAG_COUNT {
            let task_tag = TaskTag::from_index(tag)?;
            let owner = match self.command_pool.lock().recovery_owner(task_tag) {
                Ok(Some(owner)) => owner,
                Ok(None) => continue,
                Err(e) => {
                    pr_err!(
                        "[RUFS] ufs_queue: recovery found unbound command tag={} errno={}\n",
                        tag,
                        e.to_errno(),
                    );
                    failed = true;
                    continue;
                }
            };
            if let Err(e) = self.dispose_recovery_request(owner, requeue) {
                pr_err!(
                    "[RUFS] ufs_queue: recovery request disposition failed tag={} errno={}\n",
                    tag,
                    e.to_errno(),
                );
                failed = true;
            } else {
                disposed += 1;
            }
        }

        if failed || self.active_commands() != 0 {
            Err(EIO)
        } else {
            Ok(disposed)
        }
    }

    pub(crate) fn require_recovery(self: &Arc<Self>, reason: &'static str, tag: usize) {
        self.request_recovery(RecoveryCause {
            reason: RecoveryReason::Driver(reason),
            scope: RecoveryScope::Controller,
            tag,
        });
    }

    pub(crate) fn require_uic_recovery(self: &Arc<Self>, errors: UicErrorStatus) {
        self.request_recovery(RecoveryCause {
            reason: RecoveryReason::Uic(errors),
            scope: RecoveryScope::Controller,
            tag: 0,
        });
    }

    fn require_mcq_recovery(self: &Arc<Self>, queue_id: u32, tag: usize) {
        self.request_recovery(RecoveryCause {
            reason: RecoveryReason::InvalidMcqCompletion,
            scope: RecoveryScope::Queue(queue_id),
            tag,
        });
    }

    fn request_recovery(self: &Arc<Self>, cause: RecoveryCause) {
        let schedule = {
            let mut state = self.recovery.lock();
            if matches!(*state, RecoveryState::Operational) {
                *state = RecoveryState::Requested(cause);
                true
            } else {
                false
            }
        };

        if schedule {
            pr_err!(
                "[RUFS] ufs_queue: recovery required reason={} queue={:?} tag={}\n",
                cause.reason.name(),
                cause.scope.queue_id(),
                cause.tag,
            );
            let _ = workqueue::system().enqueue(self.clone());
        }
    }

    // Issuing
    pub(crate) fn compose_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag) -> Result<()> {
        self.dma
            .compose_devman_upiu(cmd, u32::from(task_tag.value()))
    }

    fn compose_scsi(
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
        task_tag: TaskTag,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        let queue = rq.queue_data().queue();

        queue
            .dma
            .compose_scsi_upiu(rq, cmd, task_tag.value(), mempool)
    }

    fn fetch_dev(
        &self,
        cmd: UfsDevCmd,
        task_tag: TaskTag,
        completion: TransferCompletion,
    ) -> Result<UfsCmd> {
        match completion {
            TransferCompletion::Sdb => self.dma.fetch_devman_upiu(cmd, task_tag.index()),
            TransferCompletion::Mcq(cqe) => {
                self.dma.fetch_mcq_devman_upiu(cmd, task_tag.index(), cqe)
            }
        }
    }

    fn fetch_scsi_completion(
        &self,
        task_tag: TaskTag,
        completion: TransferCompletion,
    ) -> UfsScsiResult {
        match completion {
            TransferCompletion::Sdb => self.dma.fetch_scsi_completion(task_tag.index()),
            TransferCompletion::Mcq(cqe) => {
                self.dma.fetch_mcq_scsi_completion(task_tag.index(), cqe)
            }
        }
    }

    fn collect_backend_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        self.backend.collect_completions(completed)
    }

    fn dump_backend_state(&self, tag: usize, reason: &str) {
        self.backend.dump_state(tag, reason);
    }

    fn request_at_task_tag(
        &self,
        task_tag: TaskTag,
    ) -> Result<(CommandOwner, Option<ARef<mq::Request<UfsLuBlockOps>>>)> {
        let owner = self.command_owner(task_tag)?;
        Ok((
            owner,
            self.tags.try_tag_to_rq(owner.queue_id, owner.blk_tag)?,
        ))
    }

    fn resolve_completion(
        self: &Arc<Self>,
        request: CompletedRequest,
    ) -> Option<ResolvedCompletion> {
        let task_tag = request.task_tag;
        match self.request_at_task_tag(task_tag) {
            Ok((owner, Some(rq))) => {
                if owner.queue_id != request.queue_id {
                    self.require_recovery("completion queue mismatch", task_tag.index());
                    return None;
                }
                Some(ResolvedCompletion {
                    rq,
                    task_tag,
                    queue_id: request.queue_id,
                    completion: request.completion,
                })
            }
            Ok((_, None)) => {
                self.require_recovery("completion tag has no request", task_tag.index());
                None
            }
            Err(_) => {
                self.require_recovery("completion request is not shareable", task_tag.index());
                None
            }
        }
    }

    pub(crate) fn complete(self: &Arc<Self>) -> bool {
        if self.recovery_required() {
            return false;
        }
        // Completion is tag-driven: the backend collects completed tags, then
        // the queue finalizes exactly those requests. Finalization still runs
        // from the threaded IRQ path because it takes request, backend, and DMA
        // locks that are shared with submission and hands requests back to
        // blk-mq. Once those lock domains are IRQ-safe, this path can move into
        // hard IRQ context.
        let mut any_completed = false;
        for _ in 0..self.completion_pass_limit() {
            let mut requests = CompletedRequests::new();
            let collect_result = self.collect_backend_completions(&mut requests);

            let batch_full = requests.is_full();
            while let Some(request) = requests.take_next() {
                if let Some(completion) = self.resolve_completion(request) {
                    any_completed |= completion.complete().returned();
                }
            }

            if let Some(fault) = requests.take_fault() {
                if let Some(queue_id) = fault.queue_id {
                    self.require_mcq_recovery(queue_id, fault.tag);
                } else {
                    self.require_recovery(fault.reason, fault.tag);
                }
                return any_completed;
            }
            if let Err(e) = collect_result {
                pr_err!(
                    "[RUFS] ufs_queue: collect completions failed errno={}\n",
                    e.to_errno(),
                );
                self.dump_backend_state(0, "collect completions failed");
                self.require_recovery("completion collection failed", 0);
                return any_completed;
            }
            if !batch_full {
                break;
            }
        }

        any_completed
    }

    pub(crate) fn poll(
        self: &Arc<Self>,
        hw_queue: &UfsHwQueue,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> bool {
        if self.recovery_required() {
            return false;
        }
        let mut any_completed = false;
        for _ in 0..self.completion_pass_limit() {
            let mut requests = CompletedRequests::new();
            let poll_result = hw_queue.poll(&mut requests);

            let batch_full = requests.is_full();
            while let Some(request) = requests.take_next() {
                if let Some(completion) = self.resolve_completion(request) {
                    any_completed |= completion.complete_polled(batch).returned();
                }
            }

            if let Some(fault) = requests.take_fault() {
                if let Some(queue_id) = fault.queue_id {
                    self.require_mcq_recovery(queue_id, fault.tag);
                } else {
                    self.require_recovery(fault.reason, fault.tag);
                }
                return any_completed;
            }
            if let Err(e) = poll_result {
                pr_err!(
                    "[RUFS] ufs_queue: poll queue {} failed errno={}\n",
                    hw_queue.id(),
                    e.to_errno(),
                );
                self.require_recovery("polled completion collection failed", hw_queue.id() as _);
                return any_completed;
            }
            if !batch_full {
                break;
            }
        }

        any_completed
    }

    fn complete_scsi(
        self: &Arc<Self>,
        cmd: UfsSCSICmd,
        result: UfsScsiResult,
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        target: CompletionTarget<'_>,
    ) -> CompletionOutcome {
        let tag = rq.tag();
        let sense_len = result.sense_data_len.min(result.sense_data.len());
        let sense = parse_scsi_sense(&result.sense_data, sense_len);
        let suppress_log = matches!(result.completion, UfsScsiCompletion::CheckCondition)
            && matches!(sense.as_ref(), Some(sense) if sense.is_power_on_reset());
        let requeue = should_requeue_scsi(result.completion, sense.as_ref());

        if !matches!(result.completion, UfsScsiCompletion::Good) && !suppress_log {
            let cdb = cmd.cdb();
            pr_err!(
                "[RUFS] ufs_queue: SCSI request completion error: tag={} lun={} \
                 opcode=0x{:02x} dir={:?} data_len={} completion={:?} ocs=0x{:x} \
                 transaction=0x{:02x} response=0x{:02x} status=0x{:02x} residual={} \
                 cdb={:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} \
                 {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}\n",
                tag,
                cmd.lun(),
                cdb[0],
                cmd.direction(),
                cmd.data_len(),
                result.completion,
                result.ocs,
                result.transaction,
                result.response,
                result.status,
                result.residual_transfer_count,
                cdb[0],
                cdb[1],
                cdb[2],
                cdb[3],
                cdb[4],
                cdb[5],
                cdb[6],
                cdb[7],
                cdb[8],
                cdb[9],
                cdb[10],
                cdb[11],
                cdb[12],
                cdb[13],
                cdb[14],
                cdb[15],
            );

            if let Some(sense) = sense.as_ref() {
                pr_err!(
                    "[RUFS] ufs_queue: SCSI sense tag={} response_code=0x{:02x} \
                     sense_key=0x{:x}({}) asc=0x{:02x} ascq=0x{:02x} \
                     additional_len={}\n",
                    tag,
                    sense.response_code,
                    sense.sense_key,
                    sense_key_name(sense.sense_key),
                    sense.asc,
                    sense.ascq,
                    sense.additional_len,
                );
            } else if sense_len > 0 {
                pr_err!(
                    "[RUFS] ufs_queue: SCSI sense tag={} unable to parse \
                     sense_len={} raw={:02x} {:02x} {:02x} {:02x} {:02x} \
                     {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} \
                     {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}\n",
                    tag,
                    sense_len,
                    result.sense_data[0],
                    result.sense_data[1],
                    result.sense_data[2],
                    result.sense_data[3],
                    result.sense_data[4],
                    result.sense_data[5],
                    result.sense_data[6],
                    result.sense_data[7],
                    result.sense_data[8],
                    result.sense_data[9],
                    result.sense_data[10],
                    result.sense_data[11],
                    result.sense_data[12],
                    result.sense_data[13],
                    result.sense_data[14],
                    result.sense_data[15],
                    result.sense_data[16],
                    result.sense_data[17],
                );
            } else {
                pr_err!(
                    "[RUFS] ufs_queue: SCSI sense tag={} no sense data reported\n",
                    tag,
                );
            }
        }

        let status = match result.completion {
            UfsScsiCompletion::Good => bindings::BLK_STS_OK,
            UfsScsiCompletion::Busy
            | UfsScsiCompletion::TaskSetFull
            | UfsScsiCompletion::Requeue => bindings::BLK_STS_RESOURCE,
            UfsScsiCompletion::TaskAborted => bindings::BLK_STS_TARGET,
            UfsScsiCompletion::ReservationConflict => bindings::BLK_STS_RESV_CONFLICT,
            UfsScsiCompletion::CheckCondition => {
                if retryable_check_condition(sense.as_ref()) {
                    bindings::BLK_STS_RESOURCE
                } else {
                    bindings::BLK_STS_IOERR
                }
            }
            UfsScsiCompletion::Error => bindings::BLK_STS_IOERR,
        };

        let status = status as u32;
        let disposition = if requeue {
            CompletionDisposition::Requeue
        } else {
            CompletionDisposition::End(status)
        };
        match target {
            CompletionTarget::Direct => {
                if rq
                    .data_ref()
                    .inner
                    .lock()
                    .schedule_completion(disposition)
                    .is_err()
                {
                    self.require_recovery("invalid SCSI completion state", tag as usize);
                    return CompletionOutcome::RetainedForRecovery;
                }
                rq.release_budget_and_run_queue();
                mq::Request::complete(rq);
                CompletionOutcome::Returned
            }
            CompletionTarget::Poll(batch) => {
                let rq = match OwnableRefCounted::try_from_shared(rq) {
                    Ok(rq) => rq,
                    Err(_rq) => {
                        self.require_recovery("polled completion ownership conflict", tag as usize);
                        return CompletionOutcome::RetainedForRecovery;
                    }
                };
                if rq
                    .data_ref()
                    .inner
                    .lock()
                    .finish_direct_completion()
                    .is_err()
                {
                    self.require_recovery("invalid polled completion state", tag as usize);
                    return CompletionOutcome::RetainedForRecovery;
                }
                rq.release_budget_and_run_queue();

                if requeue {
                    rq.requeue(true);
                    return CompletionOutcome::Returned;
                }
                if status != u32::from(bindings::BLK_STS_OK) {
                    rq.end(u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR as u8));
                    return CompletionOutcome::Returned;
                }

                if let Err(rq) = batch.add_request(rq, false) {
                    rq.end(status as u8);
                }
                CompletionOutcome::Returned
            }
        }
    }
}

impl WorkItem for UfsQueue {
    type Pointer = Arc<Self>;

    fn run(this: Arc<Self>) {
        let cause = {
            let mut state = this.recovery.lock();
            let RecoveryState::Requested(cause) = *state else {
                return;
            };
            *state = RecoveryState::Quiescing(cause);
            cause
        };

        this.tags.quiesce();

        {
            let mut state = this.recovery.lock();
            if matches!(*state, RecoveryState::Quiescing(_)) {
                *state = RecoveryState::Recovering(cause);
            } else {
                return;
            }
        }

        pr_info!(
            "[RUFS] ufs_queue: controller recovery started reason={} queue={:?} tag={}\n",
            cause.reason.name(),
            cause.scope.queue_id(),
            cause.tag,
        );
        if let Some(errors) = cause.reason.uic_errors() {
            pr_err!(
                "[RUFS] ufs_queue: recovery UIC status phy=0x{:08x} dl=0x{:08x} nl=0x{:08x} tl=0x{:08x} dme=0x{:08x}\n",
                errors.phy,
                errors.data_link,
                errors.network,
                errors.transport,
                errors.dme,
            );
        }
        // A controller reset makes every command that was visible to the old
        // controller instance unreachable. Only after that boundary may RUFS
        // return their blk-mq tags: requeue them when the new link is usable,
        // or finish them with I/O error when reset failed.
        let reset = this.reset_controller();
        let requeue = reset.is_ok();
        if reset.is_err() {
            let _ = this.stop_controller();
        }
        let disposition = this.dispose_recovery_requests(requeue);

        match (reset, disposition) {
            (Ok(()), Ok(disposed)) => {
                let resume = {
                    let mut state = this.recovery.lock();
                    if matches!(*state, RecoveryState::Recovering(_)) {
                        *state = RecoveryState::Operational;
                        true
                    } else {
                        false
                    }
                };
                if resume {
                    this.tags.unquiesce();
                    pr_info!(
                        "[RUFS] ufs_queue: controller recovery completed requeued={}\n",
                        disposed,
                    );
                }
            }
            (reset, disposition) => {
                if let Err(e) = reset {
                    pr_err!(
                        "[RUFS] ufs_queue: controller recovery reset failed errno={}\n",
                        e.to_errno(),
                    );
                }
                if let Err(e) = disposition {
                    pr_err!(
                        "[RUFS] ufs_queue: recovery request cleanup failed errno={}\n",
                        e.to_errno(),
                    );
                }
                let mut state = this.recovery.lock();
                if matches!(*state, RecoveryState::Recovering(_)) {
                    *state = RecoveryState::Failed(cause);
                }
            }
        }
    }
}
