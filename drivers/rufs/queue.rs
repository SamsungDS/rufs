// SPDX-License-Identifier: GPL-2.0

use crate::command::TaskTag;
use crate::dma::*;
use crate::host::{
    RecoveryCause, RecoveryOutcome, RecoveryReason, UfsHostShared,
};
use crate::lu::{TagSetData, UfsLuBlockOps, UfsRequestData};
use crate::protocol::scsi::*;
use crate::protocol::task_management::TaskManagementServiceResponse;
use crate::protocol::{query::UfsDevCmd, UfsCmd};
use crate::reg::*;
use crate::transport::*;
use kernel::alloc::mempool::MemPool;
use kernel::block::mq;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::block::mq::TagSet;
use kernel::sync::{Arc, MutexGuard};
use kernel::time::{delay::fsleep, Delta};
use kernel::types::{OwnableRefCounted, Owned};
use kernel::{bindings, prelude::*};

const RECOVERY_OWNERSHIP_RETRIES: usize = 100;
const RECOVERY_OWNERSHIP_RETRY_DELAY_US: i64 = 100;
const TASK_QUERY_RETRIES: usize = 100;
const TASK_QUERY_RETRY_DELAY_US: i64 = 100;

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
    Interrupt,
    Poll(&'a mut mq::IoCompletionBatch<UfsLuBlockOps>),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum CompletionOutcome {
    Released,
    RetainedForRecovery,
}

impl CompletionOutcome {
    fn released(self) -> bool {
        self == Self::Released
    }
}

pub(crate) struct UfsRequestInner {
    state: UfsRequestState,
}

struct PreparedIo {
    cmd: UfsCmd,
    mapping: UfsPreparedMapping,
}

impl PreparedIo {
    fn publish(self, queue_id: u32) -> ActiveIo {
        ActiveIo {
            cmd: self.cmd,
            mapping: self.mapping.publish(),
            queue_id,
        }
    }
}

struct ActiveIo {
    cmd: UfsCmd,
    mapping: UfsActiveMapping,
    queue_id: u32,
}

impl ActiveIo {
    fn begin_completion(
        self,
        queue_id: u32,
    ) -> core::result::Result<CompletingIo, ActiveIo> {
        if self.queue_id != queue_id {
            return Err(self);
        }

        Ok(CompletingIo {
            cmd: self.cmd,
            mapping: self.mapping,
        })
    }

    fn retire_after_stop(self, _proof: &DmaAccessEnded) -> UfsCmd {
        // SAFETY: `DmaAccessEnded` is created only after the controller has
        // invalidated every command that could refer to this mapping.
        unsafe { self.mapping.complete() };
        self.cmd
    }

    fn retire_after_request_stop(self, _proof: RequestDmaAccessEnded) -> UfsCmd {
        // SAFETY: `RequestDmaAccessEnded` is created only after the matching
        // request has been removed from completion collection and the backend
        // has proved that hardware can no longer access its DMA mapping.
        unsafe { self.mapping.complete() };
        self.cmd
    }

}

struct CompletingIo {
    cmd: UfsCmd,
    mapping: UfsActiveMapping,
}

impl CompletingIo {
    fn retire(self) -> UfsCmd {
        // SAFETY: `CompletingIo` is created only after the transfer backend has
        // destructively consumed the hardware completion for this command.
        unsafe { self.mapping.complete() };
        self.cmd
    }
}

enum UfsRequestState {
    Idle,
    Prepared(PreparedIo),
    InFlight(ActiveIo),
    TimedOut(ActiveIo),
    Completing,
    CompletionReady(CompletionDisposition),
    DeviceComplete(UfsDevCmd),
}

pub(crate) enum CompletionDisposition {
    End(u32),
    Requeue,
}

enum RecoveryPlan {
    Resume,
    Task(TaskTag),
    LogicalUnit(u8),
    Controller,
}

enum TaskRecoveryResult {
    Recovered,
    LogicalUnit,
    Controller,
}

enum LogicalUnitRecoveryResult {
    Recovered,
    Reevaluate,
    Controller,
}

/// Proof that the controller can no longer access previously published DMA
/// mappings.
pub(crate) struct DmaAccessEnded;

pub(crate) enum ControllerResetResult {
    Restored(DmaAccessEnded),
    Stopped {
        dma_ended: DmaAccessEnded,
        error: Error,
    },
    UnsafeStop {
        reset_error: Option<Error>,
        stop_error: Error,
    },
}

enum TimeoutDisposition {
    StartRecovery(UfsCmd),
    TimedOut(UfsCmd),
    Pending(UfsCmd),
    NoAction,
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
        self.state = UfsRequestState::Prepared(PreparedIo {
            cmd,
            mapping: UfsPreparedMapping::None,
        });
        Ok(())
    }

    fn prepare_scsi(&mut self, cmd: UfsSCSICmd, mapping: UfsPreparedMapping) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Idle) {
            return Err(EBUSY);
        }
        self.state = UfsRequestState::Prepared(PreparedIo {
            cmd: UfsCmd::Scsi(cmd),
            mapping,
        });
        Ok(())
    }

    fn prepared_command(&self) -> Result<UfsCmd> {
        match &self.state {
            UfsRequestState::Prepared(io) => Ok(io.cmd),
            _ => Err(EIO),
        }
    }

    fn mark_in_flight(&mut self, queue_id: u32) -> Result<()> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        self.state = match state {
            UfsRequestState::Prepared(io) => UfsRequestState::InFlight(io.publish(queue_id)),
            state => {
                self.state = state;
                return Err(EIO);
            }
        };
        Ok(())
    }

    fn begin_completion(&mut self, queue_id: u32) -> Result<CompletingIo> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::InFlight(io) => match io.begin_completion(queue_id) {
                Ok(io) => {
                    self.state = UfsRequestState::Completing;
                    Ok(io)
                }
                Err(io) => {
                    self.state = UfsRequestState::InFlight(io);
                    Err(EIO)
                }
            },
            UfsRequestState::TimedOut(io) => match io.begin_completion(queue_id) {
                Ok(io) => {
                    self.state = UfsRequestState::Completing;
                    Ok(io)
                }
                Err(io) => {
                    self.state = UfsRequestState::TimedOut(io);
                    Err(EIO)
                }
            },
            state => {
                self.state = state;
                Err(EIO)
            }
        }
    }

    fn timeout(&mut self) -> TimeoutDisposition {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::Prepared(io) => {
                let cmd = io.cmd;
                self.state = UfsRequestState::Prepared(io);
                TimeoutDisposition::Pending(cmd)
            }
            UfsRequestState::InFlight(io) => {
                let cmd = io.cmd;
                self.state = UfsRequestState::TimedOut(io);
                TimeoutDisposition::StartRecovery(cmd)
            }
            UfsRequestState::TimedOut(io) => {
                let cmd = io.cmd;
                self.state = UfsRequestState::TimedOut(io);
                TimeoutDisposition::TimedOut(cmd)
            }
            state => {
                self.state = state;
                TimeoutDisposition::NoAction
            }
        }
    }

    fn take_for_recovery(&mut self) -> Result<(ActiveIo, bool)> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::InFlight(io) => Ok((io, false)),
            UfsRequestState::TimedOut(io) => Ok((io, true)),
            state => {
                self.state = state;
                Err(EBUSY)
            }
        }
    }

    fn restore_after_recovery(
        &mut self,
        io: ActiveIo,
    ) -> core::result::Result<(), ActiveIo> {
        if !matches!(self.state, UfsRequestState::Idle) {
            return Err(io);
        }
        self.state = UfsRequestState::InFlight(io);
        Ok(())
    }

    fn prepare_recovery_return(&mut self, cmd: UfsCmd, requeue: bool) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Idle) {
            return Err(EBUSY);
        }
        if requeue && matches!(cmd, UfsCmd::Device(_)) {
            self.state = UfsRequestState::Prepared(PreparedIo {
                cmd,
                mapping: UfsPreparedMapping::None,
            });
        }
        Ok(())
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

    pub(crate) fn prepare_unsubmitted_return(&mut self, requeue: bool) -> Result<()> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::Idle => Ok(()),
            UfsRequestState::Prepared(io)
                if requeue
                    && matches!(io.cmd, UfsCmd::Device(_))
                    && matches!(&io.mapping, UfsPreparedMapping::None) =>
            {
                self.state = UfsRequestState::Prepared(io);
                Ok(())
            }
            UfsRequestState::Prepared(_) => Ok(()),
            state => {
                self.state = state;
                Err(EBUSY)
            }
        }
    }

    pub(crate) fn clear_dma_free(&mut self) -> Result<()> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::Idle
            | UfsRequestState::Completing
            | UfsRequestState::CompletionReady(_)
            | UfsRequestState::DeviceComplete(_) => Ok(()),
            state => {
                self.state = state;
                Err(EBUSY)
            }
        }
    }

    fn returnable(&self, requeue: bool) -> bool {
        match &self.state {
            UfsRequestState::Idle => true,
            UfsRequestState::DeviceComplete(_) => !requeue,
            UfsRequestState::Prepared(io) if requeue => {
                matches!(io.cmd, UfsCmd::Device(_))
                    && matches!(&io.mapping, UfsPreparedMapping::None)
            }
            _ => false,
        }
    }

}

struct ResolvedCompletion {
    rq: Owned<mq::Request<UfsLuBlockOps>>,
    task_tag: TaskTag,
    queue_id: u32,
    payload: CompletionPayload,
}

pub(crate) struct RecoveryRequest {
    request: Owned<mq::Request<UfsLuBlockOps>>,
    io: ActiveIo,
    state: RecoveryRequestState,
}

enum RecoveryRequestState {
    /// Recovery owns the request, but an ordinary completion may still win.
    Pending {
        timed_out: bool,
        completion: Option<BackendCompletion>,
    },
    /// The backend has excluded this task from ordinary completion handling.
    TaskRecovery,
    /// The backend has claimed this request as part of a logical-unit reset.
    LogicalUnitRecovery,
}

impl RecoveryRequest {
    fn new(
        request: Owned<mq::Request<UfsLuBlockOps>>,
        io: ActiveIo,
        timed_out: bool,
    ) -> Self {
        Self {
            request,
            io,
            state: RecoveryRequestState::Pending {
                timed_out,
                completion: None,
            },
        }
    }

    fn tag(&self) -> u32 {
        self.request.tag()
    }

    fn queue_index(&self) -> u32 {
        self.request.queue_index()
    }

    fn unresolved_timeout(&self) -> Option<UfsCmd> {
        match &self.state {
            RecoveryRequestState::Pending {
                timed_out: true,
                completion: None,
            } => Some(self.io.cmd),
            _ => None,
        }
    }

    fn store_completion(&mut self, completion: BackendCompletion) -> Result<()> {
        let RecoveryRequestState::Pending {
            completion: pending,
            ..
        } = &mut self.state
        else {
            return Err(EIO);
        };
        if self.io.queue_id != completion.queue_id() || pending.is_some() {
            return Err(EIO);
        }
        *pending = Some(completion);
        Ok(())
    }

    fn completion(&self) -> Option<&BackendCompletion> {
        match &self.state {
            RecoveryRequestState::Pending { completion, .. } => completion.as_ref(),
            _ => None,
        }
    }

    fn has_completion(&self) -> bool {
        self.completion().is_some()
    }

    fn validate_task_recovery(&self) -> Result<()> {
        match &self.state {
            RecoveryRequestState::Pending {
                timed_out: true,
                completion: None,
            } => Ok(()),
            _ => Err(EINVAL),
        }
    }

    fn mark_task_recovery(&mut self) -> Result<()> {
        self.validate_task_recovery()?;
        self.state = RecoveryRequestState::TaskRecovery;
        Ok(())
    }

    fn task_recovery_claimed(&self) -> bool {
        matches!(&self.state, RecoveryRequestState::TaskRecovery)
    }

    fn validate_logical_unit_recovery(&self, lun: u8) -> Result<()> {
        if !matches!(self.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
            return Err(EINVAL);
        }
        match &self.state {
            RecoveryRequestState::Pending {
                completion: None, ..
            }
            | RecoveryRequestState::TaskRecovery => Ok(()),
            RecoveryRequestState::Pending {
                completion: Some(_),
                ..
            } => Err(EAGAIN),
            _ => Err(EINVAL),
        }
    }

    fn mark_logical_unit_recovery(&mut self, lun: u8) -> Result<()> {
        self.validate_logical_unit_recovery(lun)?;
        self.state = RecoveryRequestState::LogicalUnitRecovery;
        Ok(())
    }

    fn restore(self) -> core::result::Result<(), Self> {
        if !matches!(
            &self.state,
            RecoveryRequestState::Pending {
                completion: None,
                ..
            }
        ) {
            return Err(self);
        }
        let Self {
            request,
            io,
            state,
        } = self;
        let result = request.data_ref().inner.lock().restore_after_recovery(io);
        match result {
            Ok(()) => {
                drop(request);
                Ok(())
            }
            Err(io) => Err(Self {
                request,
                io,
                state,
            }),
        }
    }

    fn into_completion(
        self,
    ) -> core::result::Result<
        (Owned<mq::Request<UfsLuBlockOps>>, BackendCompletion),
        Self,
    > {
        let Self {
            request,
            io,
            state,
        } = self;
        let (timed_out, completion) = match state {
            RecoveryRequestState::Pending {
                timed_out,
                completion: Some(completion),
            } => (timed_out, completion),
            state => return Err(Self { request, io, state }),
        };
        let restore = {
            let mut state = request.data_ref().inner.lock();
            state.restore_after_recovery(io)
        };
        match restore {
            Ok(()) => Ok((request, completion)),
            Err(io) => Err(Self {
                request,
                io,
                state: RecoveryRequestState::Pending {
                    timed_out,
                    completion: Some(completion),
                },
            }),
        }
    }

    fn requeue_after_request_stop(
        self,
        proof: RequestDmaAccessEnded,
        expected_lun: u8,
    ) -> core::result::Result<(), (Self, RequestDmaAccessEnded)> {
        let matching_lun = matches!(self.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == expected_lun);
        if self.request.tag() != u32::from(proof.task_tag().value())
            || !matching_lun
        {
            return Err((self, proof));
        }

        let Self { request, io, .. } = self;
        let _cmd = io.retire_after_request_stop(proof);
        request.requeue(true);
        Ok(())
    }

    fn requeue_after_task_abort(
        self,
        proof: RequestDmaAccessEnded,
    ) -> core::result::Result<(), (Self, RequestDmaAccessEnded)> {
        let UfsCmd::Scsi(cmd) = self.io.cmd else {
            return Err((self, proof));
        };
        if !matches!(&self.state, RecoveryRequestState::TaskRecovery) {
            return Err((self, proof));
        }
        self.requeue_after_request_stop(proof, cmd.lun())
    }

    fn requeue_after_logical_reset(
        self,
        lun: u8,
        proof: RequestDmaAccessEnded,
    ) -> core::result::Result<(), (Self, RequestDmaAccessEnded)> {
        if !matches!(&self.state, RecoveryRequestState::LogicalUnitRecovery) {
            return Err((self, proof));
        }
        self.requeue_after_request_stop(proof, lun)
    }
}

impl ResolvedCompletion {
    fn complete(self) -> CompletionOutcome {
        UfsRequestData::complete(self.rq, self.task_tag, self.queue_id, self.payload)
    }

    fn complete_polled(
        self,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> CompletionOutcome {
        UfsRequestData::complete_polled(
            self.rq,
            self.task_tag,
            self.queue_id,
            self.payload,
            batch,
        )
    }
}

impl UfsRequestData {
    fn task_tag(rq: &mq::Request<UfsLuBlockOps>) -> Result<TaskTag> {
        TaskTag::new(rq.tag())
    }

    pub(crate) fn try_end_request(
        rq: Owned<mq::Request<UfsLuBlockOps>>,
        status: u8,
    ) -> core::result::Result<(), Owned<mq::Request<UfsLuBlockOps>>> {
        if !rq.data_ref().inner.lock().returnable(false) {
            return Err(rq);
        }
        rq.end(status);
        Ok(())
    }

    pub(crate) fn try_requeue_request(
        rq: Owned<mq::Request<UfsLuBlockOps>>,
    ) -> core::result::Result<(), Owned<mq::Request<UfsLuBlockOps>>> {
        if !rq.data_ref().inner.lock().returnable(true) {
            return Err(rq);
        }
        rq.requeue(true);
        Ok(())
    }

    fn returnable_for_batch(rq: &mq::Request<UfsLuBlockOps>) -> bool {
        rq.data_ref().inner.lock().returnable(false)
    }

    pub(crate) fn compose_dev_request(rq: &Owned<mq::Request<UfsLuBlockOps>>) -> Result<()> {
        if let Some(queue) = rq.queue_data().dev_queue() {
            let cmd = rq
                .data_ref()
                .inner
                .lock()
                .prepared_command()?
                .get_device()?;
            let task_tag = Self::task_tag(rq)?;
            queue.compose_dev(cmd, task_tag)?;
            Ok(())
        } else {
            Err(EIO)
        }
    }

    pub(crate) fn compose_scsi_cmd(
        rq: &Owned<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
    ) -> Result<()> {
        let mempool = rq.queue().tag_set().data().dma_vec_mempool.clone();
        let task_tag = Self::task_tag(rq)?;
        let mapping = UfsQueue::compose_scsi(rq, cmd, task_tag, &mempool)?;

        rq.data_ref().inner.lock().prepare_scsi(cmd, mapping)?;
        Ok(())
    }

    pub(crate) fn submit(
        rq: Owned<mq::Request<UfsLuBlockOps>>,
        hw_queue: &UfsHwQueue,
    ) -> core::result::Result<(), (Owned<mq::Request<UfsLuBlockOps>>, Error)> {
        let host = rq.queue_data().host().clone();
        let queue_id = hw_queue.id();
        let task_tag = match Self::task_tag(&rq) {
            Ok(task_tag) => task_tag,
            Err(e) => return Err((rq, e)),
        };
        let polled = rq.flags().contains(mq::RequestFlag::Polled);

        if host.recovery_required() {
            return Err((rq, EBUSY));
        }

        if rq.queue_index() != queue_id {
            return Err((rq, EINVAL));
        }

        let mut rq = Some(rq);
        let outcome = hw_queue.submit(u32::from(task_tag.value()), polled, || {
            let request = rq.as_mut().ok_or(EIO)?;
            request.data_ref().inner.lock().mark_in_flight(queue_id)?;

            // Park unique ownership in the blk-mq tag map at the publication
            // boundary. No submit-side request reference remains live when
            // hardware can observe the task tag.
            let request = rq.take().ok_or(EIO)?;
            drop(request);
            Ok(())
        });

        match outcome {
            SubmissionOutcome::Submitted => Ok(()),
            SubmissionOutcome::NotSubmitted(e) => {
                let Some(rq) = rq else {
                    host.require_recovery("invalid submission ownership", task_tag.index());
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
                host.require_recovery("submission publish failed", task_tag.index());
                Ok(())
            }
        }
    }

    pub(crate) fn timeout(
        rq: &mq::Request<UfsLuBlockOps>,
        tag: u32,
    ) -> Option<Arc<UfsHostShared>> {
        let host = rq.queue_data().host().clone();
        let disposition = rq.data_ref().inner.lock().timeout();
        let (cmd, start_recovery) = match disposition {
            TimeoutDisposition::StartRecovery(cmd) => (Some(cmd), true),
            TimeoutDisposition::TimedOut(cmd) => (Some(cmd), false),
            TimeoutDisposition::Pending(cmd) => (Some(cmd), false),
            TimeoutDisposition::NoAction => return None,
        };

        if let Some(UfsCmd::Scsi(cmd)) = cmd {
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
        start_recovery.then_some(host)
    }

    fn complete(
        rq: Owned<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        payload: CompletionPayload,
    ) -> CompletionOutcome {
        Self::complete_with(rq, task_tag, queue_id, payload, CompletionTarget::Interrupt)
    }

    fn complete_polled(
        rq: Owned<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        payload: CompletionPayload,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> CompletionOutcome {
        Self::complete_with(
            rq,
            task_tag,
            queue_id,
            payload,
            CompletionTarget::Poll(batch),
        )
    }

    fn complete_with(
        rq: Owned<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        payload: CompletionPayload,
        target: CompletionTarget<'_>,
    ) -> CompletionOutcome {
        let request_queue = rq.queue_data().queue_arc().clone();
        let host = rq.queue_data().host().clone();
        let completion_state = {
            let mut request = rq.data_ref().inner.lock();
            request.begin_completion(queue_id)
        };
        let completing = match completion_state {
            Ok(state) => state,
            Err(_) => {
                pr_err!(
                    "[RUFS] ufs_queue: completion for inactive request tag={}\n",
                    rq.tag(),
                );
                host.require_recovery("completion for inactive request", rq.tag() as usize);
                // Keep the request owned by the Rust blk-mq abstraction until
                // recovery decides its final disposition.
                drop(rq);
                return CompletionOutcome::RetainedForRecovery;
            }
        };
        let cmd = completing.retire();

        match cmd {
            UfsCmd::Device(cmd) => {
                let Some(queue) = rq.queue_data().dev_queue() else {
                    pr_err!("[RUFS] ufs_queue: device request has invalid context\n");
                    return Self::fail_dma_free_completion(
                        rq,
                        &request_queue,
                        "device request has invalid context",
                    );
                };
                let result = queue.fetch_dev(cmd, task_tag, payload);
                match result {
                    Ok(UfsCmd::Device(cmd)) => {
                        if rq.data_ref().inner.lock().complete_device(cmd).is_err() {
                            pr_err!("[RUFS] ufs_queue: invalid device completion state\n");
                            Self::fail_dma_free_completion(
                                rq,
                                &request_queue,
                                "invalid device completion state",
                            )
                        } else {
                            Self::end_request(rq, u32::from(bindings::BLK_STS_OK))
                        }
                    }
                    _ => {
                        pr_err!("[RUFS] ufs_queue: failed to fetch device response\n");
                        Self::fail_dma_free_completion(
                            rq,
                            &request_queue,
                            "failed to fetch device response",
                        )
                    }
                }
            }
            UfsCmd::Scsi(cmd) => {
                let Some(lu) = rq.queue_data().logical_unit() else {
                    pr_err!("[RUFS] ufs_queue: SCSI request has invalid context\n");
                    return Self::fail_dma_free_completion(
                        rq,
                        &request_queue,
                        "SCSI request has invalid context",
                    );
                };
                let queue = &lu.queue;
                let result = queue.fetch_scsi_completion(task_tag, payload);

                queue.clone().complete_scsi(cmd, result, rq, target)
            }
        }
    }

    fn fail_dma_free_completion(
        rq: Owned<mq::Request<UfsLuBlockOps>>,
        _queue: &Arc<UfsQueue>,
        reason: &'static str,
    ) -> CompletionOutcome {
        let tag = rq.tag() as usize;
        rq.queue_data().host().require_recovery(reason, tag);
        if rq.data_ref().inner.lock().clear_dma_free().is_err() {
            drop(rq);
            return CompletionOutcome::RetainedForRecovery;
        }

        Self::end_request(rq, u32::from(bindings::BLK_STS_IOERR))
    }

    fn end_request(rq: Owned<mq::Request<UfsLuBlockOps>>, status: u32) -> CompletionOutcome {
        let status = u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR);
        match Self::try_end_request(rq, status) {
            Ok(()) => CompletionOutcome::Released,
            Err(rq) => {
                let host = rq.queue_data().host().clone();
                let tag = rq.tag() as usize;
                host.require_recovery("request returned with active DMA", tag);
                drop(rq);
                CompletionOutcome::RetainedForRecovery
            }
        }
    }
}

#[pin_data]
pub(crate) struct UfsQueue {
    pub(crate) tags: Arc<TagSet<UfsLuBlockOps>>,
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    backend: UfsTransferBackend,
}

impl UfsQueue {
    pub(crate) fn new(
        config: UfsTransferConfig,
        reg: Arc<UfsReg>,
        dma: Arc<UfsDma>,
    ) -> Result<Arc<Self>> {
        let backend = UfsTransferBackend::new(config, reg.clone(), dma.clone())?;
        let hw_queues = backend.hw_queues()?;
        let queue_map = config.queue_map()?;
        let nr_hw_queues = queue_map.nr_hw_queues();
        let queue_depth = config.queue_depth();
        if queue_depth == 0 || nr_hw_queues == 0 || hw_queues.len() != nr_hw_queues {
            return Err(EINVAL);
        }
        let tagset_data = KBox::new(
            TagSetData {
                queue_map,
                hw_queues,
                // One reserved vector array guarantees forward progress under
                // memory pressure. Additional allocation failures are returned
                // to blk-mq as resource shortages and retried.
                dma_vec_mempool: MemPool::new(1)?,
            },
            GFP_KERNEL,
        )?;

        let mut tagset_flags = kernel::block::mq::tag_set::Flags::default();
        tagset_flags |= kernel::block::mq::tag_set::Flag::TagHctxShared;
        let tagset = Arc::pin_init(
            TagSet::<UfsLuBlockOps>::new(
                nr_hw_queues as u32,
                tagset_data,
                u32::try_from(queue_depth).map_err(|_| EOVERFLOW)?,
                queue_map.num_maps(),
                kernel::alloc::NumaNode::NO_NODE,
                tagset_flags,
            ),
            GFP_KERNEL,
        )?;

        let queue = Arc::pin_init(
            try_pin_init!(Self {
                tags <- tagset,
                reg,
                dma,
                backend,
            }),
            GFP_KERNEL,
        )?;

        Ok(queue)
    }

    pub(crate) fn busy_requests(&self) -> usize {
        self.tags.busy_request_count()
    }

    pub(crate) fn wait_completed_requests(&self) {
        self.tags.wait_completed_requests();
    }

    pub(crate) fn enable_interrupts(&self) {
        self.backend.enable_interrupts();
    }

    pub(crate) fn reset_backend(&self) -> Result<()> {
        self.backend.reset()
    }

    fn completion_pass_limit(&self) -> usize {
        let queue_depth = self.tags.queue_depth() as usize;
        core::cmp::max(1, queue_depth.div_ceil(CompletionBatch::capacity()))
    }

    fn claim_recovery_requests<'a>(
        &self,
        host: &'a UfsHostShared,
    ) -> Result<MutexGuard<'a, KVec<RecoveryRequest>>> {
        self.tags.wait_completed_requests();
        let mut requests = host.recovery_requests.lock();
        if !requests.is_empty() {
            return Err(EBUSY);
        }

        for tag in 0..self.tags.queue_depth() {
            let task_tag = TaskTag::new(tag)?;
            let mut request = Err(EAGAIN);
            for attempt in 0..RECOVERY_OWNERSHIP_RETRIES {
                match self.try_claim_recovery_request(task_tag) {
                    Err(EAGAIN) if attempt + 1 < RECOVERY_OWNERSHIP_RETRIES => {
                        // A timeout or scheduled-completion callback may still
                        // hold a transient ARef after the tag set is quiesced.
                        // Wait for that shared reference to reach the unique
                        // recovery ownership boundary.
                        fsleep(Delta::from_micros(
                            RECOVERY_OWNERSHIP_RETRY_DELAY_US,
                        ));
                    }
                    result => {
                        request = result;
                        break;
                    }
                }
            }

            match request {
                Ok(Some(rq)) => {
                    let recovery_io = {
                        let mut request = rq.data_ref().inner.lock();
                        request.take_for_recovery()
                    };
                    let (io, timed_out) = match recovery_io {
                        Ok(recovery_io) => recovery_io,
                        Err(e) => {
                            pr_err!(
                                "rufs: recovery request claim failed tag={} errno={}\n",
                                tag,
                                e.to_errno(),
                            );
                            drop(rq);
                            return Err(e);
                        }
                    };
                    requests
                        .push_within_capacity(RecoveryRequest::new(rq, io, timed_out))
                        .map_err(|_| EOVERFLOW)?;
                }
                Ok(None) => {
                    // Quiescing waits for queue_rq callbacks. An exclusively
                    // owned tagged request therefore has not crossed RUFS's
                    // hardware publication boundary and is not recoverable
                    // hardware state.
                    continue;
                }
                Err(e) => {
                    pr_err!(
                        "rufs: recovery request ownership failed tag={} errno={}\n",
                        tag,
                        e.to_errno(),
                    );
                    return Err(e);
                }
            }
        }

        Ok(requests)
    }

    fn store_recovery_completion(
        self: &Arc<Self>,
        requests: &mut KVec<RecoveryRequest>,
        completion: BackendCompletion,
    ) -> Result<()> {
        let task_tag = completion.task_tag();
        let queue_id = completion.queue_id();
        let Some(rq) = requests
            .iter_mut()
            .find(|rq| rq.tag() == u32::from(task_tag.value()))
        else {
            if let Some(request) = requests.get(0) {
                request.request.queue_data().host().require_recovery(
                    "completion tag has no held request",
                    task_tag.index(),
                );
            }
            return Err(EIO);
        };

        if rq.queue_index() != queue_id {
            rq.request.queue_data().host().require_mcq_recovery(
                "held completion queue mismatch",
                queue_id,
                task_tag.index(),
            );
            return Err(EIO);
        }

        let result = rq.store_completion(completion);
        if result.is_err() {
            rq.request.queue_data().host().require_recovery(
                "duplicate or invalid held completion",
                task_tag.index(),
            );
        }
        result
    }

    fn collect_held_completions(
        self: &Arc<Self>,
        requests: &mut KVec<RecoveryRequest>,
    ) {
        for _ in 0..self.completion_pass_limit() {
            let mut completions = CompletionBatch::new();
            let collect_result = self.collect_recovery_completions(&mut completions);
            let batch_full = completions.is_full();

            while let Some(completion) = completions.take_next() {
                let _ = self.store_recovery_completion(requests, completion);
            }

            if let Some(fault) = completions.take_fault() {
                if let Some(request) = requests.get(0) {
                    let host = request.request.queue_data().host();
                    if let Some(queue_id) = fault.queue_id {
                        host.require_mcq_recovery(fault.reason, queue_id, fault.tag);
                    } else {
                        host.require_recovery(fault.reason, fault.tag);
                    }
                }
            }
            if let Err(e) = collect_result {
                pr_err!(
                    "rufs: recovery completion collection failed errno={}\n",
                    e.to_errno(),
                );
                if let Some(request) = requests.get(0) {
                    request
                        .request
                        .queue_data()
                        .host()
                        .require_recovery("recovery completion collection failed", 0);
                }
                return;
            }
            if !batch_full {
                return;
            }
        }
    }

    fn recovery_request_index(
        requests: &KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> Option<usize> {
        requests
            .iter()
            .position(|rq| rq.tag() == u32::from(task_tag.value()))
    }

    fn recovery_request_lun(
        requests: &KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> Result<u8> {
        let index = Self::recovery_request_index(requests, task_tag).ok_or(ENOENT)?;
        match requests[index].io.cmd {
            UfsCmd::Scsi(cmd) => Ok(cmd.lun()),
            _ => Err(EINVAL),
        }
    }

    fn recovery_request_queue_id(
        requests: &KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> Result<u32> {
        let index = Self::recovery_request_index(requests, task_tag).ok_or(ENOENT)?;
        Ok(requests[index].queue_index())
    }

    fn complete_recovery_request(
        self: &Arc<Self>,
        requests: &mut KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> Result<bool> {
        let index = Self::recovery_request_index(requests, task_tag).ok_or(ENOENT)?;
        let Some(completion) = requests[index].completion() else {
            return Ok(false);
        };
        if completion.task_tag() != task_tag
            || completion.queue_id() != requests[index].queue_index()
        {
            return Err(EIO);
        }
        let recovery = requests.remove(index).map_err(|_| EIO)?;
        let (rq, completion) = match recovery.into_completion() {
            Ok(completion) => completion,
            Err(recovery) => {
                requests
                    .push_within_capacity(recovery)
                    .map_err(|_| EOVERFLOW)?;
                return Err(EIO);
            }
        };
        let (completion_tag, queue_id, payload) = completion.into_parts();
        if completion_tag != task_tag || rq.queue_index() != queue_id {
            return Err(EIO);
        }

        if !UfsRequestData::complete(rq, task_tag, queue_id, payload).released() {
            return Err(EIO);
        }
        self.tags.wait_completed_requests();
        Ok(true)
    }

    fn complete_recovery_completions(
        self: &Arc<Self>,
        requests: &mut KVec<RecoveryRequest>,
    ) -> Result<usize> {
        let mut completed = 0usize;

        loop {
            let Some(request) = requests.iter().find(|request| request.has_completion())
            else {
                return Ok(completed);
            };
            let task_tag = TaskTag::new(request.tag())?;
            if !self.complete_recovery_request(requests, task_tag)? {
                return Err(EIO);
            }
            completed += 1;
        }
    }

    fn requeue_task_aborted_request(
        &self,
        requests: &mut KVec<RecoveryRequest>,
        proof: RequestDmaAccessEnded,
    ) -> Result<()> {
        let task_tag = proof.task_tag();
        let index = Self::recovery_request_index(requests, task_tag).ok_or(ENOENT)?;
        let recovery = requests.remove(index).map_err(|_| EIO)?;
        if let Err((recovery, _proof)) = recovery.requeue_after_task_abort(proof) {
            requests
                .push_within_capacity(recovery)
                .map_err(|_| EOVERFLOW)?;
            return Err(EIO);
        }
        Ok(())
    }

    fn mark_task_recovery(
        requests: &mut KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> Result<()> {
        let index = Self::recovery_request_index(requests, task_tag).ok_or(ENOENT)?;
        requests[index].mark_task_recovery()
    }

    fn sdb_logical_unit_task_masks(
        requests: &KVec<RecoveryRequest>,
        lun: u8,
    ) -> Result<(u32, u32)> {
        let mut task_mask = 0u32;
        let mut task_claimed_mask = 0u32;
        for request in requests.iter() {
            if !matches!(request.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                continue;
            }
            request.validate_logical_unit_recovery(lun)?;
            let task_tag = TaskTag::new(request.tag())?;
            let mask = 1u32
                .checked_shl(u32::from(task_tag.value()))
                .ok_or(EINVAL)?;
            task_mask |= mask;
            if request.task_recovery_claimed() {
                task_claimed_mask |= mask;
            }
        }
        if task_mask == 0 {
            return Err(ENOENT);
        }
        Ok((task_mask, task_claimed_mask))
    }

    fn mark_logical_unit_recovery(
        requests: &mut KVec<RecoveryRequest>,
        lun: u8,
    ) -> Result<()> {
        for request in requests.iter() {
            if matches!(request.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                request.validate_logical_unit_recovery(lun)?;
            }
        }
        for request in requests.iter_mut() {
            if matches!(request.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                request.mark_logical_unit_recovery(lun)?;
            }
        }
        Ok(())
    }

    fn requeue_logical_unit_requests(
        &self,
        requests: &mut KVec<RecoveryRequest>,
        lun: u8,
        mut proof: SdbRequestsDmaAccessEnded,
    ) -> Result<usize> {
        let mut requeued = 0usize;
        let mut index = 0usize;
        let mut expected = 0usize;

        // Validate the complete LU/proof correspondence before returning any
        // blk-mq request. Recovery must not partially publish a disposition
        // because of a software identity mismatch.
        for request in requests.iter() {
            if !matches!(request.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                continue;
            }
            let task_tag = TaskTag::new(request.tag())?;
            if request.has_completion() || !proof.contains(task_tag) {
                return Err(EIO);
            }
            expected += 1;
        }
        if proof.len() != expected {
            return Err(EIO);
        }

        while index < requests.len() {
            if !matches!(requests[index].io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                index += 1;
                continue;
            }
            let task_tag = TaskTag::new(requests[index].tag())?;
            let request_proof = proof.take(task_tag)?;
            let recovery = requests.remove(index).map_err(|_| EIO)?;
            if let Err((recovery, _proof)) =
                recovery.requeue_after_logical_reset(lun, request_proof)
            {
                requests
                    .push_within_capacity(recovery)
                    .map_err(|_| EOVERFLOW)?;
                return Err(EIO);
            }
            requeued += 1;
        }

        if !proof.is_empty() {
            return Err(EIO);
        }
        Ok(requeued)
    }

    fn mcq_logical_unit_targets(
        requests: &KVec<RecoveryRequest>,
        lun: u8,
    ) -> Result<KVec<McqLogicalResetTarget>> {
        let mut targets = KVec::with_capacity(requests.len(), GFP_KERNEL)?;
        for request in requests.iter() {
            if !matches!(request.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                continue;
            }
            request.validate_logical_unit_recovery(lun)?;
            targets
                .push_within_capacity(McqLogicalResetTarget::new(
                    request.queue_index(),
                    TaskTag::new(request.tag())?,
                    request.task_recovery_claimed(),
                ))
                .map_err(|_| EOVERFLOW)?;
        }
        if targets.is_empty() {
            return Err(ENOENT);
        }
        Ok(targets)
    }

    fn requeue_mcq_logical_unit_requests(
        &self,
        requests: &mut KVec<RecoveryRequest>,
        lun: u8,
        mut proof: McqRequestsDmaAccessEnded,
    ) -> Result<usize> {
        let mut requeued = 0usize;
        let mut index = 0usize;
        let mut expected = 0usize;

        for request in requests.iter() {
            if !matches!(request.io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                continue;
            }
            let task_tag = TaskTag::new(request.tag())?;
            if request.has_completion() || !proof.contains(task_tag) {
                return Err(EIO);
            }
            expected += 1;
        }
        if proof.len() != expected {
            return Err(EIO);
        }

        while index < requests.len() {
            if !matches!(requests[index].io.cmd, UfsCmd::Scsi(cmd) if cmd.lun() == lun) {
                index += 1;
                continue;
            }
            let task_tag = TaskTag::new(requests[index].tag())?;
            let request_proof = proof.take(task_tag)?;
            let recovery = requests.remove(index).map_err(|_| EIO)?;
            if let Err((recovery, _proof)) =
                recovery.requeue_after_logical_reset(lun, request_proof)
            {
                requests
                    .push_within_capacity(recovery)
                    .map_err(|_| EOVERFLOW)?;
                return Err(EIO);
            }
            requeued += 1;
        }

        if !proof.is_empty() {
            return Err(EIO);
        }
        Ok(requeued)
    }

    fn wait_for_mcq_request_retirement<F, T>(
        self: &Arc<Self>,
        requests: &mut KVec<RecoveryRequest>,
        mut finish: F,
    ) -> Result<T>
    where
        F: FnMut() -> Result<T>,
    {
        const RETRIES: usize = 500;
        const RETRY_DELAY_US: i64 = 1000;

        for attempt in 0..RETRIES {
            self.collect_held_completions(requests);
            match finish() {
                Ok(proof) => return Ok(proof),
                Err(EAGAIN) if attempt + 1 < RETRIES => {
                    fsleep(Delta::from_micros(RETRY_DELAY_US));
                }
                Err(EAGAIN) => return Err(ETIMEDOUT),
                Err(error) => return Err(error),
            }
        }

        Err(ETIMEDOUT)
    }

    fn recover_sdb_task(
        self: &Arc<Self>,
        host: &UfsHostShared,
        requests: &mut KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> TaskRecoveryResult {
        if !self.backend.supports_sdb_task_abort() {
            return TaskRecoveryResult::Controller;
        }
        let lun = match Self::recovery_request_lun(requests, task_tag) {
            Ok(lun) => lun,
            Err(_) => return TaskRecoveryResult::Controller,
        };

        for attempt in 0..TASK_QUERY_RETRIES {
            match host.task_management.query_task(lun, task_tag) {
                Ok(TaskManagementServiceResponse::FunctionComplete) => {
                    // The device no longer has the task, but its transfer
                    // completion may still be moving through the host
                    // controller. Reconcile it before querying again.
                    self.collect_held_completions(requests);
                    match self.complete_recovery_request(requests, task_tag) {
                        Ok(true) => {
                            pr_info!(
                                "[RUFS] ufs_queue: timed-out SDB task completed tag={}\n",
                                task_tag.value(),
                            );
                            return TaskRecoveryResult::Recovered;
                        }
                        Ok(false) => {}
                        Err(_) => return TaskRecoveryResult::Controller,
                    }
                }
                Ok(TaskManagementServiceResponse::FunctionSucceeded) => {
                    let index = match Self::recovery_request_index(requests, task_tag) {
                        Some(index) => index,
                        None => return TaskRecoveryResult::Controller,
                    };
                    if requests[index].validate_task_recovery().is_err() {
                        return TaskRecoveryResult::Controller;
                    }
                    match self.backend.claim_sdb_task(task_tag) {
                        Ok(()) => {}
                        Err(EAGAIN) => {
                            // The transfer completed before the irrevocable
                            // abort decision. Its completion is still valid.
                            self.collect_held_completions(requests);
                            continue;
                        }
                        Err(_) => return TaskRecoveryResult::Controller,
                    }
                    if Self::mark_task_recovery(requests, task_tag).is_err() {
                        return TaskRecoveryResult::Controller;
                    }

                    match host.task_management.abort_task(lun, task_tag) {
                        Ok(TaskManagementServiceResponse::FunctionComplete) => {}
                        Ok(_) => return TaskRecoveryResult::LogicalUnit,
                        Err(_) => return TaskRecoveryResult::Controller,
                    }
                    let proof = match self.backend.finish_sdb_task(task_tag) {
                        Ok(proof) => proof,
                        Err(_) => return TaskRecoveryResult::Controller,
                    };
                    if self
                        .requeue_task_aborted_request(requests, proof)
                        .is_err()
                    {
                        return TaskRecoveryResult::Controller;
                    }
                    pr_info!(
                        "[RUFS] ufs_queue: SDB task aborted and requeued tag={}\n",
                        task_tag.value(),
                    );
                    return TaskRecoveryResult::Recovered;
                }
                Ok(_) => return TaskRecoveryResult::LogicalUnit,
                Err(_) => return TaskRecoveryResult::Controller,
            }

            if attempt + 1 < TASK_QUERY_RETRIES {
                fsleep(Delta::from_micros(TASK_QUERY_RETRY_DELAY_US));
            }
        }

        TaskRecoveryResult::LogicalUnit
    }

    fn recover_mcq_task(
        self: &Arc<Self>,
        host: &UfsHostShared,
        requests: &mut KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> TaskRecoveryResult {
        if !self.backend.supports_mcq_task_abort() {
            return TaskRecoveryResult::Controller;
        }
        let lun = match Self::recovery_request_lun(requests, task_tag) {
            Ok(lun) => lun,
            Err(_) => return TaskRecoveryResult::Controller,
        };
        let queue_id = match Self::recovery_request_queue_id(requests, task_tag) {
            Ok(queue_id) => queue_id,
            Err(_) => return TaskRecoveryResult::Controller,
        };

        for attempt in 0..TASK_QUERY_RETRIES {
            match host.task_management.query_task(lun, task_tag) {
                Ok(TaskManagementServiceResponse::FunctionComplete) => {
                    self.collect_held_completions(requests);
                    match self.complete_recovery_request(requests, task_tag) {
                        Ok(true) => {
                            pr_info!(
                                "[RUFS] ufs_queue: timed-out MCQ task completed tag={}\n",
                                task_tag.value(),
                            );
                            return TaskRecoveryResult::Recovered;
                        }
                        Ok(false) => {}
                        Err(_) => return TaskRecoveryResult::Controller,
                    }

                    let index = match Self::recovery_request_index(requests, task_tag) {
                        Some(index) => index,
                        None => return TaskRecoveryResult::Controller,
                    };
                    if requests[index].validate_task_recovery().is_err() {
                        return TaskRecoveryResult::Controller;
                    }
                    match self
                        .backend
                        .claim_mcq_sqe_nullification(queue_id, task_tag)
                    {
                        Ok(()) => {}
                        Err(EAGAIN) => {
                            self.collect_held_completions(requests);
                            if attempt + 1 < TASK_QUERY_RETRIES {
                                fsleep(Delta::from_micros(TASK_QUERY_RETRY_DELAY_US));
                            }
                            continue;
                        }
                        Err(ENOENT) => {
                            if attempt + 1 < TASK_QUERY_RETRIES {
                                fsleep(Delta::from_micros(TASK_QUERY_RETRY_DELAY_US));
                            }
                            continue;
                        }
                        Err(_) => return TaskRecoveryResult::Controller,
                    }
                    if Self::mark_task_recovery(requests, task_tag).is_err() {
                        return TaskRecoveryResult::Controller;
                    }

                    let proof = match self.wait_for_mcq_request_retirement(requests, || {
                        self.backend
                            .finish_mcq_sqe_nullification(queue_id, task_tag)
                    }) {
                        Ok(proof) => proof,
                        Err(_) => return TaskRecoveryResult::Controller,
                    };
                    if self
                        .requeue_task_aborted_request(requests, proof)
                        .is_err()
                    {
                        return TaskRecoveryResult::Controller;
                    }
                    pr_info!(
                        "[RUFS] ufs_queue: MCQ SQE nullified and requeued tag={} queue={}\n",
                        task_tag.value(),
                        queue_id,
                    );
                    return TaskRecoveryResult::Recovered;
                }
                Ok(TaskManagementServiceResponse::FunctionSucceeded) => {
                    let index = match Self::recovery_request_index(requests, task_tag) {
                        Some(index) => index,
                        None => return TaskRecoveryResult::Controller,
                    };
                    if requests[index].validate_task_recovery().is_err() {
                        return TaskRecoveryResult::Controller;
                    }
                    match self.backend.claim_mcq_task(queue_id, task_tag) {
                        Ok(()) => {}
                        Err(EAGAIN) => {
                            self.collect_held_completions(requests);
                            continue;
                        }
                        Err(_) => return TaskRecoveryResult::Controller,
                    }
                    if Self::mark_task_recovery(requests, task_tag).is_err() {
                        return TaskRecoveryResult::Controller;
                    }

                    match host.task_management.abort_task(lun, task_tag) {
                        Ok(TaskManagementServiceResponse::FunctionComplete) => {}
                        Ok(_) => return TaskRecoveryResult::LogicalUnit,
                        Err(_) => return TaskRecoveryResult::Controller,
                    }
                    if self
                        .backend
                        .begin_mcq_task_cleanup(queue_id, task_tag, lun)
                        .is_err()
                    {
                        return TaskRecoveryResult::Controller;
                    }
                    let proof = match self.wait_for_mcq_request_retirement(requests, || {
                        self.backend.finish_mcq_task(queue_id, task_tag)
                    }) {
                        Ok(proof) => proof,
                        Err(_) => return TaskRecoveryResult::Controller,
                    };
                    if self
                        .requeue_task_aborted_request(requests, proof)
                        .is_err()
                    {
                        return TaskRecoveryResult::Controller;
                    }
                    pr_info!(
                        "[RUFS] ufs_queue: MCQ task aborted and requeued tag={} queue={}\n",
                        task_tag.value(),
                        queue_id,
                    );
                    return TaskRecoveryResult::Recovered;
                }
                Ok(_) => return TaskRecoveryResult::LogicalUnit,
                Err(_) => return TaskRecoveryResult::Controller,
            }

        }

        TaskRecoveryResult::LogicalUnit
    }

    fn recover_task(
        self: &Arc<Self>,
        host: &UfsHostShared,
        requests: &mut KVec<RecoveryRequest>,
        task_tag: TaskTag,
    ) -> TaskRecoveryResult {
        if self.backend.supports_sdb_task_abort() {
            self.recover_sdb_task(host, requests, task_tag)
        } else if self.backend.supports_mcq_task_abort() {
            self.recover_mcq_task(host, requests, task_tag)
        } else {
            TaskRecoveryResult::Controller
        }
    }

    fn recover_sdb_logical_unit(
        self: &Arc<Self>,
        host: &UfsHostShared,
        requests: &mut KVec<RecoveryRequest>,
        lun: u8,
    ) -> LogicalUnitRecoveryResult {
        if !self.backend.supports_sdb_logical_reset() {
            return LogicalUnitRecoveryResult::Controller;
        }
        let (task_mask, task_claimed_mask) =
            match Self::sdb_logical_unit_task_masks(requests, lun) {
                Ok(masks) => masks,
                Err(EAGAIN) => return LogicalUnitRecoveryResult::Reevaluate,
                Err(_) => return LogicalUnitRecoveryResult::Controller,
            };
        match self
            .backend
            .claim_sdb_logical_unit(task_mask, task_claimed_mask)
        {
            Ok(()) => {}
            Err(EAGAIN) if task_claimed_mask == 0 => {
                self.collect_held_completions(requests);
                return LogicalUnitRecoveryResult::Reevaluate;
            }
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        }
        if Self::mark_logical_unit_recovery(requests, lun).is_err() {
            return LogicalUnitRecoveryResult::Controller;
        }

        match host.task_management.logical_reset(lun) {
            Ok(TaskManagementServiceResponse::FunctionComplete) => {}
            Ok(response) => {
                pr_info!(
                    "[RUFS] ufs_queue: logical reset rejected lun={} response={:?}\n",
                    lun,
                    response,
                );
                return LogicalUnitRecoveryResult::Controller;
            }
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        }

        let proof = match self.backend.finish_sdb_logical_unit(task_mask) {
            Ok(proof) => proof,
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        };
        let requeued = match self.requeue_logical_unit_requests(requests, lun, proof) {
            Ok(requeued) => requeued,
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        };
        pr_info!(
            "[RUFS] ufs_queue: logical unit recovery completed lun={} requeued={}\n",
            lun,
            requeued,
        );
        LogicalUnitRecoveryResult::Recovered
    }

    fn recover_mcq_logical_unit(
        self: &Arc<Self>,
        host: &UfsHostShared,
        requests: &mut KVec<RecoveryRequest>,
        lun: u8,
    ) -> LogicalUnitRecoveryResult {
        if !self.backend.supports_mcq_logical_reset() {
            return LogicalUnitRecoveryResult::Controller;
        }
        let mut targets = match Self::mcq_logical_unit_targets(requests, lun) {
            Ok(targets) => targets,
            Err(EAGAIN) => return LogicalUnitRecoveryResult::Reevaluate,
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        };
        let task_claimed = targets.iter().any(|target| target.task_claimed());
        match self.backend.claim_mcq_logical_unit(&mut targets) {
            Ok(()) => {}
            Err(EAGAIN) if !task_claimed => {
                self.collect_held_completions(requests);
                return LogicalUnitRecoveryResult::Reevaluate;
            }
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        }
        if Self::mark_logical_unit_recovery(requests, lun).is_err() {
            return LogicalUnitRecoveryResult::Controller;
        }

        match host.task_management.logical_reset(lun) {
            Ok(TaskManagementServiceResponse::FunctionComplete) => {}
            Ok(response) => {
                pr_info!(
                    "[RUFS] ufs_queue: MCQ logical reset rejected lun={} response={:?}\n",
                    lun,
                    response,
                );
                return LogicalUnitRecoveryResult::Controller;
            }
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        }

        self.collect_held_completions(requests);
        if self
            .backend
            .begin_mcq_logical_unit_cleanup(&targets, lun)
            .is_err()
        {
            return LogicalUnitRecoveryResult::Controller;
        }
        let proof = match self.wait_for_mcq_request_retirement(requests, || {
            self.backend.finish_mcq_logical_unit(&targets)
        }) {
            Ok(proof) => proof,
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        };
        let requeued = match self.requeue_mcq_logical_unit_requests(requests, lun, proof) {
            Ok(requeued) => requeued,
            Err(_) => return LogicalUnitRecoveryResult::Controller,
        };
        pr_info!(
            "[RUFS] ufs_queue: MCQ logical unit recovery completed lun={} requeued={}\n",
            lun,
            requeued,
        );
        LogicalUnitRecoveryResult::Recovered
    }

    fn recover_logical_unit(
        self: &Arc<Self>,
        host: &UfsHostShared,
        requests: &mut KVec<RecoveryRequest>,
        lun: u8,
    ) -> LogicalUnitRecoveryResult {
        if self.backend.supports_sdb_logical_reset() {
            self.recover_sdb_logical_unit(host, requests, lun)
        } else if self.backend.supports_mcq_logical_reset() {
            self.recover_mcq_logical_unit(host, requests, lun)
        } else {
            LogicalUnitRecoveryResult::Controller
        }
    }

    fn dispose_recovery_requests(
        &self,
        requests: &mut KVec<RecoveryRequest>,
        requeue: bool,
        dma_ended: &DmaAccessEnded,
    ) -> Result<usize> {
        let expected = requests.len();
        let mut disposed = 0;

        while let Some(recovery) = requests.pop() {
            let RecoveryRequest { request, io, .. } = recovery;
            let cmd = io.retire_after_stop(dma_ended);
            let prepare = {
                let mut state = request.data_ref().inner.lock();
                state.prepare_recovery_return(cmd, requeue)
            };
            if let Err(e) = prepare {
                let _ = UfsRequestData::try_end_request(request, bindings::BLK_STS_IOERR);
                return Err(e);
            }

            if requeue {
                request.requeue(true);
            } else {
                request.end(bindings::BLK_STS_IOERR);
            }
            disposed += 1;
        }

        if disposed != expected {
            pr_err!(
                "rufs: recovery disposition mismatch: expected={} disposed={}\n",
                expected,
                disposed,
            );
            return Err(EIO);
        }

        Ok(disposed)
    }

    fn release_recovery_requests(
        &self,
        requests: &mut KVec<RecoveryRequest>,
    ) -> Result<()> {
        while let Some(request) = requests.pop() {
            if let Err(request) = request.restore() {
                requests
                    .push_within_capacity(request)
                    .map_err(|_| EOVERFLOW)?;
                return Err(EIO);
            }
        }
        Ok(())
    }

    // Issuing
    pub(crate) fn compose_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag) -> Result<()> {
        self.dma
            .compose_devman_upiu(cmd, u32::from(task_tag.value()))
    }

    fn compose_scsi(
        rq: &Owned<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
        task_tag: TaskTag,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<UfsPreparedMapping> {
        let queue = rq.queue_data().queue();

        queue
            .dma
            .compose_scsi_upiu(rq, cmd, task_tag.value(), mempool)
    }

    fn fetch_dev(
        &self,
        cmd: UfsDevCmd,
        task_tag: TaskTag,
        payload: CompletionPayload,
    ) -> Result<UfsCmd> {
        match payload {
            CompletionPayload::Sdb => self.dma.fetch_devman_upiu(cmd, task_tag.index()),
            CompletionPayload::Mcq(cqe) => {
                self.dma.fetch_mcq_devman_upiu(cmd, task_tag.index(), cqe)
            }
        }
    }

    fn fetch_scsi_completion(
        &self,
        task_tag: TaskTag,
        payload: CompletionPayload,
    ) -> UfsScsiResult {
        match payload {
            CompletionPayload::Sdb => self.dma.fetch_scsi_completion(task_tag.index()),
            CompletionPayload::Mcq(cqe) => {
                self.dma.fetch_mcq_scsi_completion(task_tag.index(), cqe)
            }
        }
    }

    fn collect_backend_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        self.backend.collect_completions(completed)
    }

    fn collect_recovery_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        self.backend.collect_recovery_completions(completed)
    }

    pub(crate) fn interrupt_queues(&self) -> &[McqInterruptQueue] {
        self.backend.interrupt_queues()
    }

    fn dump_backend_state(&self, tag: usize, reason: &str) {
        self.backend.dump_state(tag, reason);
    }

    fn take_request_at_task_tag(
        &self,
        task_tag: TaskTag,
    ) -> Result<Option<Owned<mq::Request<UfsLuBlockOps>>>> {
        let Some(rq) = self
            .tags
            .try_shared_tag_to_rq(u32::from(task_tag.value()))?
        else {
            return Ok(None);
        };

        OwnableRefCounted::try_from_shared(rq)
            .map(Some)
            .map_err(|_| EBUSY)
    }

    fn try_claim_recovery_request(
        &self,
        task_tag: TaskTag,
    ) -> Result<Option<Owned<mq::Request<UfsLuBlockOps>>>> {
        let rq = match self
            .tags
            .try_shared_tag_to_rq(u32::from(task_tag.value()))
        {
            Ok(Some(rq)) => rq,
            Ok(None) | Err(EBUSY) => return Ok(None),
            Err(e) => return Err(e),
        };

        OwnableRefCounted::try_from_shared(rq)
            .map(Some)
            .map_err(|_| EAGAIN)
    }

    fn resolve_completion(
        self: &Arc<Self>,
        completion: BackendCompletion,
        host: &Arc<UfsHostShared>,
    ) -> Option<ResolvedCompletion> {
        let (task_tag, queue_id, payload) = completion.into_parts();
        match self.take_request_at_task_tag(task_tag) {
            Ok(Some(rq)) => {
                if rq.tag() != u32::from(task_tag.value()) || rq.queue_index() != queue_id {
                    host.require_recovery("completion queue mismatch", task_tag.index());
                    return None;
                }
                Some(ResolvedCompletion {
                    rq,
                    task_tag,
                    queue_id,
                    payload,
                })
            }
            Ok(None) => {
                host.require_recovery("completion tag has no request", task_tag.index());
                None
            }
            Err(EBUSY) => {
                host.require_recovery("completion ownership conflict", task_tag.index());
                None
            }
            Err(_) => {
                host.require_recovery("completion request lookup failed", task_tag.index());
                None
            }
        }
    }

    fn complete_from<F>(
        self: &Arc<Self>,
        mut collect: F,
        queue_id: Option<u32>,
        during_recovery: bool,
        host: &Arc<UfsHostShared>,
    ) -> bool
    where
        F: FnMut(&mut CompletionBatch) -> Result<()>,
    {
        if !during_recovery && host.recovery_required() {
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
            let mut requests = CompletionBatch::new();
            let collect_result = collect(&mut requests);

            let batch_full = requests.is_full();
            while let Some(request) = requests.take_next() {
                if let Some(completion) = self.resolve_completion(request, host) {
                    any_completed |= completion.complete().released();
                }
            }

            if let Some(fault) = requests.take_fault() {
                if let Some(queue_id) = fault.queue_id {
                    host.require_mcq_recovery(fault.reason, queue_id, fault.tag);
                } else {
                    host.require_recovery(fault.reason, fault.tag);
                }
                return any_completed;
            }
            if let Err(e) = collect_result {
                if let Some(queue_id) = queue_id {
                    pr_err!(
                        "[RUFS] ufs_queue: collect queue {} completions failed errno={}\n",
                        queue_id,
                        e.to_errno(),
                    );
                    self.dump_backend_state(0, "collect queue failed");
                    host.require_mcq_recovery("completion collection failed", queue_id, 0);
                } else {
                    pr_err!(
                        "[RUFS] ufs_queue: collect completions failed errno={}\n",
                        e.to_errno(),
                    );
                    self.dump_backend_state(0, "collect completions failed");
                    host.require_recovery("completion collection failed", 0);
                }
                return any_completed;
            }
            if !batch_full {
                break;
            }
        }

        any_completed
    }

    pub(crate) fn complete(
        self: &Arc<Self>,
        host: &Arc<UfsHostShared>,
    ) -> bool {
        self.complete_from(
            |requests| self.collect_backend_completions(requests),
            None,
            false,
            host,
        )
    }

    pub(crate) fn complete_queue(
        self: &Arc<Self>,
        interrupt_queue: &McqInterruptQueue,
        host: &Arc<UfsHostShared>,
    ) -> bool {
        let queue_id = interrupt_queue.id();
        self.complete_from(
            |requests| interrupt_queue.collect_completions(requests),
            Some(queue_id),
            false,
            host,
        )
    }

    fn reconcile_completions(
        self: &Arc<Self>,
        host: &Arc<UfsHostShared>,
    ) {
        self.complete_from(
            |requests| self.collect_recovery_completions(requests),
            None,
            true,
            host,
        );
        self.tags.wait_completed_requests();
    }

    pub(crate) fn poll(
        self: &Arc<Self>,
        hw_queue: &UfsHwQueue,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
        host: &Arc<UfsHostShared>,
    ) -> bool {
        if host.recovery_required() {
            return false;
        }
        let mut any_completed = false;
        for _ in 0..self.completion_pass_limit() {
            let mut requests = CompletionBatch::new();
            let poll_result = hw_queue.poll(&mut requests);

            let batch_full = requests.is_full();
            while let Some(request) = requests.take_next() {
                if let Some(completion) = self.resolve_completion(request, host) {
                    any_completed |= completion.complete_polled(batch).released();
                }
            }

            if let Some(fault) = requests.take_fault() {
                if let Some(queue_id) = fault.queue_id {
                    host.require_mcq_recovery(fault.reason, queue_id, fault.tag);
                } else {
                    host.require_recovery(fault.reason, fault.tag);
                }
                return any_completed;
            }
            if let Err(e) = poll_result {
                pr_err!(
                    "[RUFS] ufs_queue: poll queue {} failed errno={}\n",
                    hw_queue.id(),
                    e.to_errno(),
                );
                host.require_recovery(
                    "polled completion collection failed",
                    hw_queue.id() as usize,
                );
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
        rq: Owned<mq::Request<UfsLuBlockOps>>,
        target: CompletionTarget<'_>,
    ) -> CompletionOutcome {
        let tag = rq.tag();
        let host = rq.queue_data().host().clone();
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

        let status = u32::from(status);
        let disposition = if requeue {
            CompletionDisposition::Requeue
        } else {
            CompletionDisposition::End(status)
        };
        match target {
            CompletionTarget::Interrupt => {
                if rq
                    .data_ref()
                    .inner
                    .lock()
                    .schedule_completion(disposition)
                    .is_err()
                {
                    return UfsRequestData::fail_dma_free_completion(
                        rq,
                        self,
                        "invalid SCSI completion state",
                    );
                }
                mq::Request::complete(OwnableRefCounted::into_shared(rq));
                CompletionOutcome::Released
            }
            CompletionTarget::Poll(batch) => {
                if rq
                    .data_ref()
                    .inner
                    .lock()
                    .finish_direct_completion()
                    .is_err()
                {
                    return UfsRequestData::fail_dma_free_completion(
                        rq,
                        self,
                        "invalid polled completion state",
                    );
                }
                if requeue {
                    return match UfsRequestData::try_requeue_request(rq) {
                        Ok(()) => CompletionOutcome::Released,
                        Err(rq) => {
                            host.require_recovery(
                                "polled request returned with active DMA",
                                tag as usize,
                            );
                            drop(rq);
                            CompletionOutcome::RetainedForRecovery
                        }
                    };
                }
                if status != u32::from(bindings::BLK_STS_OK) {
                    return UfsRequestData::end_request(rq, status);
                }

                if !UfsRequestData::returnable_for_batch(&rq) {
                    host.require_recovery(
                        "polled batch request returned with active DMA",
                        tag as usize,
                    );
                    drop(rq);
                    return CompletionOutcome::RetainedForRecovery;
                }
                if let Err(rq) = batch.add_request(rq, false) {
                    return UfsRequestData::end_request(rq, status);
                }
                CompletionOutcome::Released
            }
        }
    }
}

impl UfsHostShared {
    fn select_recovery_plan(
        &self,
        requests: &KVec<RecoveryRequest>,
        cause: RecoveryCause,
    ) -> RecoveryPlan {
        if !matches!(cause.reason, RecoveryReason::Timeout) {
            return RecoveryPlan::Controller;
        }

        let mut first_tag = None;
        let mut lun = None;
        let mut timeout_count = 0usize;

        for request in requests.iter() {
            if request.has_completion() {
                return RecoveryPlan::Controller;
            }
            let Some(cmd) = request.unresolved_timeout() else {
                continue;
            };

            let UfsCmd::Scsi(cmd) = cmd else {
                return RecoveryPlan::Controller;
            };
            timeout_count += 1;
            if first_tag.is_none() {
                first_tag = TaskTag::new(request.tag()).ok();
            }
            match lun {
                None => lun = Some(cmd.lun()),
                Some(current) if current != cmd.lun() => {
                    return RecoveryPlan::Controller;
                }
                Some(_) => {}
            }
        }

        match (timeout_count, first_tag, lun) {
            (0, _, _) => RecoveryPlan::Resume,
            (1, Some(task_tag), _) => RecoveryPlan::Task(task_tag),
            (_, _, Some(lun)) => RecoveryPlan::LogicalUnit(lun),
            _ => RecoveryPlan::Controller,
        }
    }

    pub(crate) fn run_recovery(this: Arc<Self>) {
        let queue = this.queue.clone();
        let _control = this.lock_control();

        if this.recovery.lock().begin().is_none() {
            return;
        }

        queue.tags.quiesce();
        queue.reconcile_completions(&this);

        let mut cause = {
            let mut recovery = this.recovery.lock();
            let Some(cause) = recovery.start_plan() else {
                return;
            };
            cause
        };

        pr_info!(
            "[RUFS] ufs_host: controller recovery started reason={} queue={:?} tag={}\n",
            cause.reason.name(),
            cause.source_queue,
            cause.context,
        );
        if let Some(errors) = cause.reason.uic_errors() {
            pr_err!(
                "rufs: UIC error: phy={:08x} dl={:08x} nl={:08x} tl={:08x} dme={:08x}\n",
                errors.phy,
                errors.data_link,
                errors.network,
                errors.transport,
                errors.dme,
            );
        }
        let mut requests = match queue.claim_recovery_requests(&this) {
            Ok(requests) => requests,
            Err(error) => {
                pr_err!(
                    "rufs: recovery request claim failed errno={}\n",
                    error.to_errno(),
                );
                this.recovery.lock().fail(cause);
                return;
            }
        };
        loop {
            queue.collect_held_completions(&mut requests);
            if let Err(error) = queue.complete_recovery_completions(&mut requests) {
                pr_err!(
                    "rufs: recovery completion resolution failed errno={}\n",
                    error.to_errno(),
                );
                break;
            }

            cause = this.recovery.lock().refresh_plan(cause);
            let plan = this.select_recovery_plan(&requests, cause);
            match plan {
                RecoveryPlan::Resume => {
                    if let Err(error) = queue.release_recovery_requests(&mut requests) {
                        pr_err!(
                            "rufs: recovery request release failed errno={}\n",
                            error.to_errno(),
                        );
                        this.recovery.lock().fail(cause);
                        return;
                    }
                    let outcome = this.recovery.lock().finish_plan();
                    match outcome {
                        RecoveryOutcome::Pending(pending) => {
                            // A cause raced with the final request release.
                            // Recovery remained active while requests were
                            // restored, so reclaim them before re-evaluating
                            // the wider plan.
                            cause = pending;
                            drop(requests);
                            requests = match queue.claim_recovery_requests(&this) {
                                Ok(requests) => requests,
                                Err(error) => {
                                    pr_err!(
                                        "rufs: recovery request reclaim failed errno={}\n",
                                        error.to_errno(),
                                    );
                                    this.recovery.lock().fail(cause);
                                    return;
                                }
                            };
                            continue;
                        }
                        RecoveryOutcome::Resume => {
                            // A completion may arrive after the worker's last
                            // recovery-owned drain while an IRQ thread still
                            // observes recovery as active. Such an IRQ may
                            // acknowledge its event without consuming the
                            // completion. Recovery is now operational, so do
                            // one final all-queue drain before dispatch resumes.
                            // A completion arriving after this drain is owned by
                            // the normal IRQ/poll path.
                            queue.reconcile_completions(&this);
                            queue.tags.unquiesce();
                        }
                        RecoveryOutcome::Stop => {}
                    }
                    return;
                }
                RecoveryPlan::Task(task_tag) => {
                    match queue.recover_task(&this, &mut requests, task_tag) {
                        TaskRecoveryResult::Recovered => {
                            // Keep every other request recovery-owned. The
                            // next iteration resolves completions and merges
                            // causes that arrived during the task operation
                            // before deciding whether to resume or escalate.
                            continue;
                        }
                        TaskRecoveryResult::LogicalUnit => {
                            let lun = match UfsQueue::recovery_request_lun(
                                &requests,
                                task_tag,
                            ) {
                                Ok(lun) => lun,
                                Err(_) => break,
                            };
                            match queue.recover_logical_unit(&this, &mut requests, lun) {
                                LogicalUnitRecoveryResult::Recovered
                                | LogicalUnitRecoveryResult::Reevaluate => continue,
                                LogicalUnitRecoveryResult::Controller => break,
                            }
                        }
                        TaskRecoveryResult::Controller => {
                            pr_info!(
                                "[RUFS] ufs_host: task recovery unavailable tag={}, escalate to controller recovery\n",
                                task_tag.value(),
                            );
                            break;
                        }
                    }
                }
                RecoveryPlan::LogicalUnit(lun) => {
                    match queue.recover_logical_unit(&this, &mut requests, lun) {
                        LogicalUnitRecoveryResult::Recovered
                        | LogicalUnitRecoveryResult::Reevaluate => continue,
                        LogicalUnitRecoveryResult::Controller => break,
                    }
                }
                RecoveryPlan::Controller => break,
            }
        }
        // A controller reset makes every command that was visible to the old
        // controller instance unreachable. Only after that boundary may RUFS
        // return their blk-mq tags. The controller fallback has a one-attempt
        // budget; a new fault observed during reset enters fail-stop instead
        // of repeating the same reset indefinitely.
        match this.reset_controller() {
            ControllerResetResult::Restored(dma_ended) => {
                let disposition =
                    queue.dispose_recovery_requests(&mut requests, true, &dma_ended);
                let Ok(disposed) = disposition else {
                    let error = disposition.unwrap_err();
                    pr_err!(
                        "[RUFS] ufs_host: recovery request cleanup failed errno={}\n",
                        error.to_errno(),
                    );
                    this.recovery.lock().fail(cause);
                    return;
                };

                let outcome = this.recovery.lock().finish_plan();
                match outcome {
                    RecoveryOutcome::Pending(pending) => {
                        pr_err!(
                            "rufs: recovery event arrived during controller reset reason={}\n",
                            pending.reason.name(),
                        );
                        this.recovery.lock().fail(pending);
                    }
                    RecoveryOutcome::Resume => {
                        queue.tags.unquiesce();
                        pr_info!(
                            "[RUFS] ufs_host: controller recovery completed requeued={}\n",
                            disposed,
                        );
                    }
                    RecoveryOutcome::Stop => {}
                }
            }
            ControllerResetResult::Stopped { dma_ended, error } => {
                pr_err!(
                    "[RUFS] ufs_host: controller recovery reset failed errno={}\n",
                    error.to_errno(),
                );
                if let Err(e) =
                    queue.dispose_recovery_requests(&mut requests, false, &dma_ended)
                {
                    pr_err!(
                        "[RUFS] ufs_host: recovery request cleanup failed errno={}\n",
                        e.to_errno(),
                    );
                }
                this.recovery.lock().fail(cause);
            }
            ControllerResetResult::UnsafeStop {
                reset_error,
                stop_error,
            } => {
                // Hardware may still own descriptors, mappings, requests, and
                // their tags. Keep all of them pinned in the quiesced tag set;
                // returning only the blk-mq request would permit a late DMA or
                // completion to target a reused tag.
                if let Some(error) = reset_error {
                    pr_err!(
                        "[RUFS] ufs_host: controller recovery reset failed errno={}\n",
                        error.to_errno(),
                    );
                }
                pr_err!(
                    "[RUFS] ufs_host: controller stop could not be established errno={}\n",
                    stop_error.to_errno(),
                );
                this.recovery.lock().fail(cause);
            }
        }
    }
}
