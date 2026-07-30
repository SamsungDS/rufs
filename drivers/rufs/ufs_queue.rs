// SPDX-License-Identifier: GPL-2.0

use crate::ufs_command::{CommandOwner, CommandPool, TaskTag, TASK_TAG_COUNT};
use crate::ufs_dev::*;
use crate::ufs_dma::*;
use crate::ufs_lu::{QueueData, TagSetData, UfsLuBlockOps, UfsRequestData};
use crate::ufs_reg::*;
use kernel::alloc::mempool::MemPool;
use kernel::block::mq;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::block::mq::TagSet;
use kernel::cpu;
use kernel::sync::{aref::ARef, barrier, Arc, SpinLock, SpinLockIrq};
use kernel::types::OwnableRefCounted;
use kernel::workqueue::{self, impl_has_work, new_work, Work, WorkItem};
use kernel::{bindings, new_spinlock, new_spinlock_irq, prelude::*};

const READ_10: u8 = 0x28;
const WRITE_10: u8 = 0x2a;
const SYNCHRONIZE_CACHE: u8 = 0x35;
const UNMAP: u8 = 0x42;
const READ_16: u8 = 0x88;
const WRITE_16: u8 = 0x8a;
const UFS_MCQ_DEFAULT_READ_QUEUES: usize = 0;
const UFS_MCQ_DEFAULT_POLL_QUEUES: usize = 1;
const UFS_SOFTWARE_QUEUE_DEPTH: usize = 256;
const COMPLETION_BATCH_SIZE: usize = 16;

fn possible_cpus() -> usize {
    (cpu::nr_cpu_ids() as usize).max(1)
}

#[derive(Copy, Clone)]
pub(crate) struct McqConfig {
    max_queues: usize,
    total_queues: usize,
    default_queues: usize,
    read_queues: usize,
    interrupt_queues: usize,
    poll_queues: usize,
    max_active_commands: usize,
    task_tag_count: usize,
    software_queue_depth: usize,
    ring_entries: usize,
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

    fn is_poll_queue(&self, queue: usize) -> bool {
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
        if !reg.mcq_supported() {
            let tag_count = reg.nutrs();
            if tag_count == 0 || tag_count > u32::BITS as usize {
                return Err(EINVAL);
            }
            return Ok(Self::Sdb { tag_count });
        }

        let max_queues = reg.mcq_max_queues();
        let read_queues = UFS_MCQ_DEFAULT_READ_QUEUES;
        let poll_queues = UFS_MCQ_DEFAULT_POLL_QUEUES;
        let reserved_queues = read_queues
            .checked_add(poll_queues)
            .ok_or(EOVERFLOW)?;
        if max_queues <= reserved_queues {
            return Err(ENOTSUPP);
        }

        let default_queues = core::cmp::min(max_queues - reserved_queues, possible_cpus());
        let interrupt_queues = default_queues
            .checked_add(read_queues)
            .ok_or(EOVERFLOW)?;
        let total_queues = interrupt_queues
            .checked_add(poll_queues)
            .ok_or(EOVERFLOW)?;
        let max_active_commands = core::cmp::min(reg.nutrs_mcq(), TASK_TAG_COUNT);
        let task_tag_count = TASK_TAG_COUNT;
        let software_queue_depth = UFS_SOFTWARE_QUEUE_DEPTH;
        let ring_entries = max_active_commands.checked_add(1).ok_or(EOVERFLOW)?;
        if interrupt_queues == 0 || max_active_commands == 0 {
            return Err(EINVAL);
        }

        Ok(Self::Mcq(McqConfig {
            max_queues,
            total_queues,
            default_queues,
            read_queues,
            interrupt_queues,
            poll_queues,
            max_active_commands,
            task_tag_count,
            software_queue_depth,
            ring_entries,
        }))
    }

    pub(crate) fn tag_count(&self) -> usize {
        match self {
            Self::Sdb { tag_count } => *tag_count,
            Self::Mcq(config) => config.task_tag_count,
        }
    }

    fn software_queue_depth(&self) -> usize {
        match self {
            // Keep submitters from blocking on blk-mq tags before they can
            // reap polled commands. The command pool still limits SDB to NUTRS.
            Self::Sdb { .. } => UFS_SOFTWARE_QUEUE_DEPTH,
            Self::Mcq(config) => config.software_queue_depth,
        }
    }

    fn max_active_commands(&self) -> usize {
        match self {
            Self::Sdb { tag_count } => *tag_count,
            Self::Mcq(config) => config.max_active_commands,
        }
    }

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
        let poll_offset = read_offset
            .checked_add(read_queues)
            .ok_or(EOVERFLOW)?;

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

#[derive(PartialEq, Copy, Clone, Debug)]
pub(crate) enum UfsScsiDataDirection {
    None,
    Read,
    Write,
}

struct ScsiSense {
    response_code: u8,
    sense_key: u8,
    asc: u8,
    ascq: u8,
    additional_len: u8,
}

const SCSI_SENSE_UNIT_ATTENTION: u8 = 0x6;
const SCSI_ASC_POWER_ON_RESET: u8 = 0x29;
const SCSI_ASCQ_POWER_ON_RESET: u8 = 0x00;

fn parse_scsi_sense(data: &[u8], len: usize) -> Option<ScsiSense> {
    if len == 0 {
        return None;
    }

    let response_code = data[0] & 0x7f;
    match response_code {
        // Fixed format sense data.
        0x70 | 0x71 if len >= 14 => Some(ScsiSense {
            response_code,
            sense_key: data[2] & 0x0f,
            asc: data[12],
            ascq: data[13],
            additional_len: data[7],
        }),
        // Descriptor format sense data.
        0x72 | 0x73 if len >= 4 => Some(ScsiSense {
            response_code,
            sense_key: data[1] & 0x0f,
            asc: data[2],
            ascq: data[3],
            additional_len: 0,
        }),
        _ => None,
    }
}

fn retryable_check_condition(sense: Option<&ScsiSense>) -> bool {
    matches!(sense, Some(sense) if sense.sense_key == SCSI_SENSE_UNIT_ATTENTION)
}

fn boot_unit_attention(sense: Option<&ScsiSense>) -> bool {
    matches!(
        sense,
        Some(sense)
            if sense.sense_key == SCSI_SENSE_UNIT_ATTENTION
                && sense.asc == SCSI_ASC_POWER_ON_RESET
                && sense.ascq == SCSI_ASCQ_POWER_ON_RESET
    )
}

fn should_requeue_scsi(completion: UfsScsiCompletion, sense: Option<&ScsiSense>) -> bool {
    matches!(
        completion,
        UfsScsiCompletion::Busy | UfsScsiCompletion::TaskSetFull | UfsScsiCompletion::Requeue
    ) || (matches!(completion, UfsScsiCompletion::CheckCondition)
        && retryable_check_condition(sense))
}

fn sense_key_name(key: u8) -> &'static str {
    match key {
        0x0 => "NO_SENSE",
        0x1 => "RECOVERED_ERROR",
        0x2 => "NOT_READY",
        0x3 => "MEDIUM_ERROR",
        0x4 => "HARDWARE_ERROR",
        0x5 => "ILLEGAL_REQUEST",
        0x6 => "UNIT_ATTENTION",
        0x7 => "DATA_PROTECT",
        0x8 => "BLANK_CHECK",
        0x9 => "VENDOR_SPECIFIC",
        0xb => "ABORTED_COMMAND",
        0xd => "VOLUME_OVERFLOW",
        0xe => "MISCOMPARE",
        _ => "UNKNOWN",
    }
}

#[derive(Copy, Clone)]
pub(crate) struct UfsSCSICmd {
    lun: u8,
    direction: UfsScsiDataDirection,
    data_len: u32,
    cdb: [u8; 16],
    unmap_lba: u64,
    unmap_blocks: u32,
}

impl UfsSCSICmd {
    pub(crate) fn read_write(
        lun: u8,
        write: bool,
        lba: u64,
        blocks: u32,
        data_len: u32,
        fua: bool,
    ) -> Self {
        let mut cdb = [0u8; 16];
        let direction = if write {
            UfsScsiDataDirection::Write
        } else {
            UfsScsiDataDirection::Read
        };
        let flags = if fua { 0x8 } else { 0 };

        match (u32::try_from(lba), u16::try_from(blocks)) {
            (Ok(lba), Ok(blocks)) => {
                cdb[0] = if write { WRITE_10 } else { READ_10 };
                cdb[1] = flags;
                cdb[2..6].copy_from_slice(&lba.to_be_bytes());
                cdb[7..9].copy_from_slice(&blocks.to_be_bytes());
            }
            _ => {
                cdb[0] = if write { WRITE_16 } else { READ_16 };
                cdb[1] = flags;
                cdb[2..10].copy_from_slice(&lba.to_be_bytes());
                cdb[10..14].copy_from_slice(&blocks.to_be_bytes());
            }
        }

        Self {
            lun,
            direction,
            data_len,
            cdb,
            unmap_lba: 0,
            unmap_blocks: 0,
        }
    }

    pub(crate) fn flush(lun: u8) -> Self {
        let mut cdb = [0u8; 16];
        cdb[0] = SYNCHRONIZE_CACHE;

        Self {
            lun,
            direction: UfsScsiDataDirection::None,
            data_len: 0,
            cdb,
            unmap_lba: 0,
            unmap_blocks: 0,
        }
    }

    pub(crate) fn unmap(lun: u8, lba: u64, blocks: u32) -> Self {
        let mut cdb = [0u8; 16];
        let data_len = 24u32;
        cdb[0] = UNMAP;
        cdb[7..9].copy_from_slice(&(data_len as u16).to_be_bytes());

        Self {
            lun,
            direction: UfsScsiDataDirection::Write,
            data_len,
            cdb,
            unmap_lba: lba,
            unmap_blocks: blocks,
        }
    }

    pub(crate) fn lun(&self) -> u8 {
        self.lun
    }

    pub(crate) fn direction(&self) -> UfsScsiDataDirection {
        self.direction
    }

    pub(crate) fn data_len(&self) -> u32 {
        self.data_len
    }

    pub(crate) fn cdb(&self) -> [u8; 16] {
        self.cdb
    }

    pub(crate) fn is_unmap(&self) -> bool {
        self.cdb[0] == UNMAP
    }

    pub(crate) fn unmap_lba(&self) -> u64 {
        self.unmap_lba
    }

    pub(crate) fn unmap_blocks(&self) -> u32 {
        self.unmap_blocks
    }
}

#[derive(Copy, Clone)]
pub(crate) enum UfsCmd {
    Device(UfsDevCmd),
    SCSI(UfsSCSICmd),
}

impl UfsCmd {
    pub(crate) fn get_device(&self) -> Result<UfsDevCmd> {
        match *self {
            Self::Device(cmd) => Ok(cmd),
            _ => Err(EINVAL),
        }
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
}

impl RecoveryState {
    fn cause(&self) -> Option<RecoveryCause> {
        match *self {
            Self::Operational => None,
            Self::Requested(cause)
            | Self::Quiescing(cause)
            | Self::Recovering(cause)
            | Self::Failed(cause) => Some(cause),
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
        self.state = UfsRequestState::Prepared {
            cmd,
            prdt: None,
        };
        Ok(())
    }

    fn prepare_scsi(
        &mut self,
        cmd: UfsSCSICmd,
        prdt: Option<UfsPrdtMapping>,
    ) -> Result<()> {
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

    fn begin_completion(
        &mut self,
        queue_id: u32,
    ) -> Result<(UfsCmd, Option<UfsPrdtMapping>)> {
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
}

struct SdbTransferBackend {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    state: Arc<SdbTransferState>,
}

#[derive(Default)]
struct SdbCompletionState {
    outstanding: u32,
    polled: u32,
}

#[derive(Copy, Clone)]
enum SdbCompletionSource {
    Interrupt,
    Poll,
}

#[pin_data]
struct SdbTransferState {
    #[pin]
    completion: SpinLockIrq<SdbCompletionState>,
}

#[derive(Clone, Copy)]
struct CompletedRequest {
    task_tag: TaskTag,
    queue_id: u32,
    cqe: Option<CqEntry>,
}

struct ResolvedCompletion {
    rq: ARef<mq::Request<UfsLuBlockOps>>,
    task_tag: TaskTag,
    queue_id: u32,
    cqe: Option<CqEntry>,
}

impl ResolvedCompletion {
    fn complete(self) -> CompletionOutcome {
        UfsRequestData::complete(self.rq, self.task_tag, self.queue_id, self.cqe)
    }

    fn complete_polled(
        self,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> CompletionOutcome {
        UfsRequestData::complete_polled(
            self.rq,
            self.task_tag,
            self.queue_id,
            self.cqe,
            batch,
        )
    }
}

#[derive(Clone, Copy)]
struct CompletionFault {
    reason: &'static str,
    tag: usize,
    queue_id: Option<u32>,
}

struct CompletedRequests {
    requests: [Option<CompletedRequest>; COMPLETION_BATCH_SIZE],
    len: usize,
    pos: usize,
    fault: Option<CompletionFault>,
}

enum SubmissionOutcome {
    Submitted,
    NotSubmitted(Error),
    PublishFailed(Error),
}

impl CompletedRequests {
    fn new() -> Self {
        Self {
            requests: [None; COMPLETION_BATCH_SIZE],
            len: 0,
            pos: 0,
            fault: None,
        }
    }

    fn insert(
        &mut self,
        task_tag: TaskTag,
        queue_id: u32,
        cqe: Option<CqEntry>,
    ) -> Result<()> {
        if self.len == self.requests.len() {
            return Err(ENOMEM);
        }

        self.requests[self.len] = Some(CompletedRequest {
            task_tag,
            queue_id,
            cqe,
        });
        self.len += 1;
        Ok(())
    }

    fn insert_sdb_mask(&mut self, mut mask: u32) -> Result<u32> {
        let mut inserted = 0;

        while mask != 0 && !self.is_full() {
            let tag = mask.trailing_zeros();
            let tag_mask = 1u32 << tag;
            mask &= !tag_mask;
            self.insert(TaskTag::new(tag)?, 0, None)?;
            inserted |= tag_mask;
        }

        Ok(inserted)
    }

    fn is_full(&self) -> bool {
        self.len == self.requests.len()
    }

    fn record_fault(&mut self, reason: &'static str, tag: usize, queue_id: Option<u32>) {
        if self.fault.is_none() {
            self.fault = Some(CompletionFault {
                reason,
                tag,
                queue_id,
            });
        }
    }

    fn take_fault(&mut self) -> Option<CompletionFault> {
        self.fault.take()
    }

    fn take_next(&mut self) -> Option<CompletedRequest> {
        if self.pos == self.len {
            return None;
        }

        let request = self.requests[self.pos].take();
        self.pos += 1;
        request
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

    fn submit<F>(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        tag: u32,
        publish: F,
    ) -> SubmissionOutcome
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

        barrier::dma_wmb();
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
        completion.update_tail(reg, &self.descriptor)?;
        barrier::dma_rmb();
        let mut consumed = false;
        let result = (|| {
            while !completion.is_empty() && !completed_requests.is_full() {
                consumed = true;
                if let Some(cqe) = completion.consume_entry(&self.descriptor)? {
                    match dma.tag_from_cq_entry(&cqe, self.descriptor.id()) {
                        Ok(tag) => match TaskTag::from_index(tag) {
                            Ok(task_tag) => completed_requests.insert(
                                task_tag,
                                self.descriptor.id(),
                                Some(cqe),
                            )?,
                            Err(_) => completed_requests.record_fault(
                                "invalid MCQ completion task tag",
                                tag,
                                Some(self.descriptor.id()),
                            ),
                        },
                        Err(_) => completed_requests.record_fault(
                            "invalid MCQ completion descriptor",
                            usize::from(cqe.task_tag()),
                            Some(self.descriptor.id()),
                        ),
                    }
                }
            }
            Ok(())
        })();
        if consumed {
            completion.commit_head(reg, &self.descriptor)?;
        }
        completion.acknowledge_events(reg, &self.descriptor)?;

        result
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
            let cqis = reg
                .read_mcq_cqis(descriptor.oprs(), id)
                .unwrap_or(u32::MAX);
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

            reg.set_mcq_sq_base_addr(id, sq_dma_addr)?;
            reg.write_mcq_sqdao(
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Sqd, id),
            )?;
            reg.write_mcq_sqisao(
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Sqis, id),
            )?;

            reg.set_mcq_cq_base_addr(id, cq_dma_addr)?;
            reg.write_mcq_cqdao(
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Cqd, id),
            )?;
            reg.write_mcq_cqisao(
                id,
                reg.mcq_opr_region_offset(descriptor.oprs(), UfsMcqOprRegion::Cqis, id),
            )?;

            submission.reset();
            completion.reset();
            if id < interrupt_queues {
                reg.enable_mcq_cq_tail_push_intr(descriptor.oprs(), id)?;
            }
            reg.enable_mcq_cq(id, descriptor.max_entries() as usize)?;
            reg.enable_mcq_sq(id, descriptor.max_entries() as usize, id)?;
        }

        Ok(())
    }
}

struct McqTransferBackend {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    config: McqConfig,
    queues: Arc<McqQueueSet>,
}

struct UfsTransferBackend {
    ops: KBox<dyn UfsTransferOps>,
}

#[derive(Clone)]
pub(crate) struct UfsHwQueue {
    inner: UfsHwQueueKind,
}

#[derive(Clone)]
enum UfsHwQueueKind {
    Sdb {
        reg: Arc<UfsReg>,
        state: Arc<SdbTransferState>,
    },
    Mcq {
        reg: Arc<UfsReg>,
        dma: Arc<UfsDma>,
        queue: Arc<McqHardwareQueue>,
        poll: bool,
    },
}

impl UfsHwQueue {
    pub(crate) fn id(&self) -> u32 {
        match &self.inner {
            UfsHwQueueKind::Sdb { .. } => 0,
            UfsHwQueueKind::Mcq { queue, .. } => queue.descriptor.id(),
        }
    }

    fn submit<F>(&self, tag: u32, polled: bool, publish: F) -> SubmissionOutcome
    where
        F: FnOnce() -> Result<()>,
    {
        match &self.inner {
            UfsHwQueueKind::Sdb { reg, state } => {
                let Some(mask) = SdbTransferBackend::tag_mask(tag) else {
                    return SubmissionOutcome::NotSubmitted(EINVAL);
                };
                let mut state = state.completion.lock();

                if state.outstanding & mask != 0 {
                    return SubmissionOutcome::NotSubmitted(EBUSY);
                }
                if let Err(e) = publish() {
                    return SubmissionOutcome::NotSubmitted(e);
                }
                state.outstanding |= mask;
                if polled {
                    state.polled |= mask;
                } else {
                    state.polled &= !mask;
                }
                barrier::dma_wmb();
                reg.ring_utrl_doorbell(tag);
                SubmissionOutcome::Submitted
            }
            UfsHwQueueKind::Mcq {
                reg, dma, queue, ..
            } => queue.submit(reg, dma, tag, publish),
        }
    }

    fn poll(&self, completed: &mut CompletedRequests) -> Result<()> {
        match &self.inner {
            UfsHwQueueKind::Sdb { reg, state } => {
                SdbTransferBackend::collect_state_completions(
                    reg,
                    state,
                    SdbCompletionSource::Poll,
                    completed,
                )
            }
            UfsHwQueueKind::Mcq {
                reg,
                dma,
                queue,
                poll,
            } => {
                if !poll {
                    return Err(EINVAL);
                }
                queue.collect_completions(reg, dma, completed)
            }
        }
    }
}

trait UfsTransferOps: Send + Sync {
    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>>;
    fn compose_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag) -> Result<()>;
    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>>;
    fn dump_state(&self, tag: usize, reason: &str);
    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()>;
    fn fetch_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag, cqe: Option<CqEntry>)
        -> Result<UfsCmd>;
    fn fetch_scsi_completion(&self, task_tag: TaskTag, cqe: Option<CqEntry>) -> UfsScsiResult;
}

impl SdbTransferBackend {
    fn new(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        let state = Arc::pin_init(
            pin_init!(SdbTransferState {
                completion <- new_spinlock_irq!(SdbCompletionState::default()),
            }),
            GFP_KERNEL,
        )?;

        Ok(Self { reg, dma, state })
    }

    fn tag_mask(tag: u32) -> Option<u32> {
        u32::try_from(tag).ok().and_then(|tag| 1u32.checked_shl(tag))
    }

    fn collect_completions(&self, requests: &mut CompletedRequests) -> Result<()> {
        Self::collect_state_completions(
            &self.reg,
            &self.state,
            SdbCompletionSource::Interrupt,
            requests,
        )
    }

    fn collect_state_completions(
        reg: &UfsReg,
        state: &SdbTransferState,
        source: SdbCompletionSource,
        requests: &mut CompletedRequests,
    ) -> Result<()> {
        let mut state = state.completion.lock();
        let doorbell = reg.read_utrl_doorbell();
        let completed = !doorbell & state.outstanding;
        let eligible = match source {
            SdbCompletionSource::Interrupt => completed & !state.polled,
            SdbCompletionSource::Poll => completed & state.polled,
        };
        if eligible != 0 {
            barrier::dma_rmb();
        }
        let collected = requests.insert_sdb_mask(eligible)?;

        state.outstanding &= !collected;
        state.polled &= !collected;
        Ok(())
    }
}

impl UfsTransferOps for SdbTransferBackend {
    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        let mut queues = KVec::new();
        queues.push(
            UfsHwQueue {
                inner: UfsHwQueueKind::Sdb {
                    reg: self.reg.clone(),
                    state: self.state.clone(),
                },
            },
            GFP_KERNEL,
        )?;
        Ok(queues)
    }

    fn compose_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag) -> Result<()> {
        self.dma
            .compose_devman_upiu(cmd, u32::from(task_tag.value()))
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        self.dma
            .compose_scsi_upiu(rq, cmd, task_tag.value(), mempool)
    }

    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        SdbTransferBackend::collect_completions(self, completed)
    }

    fn fetch_dev(
        &self,
        cmd: UfsDevCmd,
        task_tag: TaskTag,
        _cqe: Option<CqEntry>,
    ) -> Result<UfsCmd> {
        self.dma.fetch_devman_upiu(cmd, task_tag.index())
    }

    fn fetch_scsi_completion(&self, task_tag: TaskTag, _cqe: Option<CqEntry>) -> UfsScsiResult {
        self.dma.fetch_scsi_completion(task_tag.index())
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        let state = self.state.completion.lock();

        pr_err!(
            "[RUFS] ufs_queue: SDB dump reason={} tag={} outstanding=0x{:x} polled=0x{:x}\n",
            reason,
            tag,
            state.outstanding,
            state.polled,
        );
    }
}

impl McqTransferBackend {
    fn new(config: McqConfig, reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        let oprs = reg.mcq_default_opr_set()?;
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
            queues,
        })
    }

    fn queue_depth(&self) -> usize {
        self.config.max_active_commands
    }

    fn allocated_queues(&self) -> usize {
        self.queues.len()
    }

    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        let mut hw_queues = KVec::new();
        for queue in self.queues.queues.iter() {
            let id = queue.descriptor.id() as usize;
            hw_queues.push(
                UfsHwQueue {
                    inner: UfsHwQueueKind::Mcq {
                        reg: self.reg.clone(),
                        dma: self.dma.clone(),
                        queue: queue.clone(),
                        poll: self.config.is_poll_queue(id),
                    },
                },
                GFP_KERNEL,
            )?;
        }
        Ok(hw_queues)
    }

    fn prepare(&self) -> Result<()> {
        self.queues
            .configure_registers_with_interrupt_queues(&self.reg, self.config.interrupt_queues)
    }

    fn enable(&self) {
        self.reg.enable_mcq_mode()
    }

    fn activate(&self) -> Result<()> {
        self.prepare()?;
        self.reg.config_mcq_max_active_cmds(
            u32::try_from(self.queue_depth()).map_err(|_| EOVERFLOW)?,
        )?;
        self.enable();
        self.reg.enable_mcq_interrupts();
        Ok(())
    }

    fn compose_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag) -> Result<()> {
        self.dma
            .compose_devman_upiu(cmd, u32::from(task_tag.value()))
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        self.dma
            .compose_scsi_upiu(rq, cmd, task_tag.value(), mempool)
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

    fn fetch_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag, cqe: Option<CqEntry>) -> Result<UfsCmd> {
        self.dma
            .fetch_mcq_devman_upiu(cmd, task_tag.index(), cqe.ok_or(EIO)?)
    }

    fn fetch_scsi_completion(&self, task_tag: TaskTag, cqe: Option<CqEntry>) -> UfsScsiResult {
        match cqe {
            Some(cqe) => self.dma.fetch_mcq_scsi_completion(task_tag.index(), cqe),
            None => UfsScsiResult::error(0xf),
        }
    }
}

impl UfsTransferOps for McqTransferBackend {
    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        McqTransferBackend::hw_queues(self)
    }

    fn compose_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag) -> Result<()> {
        McqTransferBackend::compose_dev(self, cmd, task_tag)
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        McqTransferBackend::compose_scsi(self, cmd, rq, task_tag, mempool)
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        McqTransferBackend::dump_state(self, tag, reason);
    }

    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        McqTransferBackend::collect_completions(self, completed)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag, cqe: Option<CqEntry>) -> Result<UfsCmd> {
        McqTransferBackend::fetch_dev(self, cmd, task_tag, cqe)
    }

    fn fetch_scsi_completion(&self, task_tag: TaskTag, cqe: Option<CqEntry>) -> UfsScsiResult {
        McqTransferBackend::fetch_scsi_completion(self, task_tag, cqe)
    }

}

impl UfsTransferBackend {
    fn new(config: UfsTransferConfig, reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        let ops = match config {
            UfsTransferConfig::Sdb { .. } => {
                pr_info!("[RUFS] ufs_queue: use SDB backend\n");
                KBox::new(SdbTransferBackend::new(reg, dma)?, GFP_KERNEL)?
                    as KBox<dyn UfsTransferOps>
            }
            UfsTransferConfig::Mcq(config) => {
                let backend = McqTransferBackend::new(config, reg, dma)?;
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
                KBox::new(backend, GFP_KERNEL)? as KBox<dyn UfsTransferOps>
            }
        };

        Ok(Self { ops })
    }

    fn ops(&self) -> &dyn UfsTransferOps {
        &*self.ops
    }

    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        self.ops.hw_queues()
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
            let cmd = rq.data_ref().inner.lock().prepared_command()?.get_device()?;
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

        rq
            .data_ref()
            .inner
            .lock()
            .prepare_scsi(cmd, prdt)?;
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
            request
                .data_ref()
                .inner
                .lock()
                .mark_in_flight(queue_id)?;

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

    pub(crate) fn timeout(
        request_data: &UfsRequestData,
        queue_data: &QueueData,
        tag: u32,
    ) -> bool {
        let queue = queue_data.queue_arc().clone();
        let disposition = request_data.inner.lock().timeout();
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
        cqe: Option<CqEntry>,
    ) -> CompletionOutcome {
        Self::complete_with(rq, task_tag, queue_id, cqe, CompletionTarget::Direct)
    }

    fn complete_polled(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        cqe: Option<CqEntry>,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> CompletionOutcome {
        Self::complete_with(
            rq,
            task_tag,
            queue_id,
            cqe,
            CompletionTarget::Poll(batch),
        )
    }

    fn complete_with(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        task_tag: TaskTag,
        queue_id: u32,
        cqe: Option<CqEntry>,
        target: CompletionTarget<'_>,
    ) -> CompletionOutcome {
        let request_queue = rq.queue_data().queue_arc().clone();
        let (cmd, prdt) = match rq
            .data_ref()
            .inner
            .lock()
            .begin_completion(queue_id)
        {
            Ok(state) => state,
            Err(_) => {
                pr_err!(
                    "[RUFS] ufs_queue: completion for inactive request tag={}\n",
                    rq.tag(),
                );
                request_queue.require_recovery(
                    "completion for inactive request",
                    rq.tag() as usize,
                );
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
                let result = queue.fetch_dev(cmd, task_tag, cqe);
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
                let result = queue.fetch_scsi_completion(task_tag, cqe);
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
        reg: Arc<UfsReg>,
        dma: Arc<UfsDma>,
    ) -> Result<Arc<Self>> {
        let backend = UfsTransferBackend::new(config, reg, dma)?;
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
        core::cmp::max(1, active.div_ceil(COMPLETION_BATCH_SIZE))
    }

    fn recovery_required(&self) -> bool {
        self.recovery.lock().cause().is_some()
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
        self.backend.ops().compose_dev(cmd, task_tag)
    }

    fn compose_scsi(
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
        task_tag: TaskTag,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        let queue = rq.queue_data().queue();

        queue.backend.ops().compose_scsi(cmd, rq, task_tag, mempool)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, task_tag: TaskTag, cqe: Option<CqEntry>) -> Result<UfsCmd> {
        self.backend.ops().fetch_dev(cmd, task_tag, cqe)
    }

    fn fetch_scsi_completion(&self, task_tag: TaskTag, cqe: Option<CqEntry>) -> UfsScsiResult {
        self.backend.ops().fetch_scsi_completion(task_tag, cqe)
    }

    fn collect_backend_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        self.backend.ops().collect_completions(completed)
    }

    fn dump_backend_state(&self, tag: usize, reason: &str) {
        self.backend.ops().dump_state(tag, reason);
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
                    cqe: request.cqe,
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
            && boot_unit_attention(sense.as_ref());
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
                    rq.end(
                        u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR as u8),
                    );
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
            *state = RecoveryState::Recovering(cause);
        }

        // Controller reset and request disposition are added after the
        // recovery foundation is validated. Until then, keep the tag set
        // quiesced so a late completion cannot refer to a reused tag.
        pr_err!(
            "[RUFS] ufs_queue: recovery unavailable, queue stopped reason={} queue={:?} tag={}\n",
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
        *this.recovery.lock() = RecoveryState::Failed(cause);
    }
}
