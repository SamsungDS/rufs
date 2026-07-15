// SPDX-License-Identifier: GPL-2.0

use crate::ufs_dev::*;
use crate::ufs_dma::*;
use crate::ufs_lu::{QueueData, TagSetData, UfsLuBlockOps, UfsRequestData};
use crate::ufs_reg::*;
use kernel::alloc::mempool::MemPool;
use kernel::block::mq;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::block::mq::TagSet;
use kernel::cpu;
use kernel::sync::atomic::{Acquire, Atomic, Release};
use kernel::sync::{aref::ARef, barrier, Arc, SpinLock, SpinLockIrq};
use kernel::types::OwnableRefCounted;
use kernel::{bindings, new_spinlock, new_spinlock_irq, prelude::*};

const READ_10: u8 = 0x28;
const WRITE_10: u8 = 0x2a;
const SYNCHRONIZE_CACHE: u8 = 0x35;
const UNMAP: u8 = 0x42;
const READ_16: u8 = 0x88;
const WRITE_16: u8 = 0x8a;
const UFS_MCQ_DEFAULT_READ_QUEUES: usize = 0;
const UFS_MCQ_DEFAULT_POLL_QUEUES: usize = 1;
const UFS_TASK_TAG_COUNT: usize = 256;
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
    tag_count: usize,
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
        let tag_count = core::cmp::min(reg.nutrs_mcq(), UFS_TASK_TAG_COUNT);
        let ring_entries = tag_count.checked_add(1).ok_or(EOVERFLOW)?;
        if interrupt_queues == 0 || tag_count == 0 {
            return Err(EINVAL);
        }

        Ok(Self::Mcq(McqConfig {
            max_queues,
            total_queues,
            default_queues,
            read_queues,
            interrupt_queues,
            poll_queues,
            tag_count,
            ring_entries,
        }))
    }

    pub(crate) fn tag_count(&self) -> usize {
        match self {
            Self::Sdb { tag_count } => *tag_count,
            Self::Mcq(config) => config.tag_count,
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
pub(crate) struct UfsQueueMap {
    nr_hw_queues: usize,
    default_queues: usize,
    read_queues: usize,
    poll_queues: usize,
}

impl UfsQueueMap {
    fn sdb() -> Self {
        Self {
            nr_hw_queues: 1,
            default_queues: 1,
            read_queues: 0,
            poll_queues: 0,
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

        Ok(Self {
            nr_hw_queues,
            default_queues,
            read_queues,
            poll_queues,
        })
    }

    pub(crate) fn nr_hw_queues(&self) -> usize {
        self.nr_hw_queues
    }

    pub(crate) fn default_queues(&self) -> usize {
        self.default_queues
    }

    pub(crate) fn read_queues(&self) -> usize {
        self.read_queues
    }

    pub(crate) fn poll_queues(&self) -> usize {
        self.poll_queues
    }

    /// Number of blk-mq queue maps required to express this layout.
    pub(crate) fn num_maps(&self) -> u32 {
        if self.poll_queues > 0 {
            3
        } else if self.read_queues > 0 {
            2
        } else {
            1
        }
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
    },
    Recovering {
        cmd: UfsCmd,
        prdt: Option<UfsPrdtMapping>,
    },
    Completing {
        status: u32,
    },
    DeviceComplete(UfsDevCmd),
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

    fn mark_in_flight(&mut self) -> Result<()> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        self.state = match state {
            UfsRequestState::Prepared { cmd, prdt } => {
                UfsRequestState::InFlight { cmd, prdt }
            }
            state => {
                self.state = state;
                return Err(EIO);
            }
        };
        Ok(())
    }

    fn begin_completion(&mut self) -> Result<(UfsCmd, Option<UfsPrdtMapping>)> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::InFlight { cmd, prdt }
            | UfsRequestState::Recovering { cmd, prdt } => {
                self.state = UfsRequestState::Completing {
                    status: u32::from(bindings::BLK_STS_OK),
                };
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
            UfsRequestState::InFlight { cmd, prdt } => {
                self.state = UfsRequestState::Recovering { cmd, prdt };
                TimeoutDisposition::StartRecovery(cmd)
            }
            UfsRequestState::Recovering { cmd, prdt } => {
                self.state = UfsRequestState::Recovering { cmd, prdt };
                TimeoutDisposition::Recovering(cmd)
            }
            state => {
                self.state = state;
                TimeoutDisposition::Completed
            }
        }
    }

    fn complete_device(&mut self, cmd: UfsDevCmd) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Completing { .. }) {
            return Err(EIO);
        }
        self.state = UfsRequestState::DeviceComplete(cmd);
        Ok(())
    }

    fn set_completion_status(&mut self, status: u32) -> Result<()> {
        let UfsRequestState::Completing {
            status: completion_status,
        } = &mut self.state
        else {
            return Err(EIO);
        };
        *completion_status = status;
        Ok(())
    }

    pub(crate) fn finish_scheduled_completion(&mut self) -> Result<u32> {
        let state = core::mem::replace(&mut self.state, UfsRequestState::Idle);
        match state {
            UfsRequestState::Completing { status } => Ok(status),
            UfsRequestState::DeviceComplete(cmd) => {
                self.state = UfsRequestState::DeviceComplete(cmd);
                Ok(u32::from(bindings::BLK_STS_OK))
            }
            state => {
                self.state = state;
                Err(EIO)
            }
        }
    }

    fn finish_direct_completion(&mut self) -> Result<()> {
        if !matches!(self.state, UfsRequestState::Completing { .. }) {
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
}

#[pin_data]
struct SdbTransferState {
    #[pin]
    completion: SpinLockIrq<SdbCompletionState>,
}

#[derive(Clone, Copy)]
struct CompletedRequest {
    tag: usize,
    cqe: Option<CqEntry>,
}

#[derive(Clone, Copy)]
struct CompletionFault {
    reason: &'static str,
    tag: usize,
}

struct CompletedRequests {
    requests: [Option<CompletedRequest>; COMPLETION_BATCH_SIZE],
    len: usize,
    pos: usize,
    fault: Option<CompletionFault>,
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

    fn insert(&mut self, tag: usize, cqe: Option<CqEntry>) -> Result<()> {
        if self.len == self.requests.len() {
            return Err(ENOMEM);
        }

        self.requests[self.len] = Some(CompletedRequest { tag, cqe });
        self.len += 1;
        Ok(())
    }

    fn insert_sdb_mask(&mut self, mut mask: u32) -> Result<u32> {
        let mut inserted = 0;

        while mask != 0 && !self.is_full() {
            let tag = mask.trailing_zeros();
            let tag_mask = 1u32 << tag;
            mask &= !tag_mask;
            self.insert(tag as usize, None)?;
            inserted |= tag_mask;
        }

        Ok(inserted)
    }

    fn is_full(&self) -> bool {
        self.len == self.requests.len()
    }

    fn record_fault(&mut self, reason: &'static str, tag: usize) {
        if self.fault.is_none() {
            self.fault = Some(CompletionFault { reason, tag });
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

    fn submit(&self, reg: &UfsReg, dma: &UfsDma, tag: u32) -> Result<()> {
        let mut submission = self.submission.lock();
        if submission.is_full(reg, &self.descriptor)? {
            return Err(EBUSY);
        }

        let sqe = dma.transfer_request_desc(tag as usize)?;
        let tail = submission.write_entry(&self.descriptor, sqe)?;

        barrier::dma_wmb();
        reg.write_mcq_sq_tail(self.descriptor.oprs(), self.descriptor.id() as usize, tail)
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
                    match dma.tag_from_cq_entry(&cqe) {
                        Ok(tag) => completed_requests.insert(tag, Some(cqe))?,
                        Err(_) => completed_requests.record_fault(
                            "invalid MCQ completion tag",
                            usize::from(cqe.task_tag()),
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
                reg.mcq_opr_offset(descriptor.oprs(), UfsMcqOprRegion::Sqd, id, 0),
            )?;
            reg.write_mcq_sqisao(
                id,
                reg.mcq_opr_offset(descriptor.oprs(), UfsMcqOprRegion::Sqis, id, 0),
            )?;

            reg.set_mcq_cq_base_addr(id, cq_dma_addr)?;
            reg.write_mcq_cqdao(
                id,
                reg.mcq_opr_offset(descriptor.oprs(), UfsMcqOprRegion::Cqd, id, 0),
            )?;
            reg.write_mcq_cqisao(
                id,
                reg.mcq_opr_offset(descriptor.oprs(), UfsMcqOprRegion::Cqis, id, 0),
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

enum UfsTransferBackend {
    Sdb(SdbTransferBackend),
    Mcq(McqTransferBackend),
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

    fn submit(&self, tag: u32) -> Result<()> {
        match &self.inner {
            UfsHwQueueKind::Sdb { reg, state } => {
                let mask = SdbTransferBackend::tag_mask(tag).ok_or(EINVAL)?;
                let mut state = state.completion.lock();

                state.outstanding |= mask;
                barrier::dma_wmb();
                reg.ring_utrl_doorbell(tag);
                Ok(())
            }
            UfsHwQueueKind::Mcq {
                reg, dma, queue, ..
            } => queue.submit(reg, dma, tag),
        }
    }

    fn poll(&self, completed: &mut CompletedRequests) -> Result<()> {
        match &self.inner {
            UfsHwQueueKind::Sdb { reg, state } => {
                SdbTransferBackend::collect_state_completions(reg, state, completed)
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

trait UfsTransferOps {
    fn compose_dev(&self, cmd: UfsDevCmd, tag: u32) -> Result<()>;
    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>>;
    fn dump_state(&self, tag: usize, reason: &str);
    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()>;
    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize, cqe: Option<CqEntry>) -> Result<UfsCmd>;
    fn fetch_scsi_completion(&self, tag: usize, cqe: Option<CqEntry>) -> UfsScsiResult;
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
        Self::collect_state_completions(&self.reg, &self.state, requests)
    }

    fn collect_state_completions(
        reg: &UfsReg,
        state: &SdbTransferState,
        requests: &mut CompletedRequests,
    ) -> Result<()> {
        let mut state = state.completion.lock();
        let doorbell = reg.read_utrl_doorbell();
        let completed = !doorbell & state.outstanding;
        if completed != 0 {
            barrier::dma_rmb();
        }
        let collected = requests.insert_sdb_mask(completed)?;

        state.outstanding &= !collected;
        Ok(())
    }
}

impl UfsTransferOps for SdbTransferBackend {
    fn compose_dev(&self, cmd: UfsDevCmd, tag: u32) -> Result<()> {
        self.dma.compose_devman_upiu(cmd, tag)
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        self.dma.compose_scsi_upiu(rq, cmd, mempool)
    }

    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        SdbTransferBackend::collect_completions(self, completed)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize, _cqe: Option<CqEntry>) -> Result<UfsCmd> {
        self.dma.fetch_devman_upiu(cmd, tag)
    }

    fn fetch_scsi_completion(&self, tag: usize, _cqe: Option<CqEntry>) -> UfsScsiResult {
        self.dma.fetch_scsi_completion(tag)
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        let state = self.state.completion.lock();

        pr_err!(
            "[RUFS] ufs_queue: SDB dump reason={} tag={} outstanding=0x{:x}\n",
            reason,
            tag,
            state.outstanding,
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
        self.config.tag_count
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

    fn compose_dev(&self, cmd: UfsDevCmd, tag: u32) -> Result<()> {
        self.dma.compose_devman_upiu(cmd, tag)
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        self.dma.compose_scsi_upiu(rq, cmd, mempool)
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

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize, cqe: Option<CqEntry>) -> Result<UfsCmd> {
        self.dma
            .fetch_mcq_devman_upiu(cmd, tag, cqe.ok_or(EIO)?)
    }

    fn fetch_scsi_completion(&self, tag: usize, cqe: Option<CqEntry>) -> UfsScsiResult {
        match cqe {
            Some(cqe) => self.dma.fetch_mcq_scsi_completion(tag, cqe),
            None => UfsScsiResult::error(0xf),
        }
    }
}

impl UfsTransferOps for McqTransferBackend {
    fn compose_dev(&self, cmd: UfsDevCmd, tag: u32) -> Result<()> {
        McqTransferBackend::compose_dev(self, cmd, tag)
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        McqTransferBackend::compose_scsi(self, cmd, rq, mempool)
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        McqTransferBackend::dump_state(self, tag, reason);
    }

    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        McqTransferBackend::collect_completions(self, completed)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize, cqe: Option<CqEntry>) -> Result<UfsCmd> {
        McqTransferBackend::fetch_dev(self, cmd, tag, cqe)
    }

    fn fetch_scsi_completion(&self, tag: usize, cqe: Option<CqEntry>) -> UfsScsiResult {
        McqTransferBackend::fetch_scsi_completion(self, tag, cqe)
    }

}

impl UfsTransferBackend {
    fn new(config: UfsTransferConfig, reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        match config {
            UfsTransferConfig::Sdb { .. } => {
                pr_info!("[RUFS] ufs_queue: use SDB backend\n");
                Ok(Self::Sdb(SdbTransferBackend::new(reg, dma)?))
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
                Ok(Self::Mcq(backend))
            }
        }
    }

    fn ops(&self) -> &dyn UfsTransferOps {
        match self {
            Self::Sdb(backend) => backend,
            Self::Mcq(backend) => backend,
        }
    }

    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        match self {
            Self::Sdb(backend) => {
                let mut queues = KVec::new();
                queues.push(
                    UfsHwQueue {
                        inner: UfsHwQueueKind::Sdb {
                            reg: backend.reg.clone(),
                            state: backend.state.clone(),
                        },
                    },
                    GFP_KERNEL,
                )?;
                Ok(queues)
            }
            Self::Mcq(backend) => backend.hw_queues(),
        }
    }
}

impl UfsRequestData {
    pub(crate) fn compose_dev_request(rq: &ARef<mq::Request<UfsLuBlockOps>>) -> Result<()> {
        if let QueueData::Dev(queue) = rq.queue_data() {
            let cmd = rq.data_ref().inner.lock().prepared_command()?.get_device()?;
            queue.compose_dev(cmd, rq.tag())
        } else {
            Err(EIO)
        }
    }

    pub(crate) fn compose_scsi_cmd(
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
    ) -> Result<()> {
        let mempool = rq.queue().tag_set().data().dma_vec_mempool.clone();
        let prdt = UfsQueue::compose_scsi(rq, cmd, &mempool)?;

        rq.data_ref().inner.lock().prepare_scsi(cmd, prdt)
    }

    pub(crate) fn submit(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        hw_queue: &UfsHwQueue,
    ) -> core::result::Result<(), (ARef<mq::Request<UfsLuBlockOps>>, Error)> {
        let queue = match rq.queue_data() {
            QueueData::Dev(ufs_queue) => ufs_queue.clone(),
            QueueData::Lu(ufs_lu) => ufs_lu.queue.clone(),
        };
        let queue_id = hw_queue.id();
        let tag = rq.tag();

        if queue.recovery_required() {
            return Err((rq, EBUSY));
        }

        if rq.queue_index() != queue_id {
            return Err((rq, EINVAL));
        }

        let state_result = rq.data_ref().inner.lock().mark_in_flight();
        if let Err(e) = state_result {
            return Err((rq, e));
        }

        // Do not keep a submit-side request reference while making the command
        // visible to hardware. A fast completion may run before `queue_rq()`
        // returns and must be able to recover unique request ownership after
        // dropping the DMA mapping's request reference.
        drop(rq);

        match hw_queue.submit(tag) {
            Err(e) => {
                // A failed submission did not make the request visible to
                // hardware, so it is still owned by the driver and can be
                // recovered from its hctx and tag.
                let rq = queue
                    .tags
                    .tag_to_rq(queue_id, tag)
                    .expect("rufs: submitted request disappeared");
                rq.data_ref().inner.lock().reset();
                Err((rq, e))
            }
            Ok(()) => Ok(()),
        }
    }

    pub(crate) fn timeout(
        request_data: &UfsRequestData,
        queue_data: &QueueData,
        tag: u32,
    ) -> bool {
        let queue = match queue_data {
            QueueData::Dev(queue) => queue.clone(),
            QueueData::Lu(lu) => lu.queue.clone(),
        };
        queue.dump_backend_state(tag as usize, "request timeout");

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

    fn complete(rq: ARef<mq::Request<UfsLuBlockOps>>, cqe: Option<CqEntry>) -> bool {
        Self::complete_with(rq, cqe, CompletionTarget::Direct)
    }

    fn complete_polled(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        cqe: Option<CqEntry>,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> bool {
        Self::complete_with(rq, cqe, CompletionTarget::Poll(batch))
    }

    fn complete_with(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        cqe: Option<CqEntry>,
        target: CompletionTarget<'_>,
    ) -> bool {
        let request_queue = match rq.queue_data() {
            QueueData::Dev(queue) => queue.clone(),
            QueueData::Lu(lu) => lu.queue.clone(),
        };
        let (cmd, prdt) = match rq.data_ref().inner.lock().begin_completion() {
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
                return false;
            }
        };

        match cmd {
            UfsCmd::Device(cmd) => {
                let QueueData::Dev(queue) = rq.queue_data() else {
                    pr_err!("[RUFS] ufs_queue: device request has invalid context\n");
                    drop(prdt);
                    let status = u32::from(bindings::BLK_STS_IOERR);
                    let _ = rq.data_ref().inner.lock().set_completion_status(status);
                    return Self::end_device_request(rq, request_queue, status, false);
                };
                let result = queue.fetch_dev(cmd, rq.tag() as usize, cqe);
                drop(prdt);
                let (status, preserve_result) = match result {
                    Ok(UfsCmd::Device(cmd)) => {
                        if rq.data_ref().inner.lock().complete_device(cmd).is_err() {
                            pr_err!("[RUFS] ufs_queue: invalid device completion state\n");
                            (u32::from(bindings::BLK_STS_IOERR), false)
                        } else {
                            (u32::from(bindings::BLK_STS_OK), true)
                        }
                    }
                    _ => {
                        pr_err!("[RUFS] ufs_queue: failed to fetch device response\n");
                        (u32::from(bindings::BLK_STS_IOERR), false)
                    }
                };
                if !preserve_result {
                    let _ = rq.data_ref().inner.lock().set_completion_status(status);
                }
                Self::end_device_request(rq, request_queue, status, preserve_result)
            }
            UfsCmd::SCSI(cmd) => {
                let QueueData::Lu(lu) = rq.queue_data() else {
                    pr_err!("[RUFS] ufs_queue: SCSI request has invalid context\n");
                    drop(prdt);
                    let status = u32::from(bindings::BLK_STS_IOERR);
                    let _ = rq.data_ref().inner.lock().set_completion_status(status);
                    return Self::end_device_request(rq, request_queue, status, false);
                };
                let queue = &lu.queue;
                let result = queue.fetch_scsi_completion(rq.tag() as usize, cqe);
                drop(prdt);

                queue.clone().complete_scsi(cmd, result, rq, target);
                true
            }
        }
    }

    fn end_device_request(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        queue: Arc<UfsQueue>,
        status: u32,
        preserve_result: bool,
    ) -> bool {
        let tag = rq.tag();
        let rq = match OwnableRefCounted::try_from_shared(rq) {
            Ok(rq) => rq,
            Err(_rq) => {
                queue.require_recovery("device completion ownership conflict", tag as usize);
                return false;
            }
        };

        if !preserve_result && rq.data_ref().inner.lock().finish_direct_completion().is_err() {
            queue.require_recovery("invalid device completion state", tag as usize);
            return false;
        }
        rq.end(u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR as u8));
        true
    }
}

#[pin_data]
pub(crate) struct UfsQueue {
    pub(crate) tags: Arc<TagSet<UfsLuBlockOps>>,
    backend: UfsTransferBackend,
    recovery_required: Atomic<u32>,
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
        let blk_mq_tag_count = config.tag_count();
        if blk_mq_tag_count == 0 || nr_hw_queues == 0 || hw_queues.len() != nr_hw_queues {
            return Err(EINVAL);
        }

        let tagset_data = KBox::new(
            TagSetData {
                queue_map,
                hw_queues,
                // TODO: wrong depth
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
                u32::try_from(blk_mq_tag_count).map_err(|_| EOVERFLOW)?,
                queue_map.num_maps(),
                kernel::alloc::NumaNode::NO_NODE,
                tagset_flags,
            ),
            GFP_KERNEL,
        )?;

        let queue = Arc::pin_init(
            try_pin_init!(Self {
                tags <- tagset,
                backend,
                recovery_required: Atomic::new(0),
            }),
            GFP_KERNEL,
        )?;

        Ok(queue)
    }

    fn recovery_required(&self) -> bool {
        self.recovery_required.load(Acquire) != 0
    }

    pub(crate) fn require_recovery(&self, reason: &str, tag: usize) {
        if !self.recovery_required() {
            pr_err!(
                "[RUFS] ufs_queue: recovery required reason={} tag={}\n",
                reason,
                tag,
            );
        }
        self.recovery_required.store(1, Release);
    }

    // Issuing
    pub(crate) fn compose_dev(&self, cmd: UfsDevCmd, tag: u32) -> Result<()> {
        self.backend.ops().compose_dev(cmd, tag)
    }

    fn compose_scsi(
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        let queue = match rq.queue_data() {
            QueueData::Dev(ufs_queue) => ufs_queue,
            QueueData::Lu(ufs_lu) => &ufs_lu.queue,
        };

        queue.backend.ops().compose_scsi(cmd, rq, mempool)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize, cqe: Option<CqEntry>) -> Result<UfsCmd> {
        self.backend.ops().fetch_dev(cmd, tag, cqe)
    }

    fn fetch_scsi_completion(&self, tag: usize, cqe: Option<CqEntry>) -> UfsScsiResult {
        self.backend.ops().fetch_scsi_completion(tag, cqe)
    }

    fn collect_backend_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        self.backend.ops().collect_completions(completed)
    }

    fn dump_backend_state(&self, tag: usize, reason: &str) {
        self.backend.ops().dump_state(tag, reason);
    }

    fn request_at_shared_tag(
        &self,
        tag: u32,
    ) -> Result<Option<ARef<mq::Request<UfsLuBlockOps>>>> {
        // TagHctxShared makes every hctx point at the same tag map. The UFS
        // task tag is therefore sufficient request identity; CQ identity is
        // only needed while draining the hardware queue.
        self.tags.try_tag_to_rq(0, tag)
    }

    fn complete_tag(&self, request: CompletedRequest) -> bool {
        let tag = request.tag as u32;
        match self.request_at_shared_tag(tag) {
            Ok(Some(rq)) => UfsRequestData::complete(rq, request.cqe),
            Ok(None) => {
                self.require_recovery("completion tag has no request", request.tag);
                false
            }
            Err(_) => {
                self.require_recovery("completion request is not shareable", request.tag);
                false
            }
        }
    }

    fn complete_polled_tag(
        &self,
        request: CompletedRequest,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> bool {
        let tag = request.tag as u32;
        match self.request_at_shared_tag(tag) {
            Ok(Some(rq)) => UfsRequestData::complete_polled(rq, request.cqe, batch),
            Ok(None) => {
                self.require_recovery("polled completion tag has no request", request.tag);
                false
            }
            Err(_) => {
                self.require_recovery("polled request is not shareable", request.tag);
                false
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
        loop {
            let mut requests = CompletedRequests::new();
            let collect_result = self.collect_backend_completions(&mut requests);

            let batch_full = requests.is_full();
            while let Some(request) = requests.take_next() {
                any_completed |= self.complete_tag(request);
            }

            if let Some(fault) = requests.take_fault() {
                self.require_recovery(fault.reason, fault.tag);
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
        loop {
            let mut requests = CompletedRequests::new();
            let poll_result = hw_queue.poll(&mut requests);

            let batch_full = requests.is_full();
            while let Some(request) = requests.take_next() {
                any_completed |= self.complete_polled_tag(request, batch);
            }

            if let Some(fault) = requests.take_fault() {
                self.require_recovery(fault.reason, fault.tag);
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
        &self,
        cmd: UfsSCSICmd,
        result: UfsScsiResult,
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        target: CompletionTarget<'_>,
    ) {
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
        if rq
            .data_ref()
            .inner
            .lock()
            .set_completion_status(status)
            .is_err()
        {
            self.require_recovery("invalid SCSI completion state", tag as usize);
            return;
        }

        let rq = match OwnableRefCounted::try_from_shared(rq) {
            Ok(rq) => rq,
            Err(_rq) => {
                self.require_recovery("SCSI completion ownership conflict", tag as usize);
                return;
            }
        };
        if rq
            .data_ref()
            .inner
            .lock()
            .finish_direct_completion()
            .is_err()
        {
            self.require_recovery("invalid SCSI completion finalization", tag as usize);
            return;
        }

        if requeue {
            rq.requeue(true);
            return;
        }

        match target {
            CompletionTarget::Direct => rq.end(
                u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR as u8),
            ),
            CompletionTarget::Poll(batch) => {
                if status != u32::from(bindings::BLK_STS_OK) {
                    rq.end(
                        u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR as u8),
                    );
                    return;
                }

                if let Err(rq) = batch.add_request(rq, false) {
                    rq.end(status as u8);
                }
            }
        }
    }
}
