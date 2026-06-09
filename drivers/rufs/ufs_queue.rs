// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use kernel::block::mq;
use kernel::{bindings, prelude::*, kvec, new_mutex, new_spinlock};
use kernel::sync::{barrier, Arc, Completion, Mutex, SpinLock};
use kernel::types::{ARef, OwnableRefCounted};
use crate::ufs_reg::*;
use crate::ufs_dma::*;
use crate::ufs_irq::*;
use crate::ufs_dev::*;
use crate::ufs_lu::UfsLuBlockOps;

const READ_10: u8 = 0x28;
const WRITE_10: u8 = 0x2a;
const SYNCHRONIZE_CACHE: u8 = 0x35;
const UNMAP: u8 = 0x42;
const READ_16: u8 = 0x88;
const WRITE_16: u8 = 0x8a;
const MCQ_BASELINE_NR_QUEUES: usize = 1;

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
        UfsScsiCompletion::Busy
            | UfsScsiCompletion::TaskSetFull
            | UfsScsiCompletion::Requeue
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

        if blocks > u16::MAX as u32 || lba > u32::MAX as u64 {
            cdb[0] = if write { WRITE_16 } else { READ_16 };
            cdb[1] = flags;
            cdb[2..10].copy_from_slice(&lba.to_be_bytes());
            cdb[10..14].copy_from_slice(&blocks.to_be_bytes());
        } else {
            cdb[0] = if write { WRITE_10 } else { READ_10 };
            cdb[1] = flags;
            cdb[2..6].copy_from_slice(&(lba as u32).to_be_bytes());
            cdb[7..9].copy_from_slice(&(blocks as u16).to_be_bytes());
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

#[derive(PartialEq, Copy, Clone)]
enum RequestState {
    Idle,
    Issuing,
    Submitted,
    Completed,
}

struct UfsRequestInner {
    // These fields form one ownership unit. Keep them under a single lock so a
    // slot is never visible as idle while old DMA or block request state remains.
    cmd: Option<UfsCmd>,
    prdt: Option<UfsPrdtMapping>,
    block_rq: Option<ARef<mq::Request<UfsLuBlockOps>>>,
    // Decoded SCSI completion after the device has completed the request but
    // before blk-mq has accepted finalization.
    //
    // This is intentionally kept as request state instead of using
    // `mq::Request::complete()`. blk-mq may run `Operations::complete()` from
    // softirq context, and this kernel's Rust `SpinLock` API is not irqsave.
    // Letting that callback re-enter RUFS queue/request locks can deadlock if
    // it interrupts queue_rq or the threaded IRQ path while the same lock is
    // held. MCQ also makes the extra state necessary because CQE consumption is
    // destructive: once CQ head is advanced, the CQE cannot be fetched again if
    // `blk_mq_end_request()` temporarily fails due to outstanding `ARef`s.
    scsi_completion: Option<UfsScsiResult>,
    state: RequestState,
}

struct SdbTransferBackend {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
}

#[pin_data]
struct McqQueueSet {
    #[pin]
    queues: Mutex<Option<KVec<UfsMcqQueue>>>,

    #[pin]
    completed: SpinLock<KVec<Option<CqEntry>>>,
}

impl McqQueueSet {
    fn new(completed: KVec<Option<CqEntry>>) -> impl PinInit<Self> {
        pin_init!(Self {
            queues <- new_mutex!(None),
            completed <- new_spinlock!(completed),
        })
    }

    fn allocate(
        &self,
        dma: &UfsDma,
        nr_queues: usize,
        queue_depth: usize,
        oprs: UfsMcqOprSet,
    ) -> Result<()> {
        if nr_queues == 0 || queue_depth == 0 {
            return Err(EINVAL);
        }

        let mut queues = KVec::new();
        let queue_depth = u32::try_from(queue_depth).map_err(|_| EOVERFLOW)?;
        for id in 0..nr_queues {
            queues.push(
                UfsMcqQueue::new(
                    dma.dev(),
                    u32::try_from(id).map_err(|_| EOVERFLOW)?,
                    queue_depth,
                    oprs,
                )?,
                GFP_KERNEL,
            )?;
        }

        self.queues.lock().replace(queues);
        Ok(())
    }

    fn len(&self) -> usize {
        self.queues.lock().as_ref().map_or(0, |queues| queues.len())
    }

    fn submit(&self, reg: &UfsReg, dma: &UfsDma, tag: usize) -> Result<()> {
        {
            let mut completed = self.completed.lock();
            *completed.get_mut(tag).ok_or(EINVAL)? = None;
        }

        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;
        let queue = queues.get_mut(0).ok_or(EINVAL)?;
        let queue_id = queue.id() as usize;
        if queue.sq_is_full(reg)? {
            return Err(EBUSY);
        }

        let sqe = dma.transfer_request_desc(tag)?;
        let tail = queue.write_sq_entry(sqe)?;

        barrier::smp_wmb();
        reg.write_mcq_sq_tail(queue.oprs(), queue_id, tail)
    }

    fn poll_completions(&self, reg: &UfsReg, dma: &UfsDma) -> Result<()> {
        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;

        for queue in queues.iter_mut() {
            queue.update_cq_tail_slot(reg)?;
            while !queue.cq_is_empty() {
                if let Some(cqe) = queue.consume_cq_entry(reg)? {
                    let tag = dma.tag_from_cq_entry(&cqe)?;
                    let mut completed = self.completed.lock();
                    *completed.get_mut(tag).ok_or(EINVAL)? = Some(cqe);
                }
            }
            queue.acknowledge_cq_events(reg)?;
        }

        Ok(())
    }

    fn request_completed(&self, reg: &UfsReg, dma: &UfsDma, tag: usize) -> bool {
        if let Err(e) = self.poll_completions(reg, dma) {
            pr_err!(
                "[RUFS] ufs_queue: MCQ poll failed tag={} errno={}\n",
                tag,
                e.to_errno(),
            );
            return false;
        }

        self.completed
            .lock()
            .get(tag)
            .and_then(|cqe| cqe.as_ref())
            .is_some()
    }

    fn take_completion(&self, tag: usize) -> Option<CqEntry> {
        self.completed.lock().get_mut(tag).and_then(Option::take)
    }

    fn configure_registers(&self, reg: &UfsReg) -> Result<()> {
        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;

        for queue in queues.iter_mut() {
            let id = queue.id() as usize;
            let sq_dma_addr = queue.sqe_dma_addr() as u64;
            let cq_dma_addr = queue.cqe_dma_addr() as u64;

            reg.set_mcq_sq_base_addr(id, sq_dma_addr)?;
            reg.write_mcq_sqdao(
                id,
                reg.mcq_opr_offset(queue.oprs(), UfsMcqOprRegion::Sqd, id, 0),
            )?;
            reg.write_mcq_sqisao(
                id,
                reg.mcq_opr_offset(queue.oprs(), UfsMcqOprRegion::Sqis, id, 0),
            )?;

            reg.set_mcq_cq_base_addr(id, cq_dma_addr)?;
            reg.write_mcq_cqdao(
                id,
                reg.mcq_opr_offset(queue.oprs(), UfsMcqOprRegion::Cqd, id, 0),
            )?;
            reg.write_mcq_cqisao(
                id,
                reg.mcq_opr_offset(queue.oprs(), UfsMcqOprRegion::Cqis, id, 0),
            )?;

            queue.reset_slots();
            reg.enable_mcq_cq_tail_push_intr(queue.oprs(), id)?;
            reg.enable_mcq_cq(id, queue.max_entries() as usize)?;
            reg.enable_mcq_sq(id, queue.max_entries() as usize, id)?;
        }

        Ok(())
    }
}

struct McqTransferBackend {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    max_queues: usize,
    nr_queues: usize,
    queue_depth: usize,
    oprs: UfsMcqOprSet,
    queues: Arc<McqQueueSet>,
}

enum UfsTransferBackend {
    Sdb(SdbTransferBackend),
    Mcq(McqTransferBackend),
}

impl SdbTransferBackend {
    fn new(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Self {
        Self { reg, dma }
    }

    fn queue_depth(&self) -> usize {
        self.reg.nutrs()
    }

    fn nr_hw_queues(&self) -> usize {
        1
    }

    fn compose_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        self.dma.compose_devman_upiu(cmd, tag)
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        tag: usize,
        rq: &mq::Request<UfsLuBlockOps>,
    ) -> Result<Option<UfsPrdtMapping>> {
        self.dma.compose_scsi_upiu(cmd, tag, rq.as_raw())
    }

    fn submit_dev(&self, _cmd: UfsDevCmd, tag: usize) -> Result<()> {
        self.reg.ring_utrl_doorbell(tag);
        Ok(())
    }

    fn submit_scsi(&self, _cmd: UfsSCSICmd, tag: usize) -> Result<()> {
        self.reg.ring_utrl_doorbell(tag);
        Ok(())
    }

    fn request_completed(&self, tag: usize) -> bool {
        (self.reg.read_utrl_doorbell() & (1 << tag)) == 0
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        self.dma.fetch_devman_upiu(cmd, tag)
    }

    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        self.dma.fetch_scsi_completion(tag)
    }
}

impl McqTransferBackend {
    fn new(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        if !reg.mcq_supported() {
            return Err(ENOTSUPP);
        }

        let max_queues = reg.mcq_max_queues();
        if max_queues == 0 {
            return Err(EINVAL);
        }

        let nr_queues = core::cmp::min(max_queues, MCQ_BASELINE_NR_QUEUES);
        let queue_depth = core::cmp::min(reg.nutrs_mcq(), dma.transfer_slots());
        let oprs = reg.mcq_default_opr_set()?;
        let completed = kvec![None; queue_depth]?;
        let queues = Arc::pin_init(McqQueueSet::new(completed), GFP_KERNEL)?;
        queues.allocate(&dma, nr_queues, queue_depth, oprs)?;

        Ok(Self {
            reg,
            dma,
            max_queues,
            nr_queues,
            queue_depth,
            oprs,
            queues,
        })
    }

    fn max_queues(&self) -> usize {
        self.max_queues
    }

    fn nr_queues(&self) -> usize {
        self.nr_queues
    }

    fn queue_depth(&self) -> usize {
        self.queue_depth
    }

    fn nr_hw_queues(&self) -> usize {
        self.nr_queues
    }

    fn oprs(&self) -> &UfsMcqOprSet {
        &self.oprs
    }

    fn allocated_queues(&self) -> usize {
        self.queues.len()
    }

    fn prepare(&self) -> Result<()> {
        self.queues.configure_registers(&self.reg)
    }

    fn enable(&self) {
        self.reg.enable_mcq_mode()
    }

    fn compose_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        self.dma.compose_devman_upiu(cmd, tag)
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        tag: usize,
        rq: &mq::Request<UfsLuBlockOps>,
    ) -> Result<Option<UfsPrdtMapping>> {
        self.dma.compose_scsi_upiu(cmd, tag, rq.as_raw())
    }

    fn submit_dev(&self, _cmd: UfsDevCmd, tag: usize) -> Result<()> {
        self.queues.submit(&self.reg, &self.dma, tag)
    }

    fn submit_scsi(&self, _cmd: UfsSCSICmd, tag: usize) -> Result<()> {
        self.queues.submit(&self.reg, &self.dma, tag)
    }

    fn request_completed(&self, tag: usize) -> bool {
        self.queues.request_completed(&self.reg, &self.dma, tag)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        match self.queues.take_completion(tag) {
            Some(cqe) => self.dma.fetch_mcq_devman_upiu(cmd, tag, cqe),
            None => Err(EIO),
        }
    }

    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        match self.queues.take_completion(tag) {
            Some(cqe) => self.dma.fetch_mcq_scsi_completion(tag, cqe),
            None => UfsScsiResult::error(0xf),
        }
    }
}

impl UfsTransferBackend {
    fn sdb(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Self {
        Self::Sdb(SdbTransferBackend::new(reg, dma))
    }

    fn queue_depth(&self) -> usize {
        match self {
            Self::Sdb(backend) => backend.queue_depth(),
            Self::Mcq(backend) => backend.queue_depth(),
        }
    }

    fn nr_hw_queues(&self) -> usize {
        match self {
            Self::Sdb(backend) => backend.nr_hw_queues(),
            Self::Mcq(backend) => backend.nr_hw_queues(),
        }
    }

    fn compose_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        match self {
            Self::Sdb(backend) => backend.compose_dev(cmd, tag),
            Self::Mcq(backend) => backend.compose_dev(cmd, tag),
        }
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        tag: usize,
        rq: &mq::Request<UfsLuBlockOps>,
    ) -> Result<Option<UfsPrdtMapping>> {
        match self {
            Self::Sdb(backend) => backend.compose_scsi(cmd, tag, rq),
            Self::Mcq(backend) => backend.compose_scsi(cmd, tag, rq),
        }
    }

    fn submit_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        match self {
            Self::Sdb(backend) => backend.submit_dev(cmd, tag),
            Self::Mcq(backend) => backend.submit_dev(cmd, tag),
        }
    }

    fn submit_scsi(&self, cmd: UfsSCSICmd, tag: usize) -> Result<()> {
        match self {
            Self::Sdb(backend) => backend.submit_scsi(cmd, tag),
            Self::Mcq(backend) => backend.submit_scsi(cmd, tag),
        }
    }

    fn request_completed(&self, tag: usize) -> bool {
        match self {
            Self::Sdb(backend) => backend.request_completed(tag),
            Self::Mcq(backend) => backend.request_completed(tag),
        }
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        match self {
            Self::Sdb(backend) => backend.fetch_dev(cmd, tag),
            Self::Mcq(backend) => backend.fetch_dev(cmd, tag),
        }
    }

    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        match self {
            Self::Sdb(backend) => backend.fetch_scsi_completion(tag),
            Self::Mcq(backend) => backend.fetch_scsi_completion(tag),
        }
    }
}

#[pin_data]
pub(crate) struct UfsRequest {
    queue: Arc<UfsQueue>,
    tag: usize,

    #[pin]
    inner: SpinLock<UfsRequestInner>,
}

impl UfsRequest {
    fn new(queue: Arc<UfsQueue>, tag: usize) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                queue,
                tag,
                inner <- new_spinlock!(UfsRequestInner {
                    cmd: None,
                    prdt: None,
                    block_rq: None,
                    scsi_completion: None,
                    state: RequestState::Idle,
                }),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn issue(&self, cmd: UfsCmd) -> Result<UfsCmd> {
        self.compose(cmd)?;
        self.submit()?;
        self.wait()?;
        self.fetch()
    }

    pub(crate) fn compose(&self, cmd: UfsCmd) -> Result<()> {
        match cmd {
            UfsCmd::Device(cmd) => {
                {
                    let mut inner = self.inner.lock();
                    if inner.state != RequestState::Idle {
                        return Err(EBUSY);
                    }
                    inner.state = RequestState::Issuing;
                }

                if let Err(e) = self.queue.compose_dev(cmd, self.tag) {
                    self.clear();
                    return Err(e);
                }

                let mut inner = self.inner.lock();
                inner.prdt = None;
                inner.block_rq = None;
                inner.cmd = Some(UfsCmd::Device(cmd));
            },
            UfsCmd::SCSI(cmd) => return self.compose_scsi_cmd(cmd),
        }

        Ok(())
    }

    #[inline(never)]
    fn compose_scsi_cmd(&self, cmd: UfsSCSICmd) -> Result<()> {
        let block_rq = {
            let mut inner = self.inner.lock();
            if inner.state != RequestState::Idle {
                return Err(EBUSY);
            }

            let block_rq = inner.block_rq.as_ref().ok_or(EINVAL)?.clone();
            inner.state = RequestState::Issuing;
            block_rq
        };

        let prdt = match self.queue.compose_scsi(cmd, self.tag, &block_rq) {
            Ok(prdt) => prdt,
            Err(e) => {
                self.clear();
                return Err(e);
            },
        };

        let mut inner = self.inner.lock();
        inner.prdt = prdt;
        inner.cmd = Some(UfsCmd::SCSI(cmd));
        Ok(())
    }

    pub(crate) fn compose_block_request(
        &self,
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
    ) -> Result<()> {
        {
            let mut inner = self.inner.lock();
            if inner.state != RequestState::Idle {
                return Err(EBUSY);
            }
            inner.block_rq = Some(rq);
        }

        if let Err(e) = self.compose_scsi_cmd(cmd) {
            self.clear();
            return Err(e);
        }

        Ok(())
    }

    pub(crate) fn submit(&self) -> Result<()> {
        let cmd = match self.inner.lock().cmd {
            Some(cmd) => cmd,
            None => {
                pr_err!("no command in UfsRequest");
                return Err(EIO);
            },
        };

        let result = match cmd {
            UfsCmd::Device(cmd) => {
                self.queue.prepare_dev_wait();
                self.queue.submit_dev(cmd, self.tag)
            },
            UfsCmd::SCSI(cmd) => self.queue.submit_scsi(cmd, self.tag),
        };

        match result {
            Err(e) => {
                self.clear();
                Err(e)
            },
            Ok(()) => {
                self.inner.lock().state = RequestState::Submitted;
                if self.queue.request_completed(self.tag) {
                    self.queue.wake_completion_thread();
                }
                Ok(())
            }
        }
    }

    pub(crate) fn wait(&self) -> Result<()> {
        let cmd = match self.inner.lock().cmd {
            Some(cmd) => cmd,
            None => {
                pr_err!("no command in UfsRequest");
                return Err(EIO);
            },
        };

        if self.inner.lock().state == RequestState::Idle {
            pr_err!("UfsRequest is not submitted");
            return Err(EIO);
        }

        let result = match cmd {
            UfsCmd::Device(cmd) => self.queue.wait_dev(cmd, self.tag),
            UfsCmd::SCSI(_) => Err(ENOTSUPP),
        };

        match result {
            Err(e) => {
                self.clear();
                Err(e)
            },
            Ok(()) => Ok(()),
        }
    }

    pub(crate) fn fetch(&self) -> Result<UfsCmd> {
        let cmd = match self.inner.lock().cmd {
            Some(cmd) => cmd,
            None => {
                pr_err!("no command in UfsRequest");
                return Err(EIO);
            },
        };

        if self.inner.lock().state != RequestState::Completed {
            pr_err!("UfsRequest is not completed");
            return Err(EIO);
        }

        let result = match cmd {
            UfsCmd::Device(cmd) => self.queue.fetch_dev(cmd, self.tag),
            UfsCmd::SCSI(_) => Err(ENOTSUPP),
        };

        match result {
            Err(e) => {
                self.clear();
                Err(e)
            },
            Ok(cmd) => {
                self.clear();
                Ok(cmd)
            },
        }
    }

    pub(crate) fn clear(&self) {
        let mut inner = self.inner.lock();
        inner.prdt = None;
        inner.block_rq = None;
        inner.scsi_completion = None;
        inner.cmd = None;
        inner.state = RequestState::Idle;
    }

    fn scsi_completion_result(&self) -> UfsScsiResult {
        if let Some(result) = self.inner.lock().scsi_completion {
            return result;
        }

        let result = self.queue.fetch_scsi_completion(self.tag);
        let mut inner = self.inner.lock();
        inner.scsi_completion = Some(result);
        inner.state = RequestState::Completed;
        result
    }

    fn complete(&self) -> bool {
        let cmd = {
            let inner = self.inner.lock();
            match inner.cmd {
                Some(cmd) => cmd,
                None => {
                    pr_err!("No command in UfsRequest");
                    return true;
                },
            }
        };

        match cmd {
            UfsCmd::Device(cmd) => {
                self.inner.lock().state = RequestState::Completed;
                self.queue.complete_dev(cmd, self.tag);
                true
            },
            UfsCmd::SCSI(cmd) => {
                let result = self.scsi_completion_result();
                let (block_rq, prdt) = {
                    let mut inner = self.inner.lock();
                    let Some(block_rq) = inner.block_rq.take() else {
                        pr_err!("No block request for SCSI completion tag {}", self.tag);
                        return true;
                    };

                    (block_rq, inner.prdt.take())
                };

                drop(prdt);

                match self.queue.complete_scsi(cmd, self.tag, result, block_rq) {
                    Ok(()) => {
                        self.clear();
                        true
                    },
                    Err(block_rq) => {
                        pr_debug!(
                            "[RUFS] ufs_queue: SCSI completion retry pending tag={}\n",
                            self.tag,
                        );
                        let mut inner = self.inner.lock();
                        inner.block_rq = Some(block_rq);
                        inner.state = RequestState::Completed;
                        false
                    },
                }
            },
        }
    }

    fn completion_ready(&self) -> bool {
        if self.inner.lock().state == RequestState::Completed {
            return true;
        }

        self.queue.request_completed(self.tag)
    }
}

#[pin_data]
pub(crate) struct UfsQueue {
    irq: Arc<UfsIrq>,

    #[pin]
    backend: SpinLock<UfsTransferBackend>,

    #[pin]
    slot: SpinLock<KVec<Option<Arc<UfsRequest>>>>,

    #[pin]
    completion: Completion,
}

impl UfsQueue {
    pub(crate) fn new(
        reg: Arc<UfsReg>,
        irq: Arc<UfsIrq>,
        dma: Arc<UfsDma>,
    ) -> Result<Arc<Self>> {
        if reg.mcq_supported() {
            pr_info!(
                "[RUFS] ufs_queue: MCQ supported by controller mcq_depth={}\n",
                reg.nutrs_mcq(),
            );
        }

        // The request table is sized for the allocation, while each backend
        // reports the tag range that is legal for that transport.
        let slot = kvec![None; dma.transfer_slots()]?;
        let backend = UfsTransferBackend::sdb(reg, dma);

        Arc::pin_init(
            try_pin_init!(Self {
                irq,
                backend <- new_spinlock!(backend),
                slot <- new_spinlock!(slot),
                completion <- Completion::new(),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn enable_mcq_backend(
        &self,
        reg: Arc<UfsReg>,
        dma: Arc<UfsDma>,
    ) -> Result<()> {
        let backend = McqTransferBackend::new(reg, dma)?;
        backend.prepare()?;
        backend
            .reg
            .config_mcq_max_active_cmds(backend.queue_depth() as u32)?;
        backend.enable();
        backend.reg.enable_mcq_interrupts();

        *self.backend.lock() = UfsTransferBackend::Mcq(backend);
        pr_info!("[RUFS] ufs_queue: MCQ backend enabled\n");
        Ok(())
    }

    pub(crate) fn nr_hw_queues(&self) -> usize {
        self.backend.lock().nr_hw_queues()
    }

    pub(crate) fn queue_depth(&self) -> usize {
        self.backend.lock().queue_depth()
    }

    pub(crate) fn reserve(self: &Arc<Self>) -> Result<Arc<UfsRequest>> {
        let queue_depth = self.queue_depth();
        if queue_depth == 0 {
            return Err(EINVAL);
        }

        let mut slots = self.slot.lock();
        let mut tag = queue_depth - 1;
        while let Some(slot) = slots.get_mut(tag) {
            match slot {
                Some(_) => { tag -= 1; },
                None => {
                    let request = UfsRequest::new(self.clone(), tag)?;
                    slot.replace(request.clone());
                    return Ok(request);
                },
            }
        }

        Err(ENOMEM)
    }

    pub(crate) fn acquire(
        self: &Arc<Self>,
        tag: usize,
    ) -> Result<Arc<UfsRequest>> {
        let mut binding = self.slot.lock();
        let slot = match binding.get_mut(tag) {
            Some(slot) => slot,
            None => {
                pr_err!("No slot for tag {}", tag);
                return Err(EINVAL);
            }
        };

        match slot {
            Some(request) => {
                if request.inner.lock().state == RequestState::Idle {
                    Ok(request.clone())
                } else {
                    Err(EBUSY)
                }
            },
            None => {
                let request = UfsRequest::new(self.clone(), tag)?;
                slot.replace(request.clone());
                Ok(request)
            },
        }
    }

    // Issuing
    fn compose_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        self.backend.lock().compose_dev(cmd, tag)
    }

    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        tag: usize,
        rq: &mq::Request<UfsLuBlockOps>,
    ) -> Result<Option<UfsPrdtMapping>> {
        self.backend.lock().compose_scsi(cmd, tag, rq)
    }

    fn submit_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        self.backend.lock().submit_dev(cmd, tag)
    }

    fn submit_scsi(&self, cmd: UfsSCSICmd, tag: usize) -> Result<()> {
        self.backend.lock().submit_scsi(cmd, tag)
    }

    fn prepare_dev_wait(&self) {
        self.completion.reinit();
    }

    fn wait_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        match self.completion.wait_for_completion_timeout(cmd.timeout()) {
            0 => Err(ETIMEDOUT),
            _ => Ok(()),
        }
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        self.backend.lock().fetch_dev(cmd, tag)
    }

    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        self.backend.lock().fetch_scsi_completion(tag)
    }

    fn request_completed(&self, tag: usize) -> bool {
        self.backend.lock().request_completed(tag)
    }

    // Completion
    fn wake_completion_thread(&self) {
        self.irq.wake_queue_thread();
    }

    fn next_request(&self, mut tag: usize) -> Option<Arc<UfsRequest>> {
        while let Some(slot) = self.slot.lock().get_mut(tag) {
            match slot {
                Some(request) => { return Some(request.clone()); },
                None => { tag += 1; },
            }
        }
        None
    }

    fn next_completable_request(&self, mut tag: usize) -> Option<Arc<UfsRequest>> {
        while let Some(request) = self.next_request(tag) {
            match request.inner.lock().state {
                RequestState::Submitted | RequestState::Completed => {
                    return Some(request.clone());
                },
                _ => { tag += 1; },
            }
        }
        None
    }

    pub(crate) fn complete(self: &Arc<Self>) {
        // Transfer completion must not run in the hard IRQ handler.
        //
        // The Rust lock API exposed by this kernel only provides plain
        // `SpinLock::lock()`, not an irqsave guard. Completion takes request
        // and DMA locks that are also used from submission context, then hands
        // the request back to blk-mq. Running that path directly from hard IRQ
        // context could deadlock if the interrupt arrives while the same CPU
        // already holds one of those locks. The queue IRQ is therefore
        // registered as a threaded IRQ. Submit-side fast completion and retry
        // paths wake that same IRQ thread instead of using a separate work
        // item, so there is a single completion executor.
        let mut tag = 0 as usize;
        let mut retry = false;
        while let Some(request) = self.next_completable_request(tag) {
            let request_tag = request.tag;
            if request.completion_ready() {
                if !request.complete() {
                    retry = true;
                }
            }
            tag = request_tag + 1;
        }

        if retry {
            self.wake_completion_thread();
        }
    }

    fn complete_dev(&self, cmd: UfsDevCmd, tag: usize) {
        self.completion.complete();
    }

    fn complete_scsi(
        &self,
        cmd: UfsSCSICmd,
        tag: usize,
        result: UfsScsiResult,
        rq: ARef<mq::Request<UfsLuBlockOps>>,
    ) -> Result<(), ARef<mq::Request<UfsLuBlockOps>>> {
        let sense_len = result.sense_data_len.min(result.sense_data.len());
        let sense = parse_scsi_sense(&result.sense_data, sense_len);
        let suppress_log = matches!(result.completion, UfsScsiCompletion::CheckCondition)
            && boot_unit_attention(sense.as_ref());
        let requeue = should_requeue_scsi(result.completion, sense.as_ref());

        if !matches!(result.completion, UfsScsiCompletion::Good) && !suppress_log {
            let cdb = cmd.cdb();
            pr_err!(
                "SCSI request completion error: tag={} lun={} opcode=0x{:02x} dir={:?} data_len={} completion={:?} ocs=0x{:x} transaction=0x{:02x} response=0x{:02x} status=0x{:02x} residual={} cdb={:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}\n",
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
                    "SCSI sense: tag={} response_code=0x{:02x} sense_key=0x{:x}({}) asc=0x{:02x} ascq=0x{:02x} additional_len={}\n",
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
                    "SCSI sense: tag={} unable to parse sense_len={} raw={:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}\n",
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
                pr_err!("SCSI sense: tag={} no sense data reported\n", tag);
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
            },
            UfsScsiCompletion::Error => bindings::BLK_STS_IOERR,
        };

        // Take exclusive ownership so the request can be handed back to the
        // block layer. If other references are still outstanding, return the
        // request so the caller can retry the completion later.
        let rq = match OwnableRefCounted::try_from_shared(rq) {
            Ok(rq) => rq,
            Err(rq) => return Err(rq),
        };

        if requeue {
            // Hand the started request back to the block layer for a retry. The
            // block layer takes ownership, so forget our reference.
            let ptr = rq.as_raw();
            core::mem::forget(rq);
            // SAFETY: `ptr` came from a request we owned exclusively, and
            // forgetting the `Owned` transfers ownership to the requeue path.
            unsafe { bindings::blk_mq_requeue_request(ptr, true) };
        } else {
            rq.end(status as u8);
        }

        Ok(())
    }
}
