// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use kernel::block::mq;
use kernel::cpu;
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
const UFS_MCQ_DEFAULT_READ_QUEUES: usize = 0;
const UFS_MCQ_DEFAULT_POLL_QUEUES: usize = 1;

fn possible_cpus() -> usize {
    (cpu::nr_cpu_ids() as usize).max(1)
}

#[derive(Copy, Clone)]
struct McqQueueLayout {
    max_queues: usize,
    total_queues: usize,
    default_queues: usize,
    read_queues: usize,
    interrupt_queues: usize,
    poll_queues: usize,
}

impl McqQueueLayout {
    fn sdb() -> Self {
        Self {
            max_queues: 1,
            total_queues: 1,
            default_queues: 1,
            read_queues: 0,
            interrupt_queues: 1,
            poll_queues: 0,
        }
    }

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
pub(crate) struct UfsQueueMap {
    nr_hw_queues: usize,
    default_queues: usize,
    read_queues: usize,
    poll_queues: usize,
}

impl UfsQueueMap {
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

fn ufs_mcq_queue_layout(reg: &UfsReg) -> Result<McqQueueLayout> {
    if !reg.mcq_supported() {
        return Ok(McqQueueLayout::sdb());
    }

    let hba_maxq = reg.mcq_max_queues();
    if hba_maxq == 0 {
        return Err(EINVAL);
    }

    // Match the C driver's default MCQ policy: rw_queues defaults to the
    // possible CPU count, read_queues defaults to 0, and poll_queues defaults
    // to 1. The resulting layout is exposed to blk-mq through an explicit
    // queue-map configuration when LUs are allocated.
    let read_queues = UFS_MCQ_DEFAULT_READ_QUEUES;
    let poll_queues = UFS_MCQ_DEFAULT_POLL_QUEUES;
    let requested_queues = read_queues + poll_queues;

    if hba_maxq < requested_queues || hba_maxq == poll_queues {
        return Err(ENOTSUPP);
    }

    let cpu_queues = possible_cpus();
    let remaining = hba_maxq
        .checked_sub(poll_queues)
        .and_then(|remaining| remaining.checked_sub(read_queues))
        .ok_or(EINVAL)?;
    let default_queues = core::cmp::min(remaining, cpu_queues);

    let interrupt_queues = default_queues + read_queues;
    let total_queues = interrupt_queues + poll_queues;
    if total_queues == 0 || interrupt_queues == 0 {
        Err(EINVAL)
    } else {
        Ok(McqQueueLayout {
            max_queues: hba_maxq,
            total_queues,
            default_queues,
            read_queues,
            interrupt_queues,
            poll_queues,
        })
    }
}

pub(crate) fn ufs_mcq_interrupt_queue_count(reg: &UfsReg) -> Result<usize> {
    Ok(ufs_mcq_queue_layout(reg)?.interrupt_queues)
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

        match (u32::try_from(lba), u16::try_from(blocks)) {
            (Ok(lba), Ok(blocks)) => {
                cdb[0] = if write { WRITE_10 } else { READ_10 };
                cdb[1] = flags;
                cdb[2..6].copy_from_slice(&lba.to_be_bytes());
                cdb[7..9].copy_from_slice(&blocks.to_be_bytes());
            },
            _ => {
                cdb[0] = if write { WRITE_16 } else { READ_16 };
                cdb[1] = flags;
                cdb[2..10].copy_from_slice(&lba.to_be_bytes());
                cdb[10..14].copy_from_slice(&blocks.to_be_bytes());
            },
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
    hw_queue: Option<usize>,
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

    fn queue_index(queue_hint: Option<usize>, tag: usize, nr_queues: usize) -> Result<usize> {
        if nr_queues == 0 {
            return Err(EINVAL);
        }

        Ok(queue_hint.unwrap_or(tag) % nr_queues)
    }

    fn submit(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        tag: usize,
        queue_hint: Option<usize>,
    ) -> Result<()> {
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
        let queue_index = Self::queue_index(queue_hint, tag, queues.len())?;
        let queue = queues.get_mut(queue_index).ok_or(EINVAL)?;
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
            self.collect_queue_completions(reg, dma, queue)?;
        }

        Ok(())
    }

    fn poll_completions_for_queue(&self, reg: &UfsReg, dma: &UfsDma, queue_id: usize) -> Result<()> {
        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;
        let queue = queues.get_mut(queue_id).ok_or(EINVAL)?;

        self.collect_queue_completions(reg, dma, queue)
    }

    fn collect_queue_completions(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        queue: &mut UfsMcqQueue,
    ) -> Result<()> {
        queue.update_cq_tail_slot(reg)?;
        while !queue.cq_is_empty() {
            if let Some(cqe) = queue.consume_cq_entry(reg)? {
                let tag = dma.tag_from_cq_entry(&cqe)?;
                let mut completed = self.completed.lock();
                *completed.get_mut(tag).ok_or(EINVAL)? = Some(cqe);
            }
        }
        queue.acknowledge_cq_events(reg)?;

        Ok(())
    }

    fn completion_cached(&self, tag: usize) -> bool {
        self.completed
            .lock()
            .get(tag)
            .and_then(|cqe| cqe.as_ref())
            .is_some()
    }

    fn request_completed(&self, reg: &UfsReg, dma: &UfsDma, tag: usize) -> bool {
        if let Err(e) = self.poll_completions(reg, dma) {
            pr_err!(
                "[RUFS] ufs_queue: MCQ poll failed tag={} errno={}\n",
                tag,
                e.to_errno(),
            );
            self.dump_state(reg, tag, "poll failed");
            return false;
        }

        self.completion_cached(tag)
    }

    fn take_completion(&self, tag: usize) -> Option<CqEntry> {
        self.completed.lock().get_mut(tag).and_then(Option::take)
    }

    fn dump_state(&self, reg: &UfsReg, tag: usize, reason: &str) {
        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only reads queue state in
        // place and never moves queues out of the vector or grows it.
        let Some(queues) = (unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }).as_mut()
        else {
            pr_err!(
                "[RUFS] ufs_queue: MCQ dump reason={} tag={} queues=unallocated\n",
                reason,
                tag,
            );
            return;
        };

        for queue in queues.iter() {
            let id = queue.id() as usize;
            let sq_head = reg.read_mcq_sq_head(queue.oprs(), id).unwrap_or(u32::MAX);
            let sq_tail = reg.read_mcq_sq_tail(queue.oprs(), id).unwrap_or(u32::MAX);
            let cq_head = reg.read_mcq_cq_head(queue.oprs(), id).unwrap_or(u32::MAX);
            let cq_tail = reg.read_mcq_cq_tail(queue.oprs(), id).unwrap_or(u32::MAX);
            let cqis = reg.read_mcq_cqis(queue.oprs(), id).unwrap_or(u32::MAX);

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
                queue.sq_tail_slot(),
                queue.cq_head_slot(),
                queue.cq_tail_slot(),
            );
        }
    }

    fn configure_registers_with_interrupt_queues(
        &self,
        reg: &UfsReg,
        interrupt_queues: usize,
    ) -> Result<()> {
        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;
        if interrupt_queues > queues.len() {
            return Err(EINVAL);
        }

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
            if id < interrupt_queues {
                reg.enable_mcq_cq_tail_push_intr(queue.oprs(), id)?;
            }
            reg.enable_mcq_cq(id, queue.max_entries() as usize)?;
            reg.enable_mcq_sq(id, queue.max_entries() as usize, id)?;
        }

        Ok(())
    }
}

struct McqTransferBackend {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,
    layout: McqQueueLayout,
    queue_depth: usize,
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

    // SDB has no destructive completion queue entry to snapshot, so this is a
    // no-op. MCQ uses the same backend interface to separate CQ consumption
    // from request finalization.
    fn refresh_completions(&self) -> Result<()> {
        Ok(())
    }

    fn poll_queue(&self, _queue: usize) -> Result<()> {
        Ok(())
    }

    fn completion_cached(&self, tag: usize) -> bool {
        self.request_completed(tag)
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

        let layout = ufs_mcq_queue_layout(&reg)?;
        let queue_depth = core::cmp::min(reg.nutrs_mcq(), dma.transfer_slots());
        let oprs = reg.mcq_default_opr_set()?;
        let completed = kvec![None; queue_depth]?;
        let queues = Arc::pin_init(McqQueueSet::new(completed), GFP_KERNEL)?;
        queues.allocate(&dma, layout.total_queues, queue_depth, oprs)?;

        Ok(Self {
            reg,
            dma,
            layout,
            queue_depth,
            queues,
        })
    }

    fn max_queues(&self) -> usize {
        self.layout.max_queues
    }

    fn nr_queues(&self) -> usize {
        self.layout.total_queues
    }

    fn queue_depth(&self) -> usize {
        self.queue_depth
    }

    fn default_queues(&self) -> usize {
        self.layout.default_queues
    }

    fn poll_queues(&self) -> usize {
        self.layout.poll_queues
    }

    fn allocated_queues(&self) -> usize {
        self.queues.len()
    }

    fn prepare(&self) -> Result<()> {
        self.queues
            .configure_registers_with_interrupt_queues(&self.reg, self.layout.interrupt_queues)
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
        self.queues.submit(&self.reg, &self.dma, tag, Some(0))
    }

    fn submit_scsi(&self, _cmd: UfsSCSICmd, tag: usize, hw_queue: Option<usize>) -> Result<()> {
        self.queues.submit(&self.reg, &self.dma, tag, hw_queue)
    }

    fn request_completed(&self, tag: usize) -> bool {
        self.queues.request_completed(&self.reg, &self.dma, tag)
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        self.queues.dump_state(&self.reg, tag, reason);
    }

    // MCQ completion is intentionally split into two phases. Reading a CQE is
    // destructive because the software CQ head advances, so the IRQ-thread
    // executor first snapshots all visible CQEs into `completed`. It then scans
    // requests using only the cached CQE state. This avoids repeatedly walking
    // every MCQ CQ for each submitted request, and it also preserves CQEs if
    // blk-mq cannot accept final completion immediately due to outstanding
    // request references. This is not a legacy SDB completion path; it mirrors
    // the MCQ requirement that CQ consumption and request finalization are
    // separate operations.
    fn refresh_completions(&self) -> Result<()> {
        self.queues.poll_completions(&self.reg, &self.dma)
    }

    fn poll_queue(&self, queue: usize) -> Result<()> {
        if !self.layout.is_poll_queue(queue) {
            return Err(EINVAL);
        }

        self.queues.poll_completions_for_queue(&self.reg, &self.dma, queue)
    }

    fn completion_cached(&self, tag: usize) -> bool {
        self.queues.completion_cached(tag)
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

    fn queue_map(&self) -> Result<UfsQueueMap> {
        match self {
            Self::Sdb(_) => McqQueueLayout::sdb().queue_map(),
            Self::Mcq(backend) => backend.layout.queue_map(),
        }
    }

    fn default_queues(&self) -> usize {
        match self {
            Self::Sdb(_) => 1,
            Self::Mcq(backend) => backend.default_queues(),
        }
    }

    fn poll_queues(&self) -> usize {
        match self {
            Self::Sdb(_) => 0,
            Self::Mcq(backend) => backend.poll_queues(),
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

    fn submit_scsi(&self, cmd: UfsSCSICmd, tag: usize, hw_queue: Option<usize>) -> Result<()> {
        match self {
            Self::Sdb(backend) => backend.submit_scsi(cmd, tag),
            Self::Mcq(backend) => backend.submit_scsi(cmd, tag, hw_queue),
        }
    }

    fn request_completed(&self, tag: usize) -> bool {
        match self {
            Self::Sdb(backend) => backend.request_completed(tag),
            Self::Mcq(backend) => backend.request_completed(tag),
        }
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        match self {
            Self::Sdb(_) => {},
            Self::Mcq(backend) => backend.dump_state(tag, reason),
        }
    }

    fn refresh_completions(&self) -> Result<()> {
        match self {
            Self::Sdb(backend) => backend.refresh_completions(),
            Self::Mcq(backend) => backend.refresh_completions(),
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

    fn poll_queue(&self, queue: usize) -> Result<()> {
        match self {
            Self::Sdb(backend) => backend.poll_queue(queue),
            Self::Mcq(backend) => backend.poll_queue(queue),
        }
    }

    fn completion_cached(&self, tag: usize) -> bool {
        match self {
            Self::Sdb(backend) => backend.completion_cached(tag),
            Self::Mcq(backend) => backend.completion_cached(tag),
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
                    hw_queue: None,
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
        hw_queue: usize,
    ) -> Result<()> {
        {
            let mut inner = self.inner.lock();
            if inner.state != RequestState::Idle {
                return Err(EBUSY);
            }
            inner.block_rq = Some(rq);
            inner.hw_queue = Some(hw_queue);
        }

        if let Err(e) = self.compose_scsi_cmd(cmd) {
            self.clear();
            return Err(e);
        }

        Ok(())
    }

    pub(crate) fn submit(&self) -> Result<()> {
        let (cmd, hw_queue) = self.cmd_and_hw_queue()?;

        let result = match cmd {
            UfsCmd::Device(cmd) => {
                self.queue.prepare_dev_wait();
                self.queue.submit_dev(cmd, self.tag)
            },
            UfsCmd::SCSI(cmd) => self.queue.submit_scsi(cmd, self.tag, hw_queue),
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
        let cmd = self.cmd()?;

        if self.inner.lock().state == RequestState::Idle {
            pr_err!(
                "[RUFS] ufs_queue: request tag={} is not submitted\n",
                self.tag,
            );
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
        let cmd = self.cmd()?;

        if self.inner.lock().state != RequestState::Completed {
            pr_err!(
                "[RUFS] ufs_queue: request tag={} is not completed\n",
                self.tag,
            );
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

    fn cmd(&self) -> Result<UfsCmd> {
        match self.inner.lock().cmd {
            Some(cmd) => Ok(cmd),
            None => self.missing_command(),
        }
    }

    fn cmd_and_hw_queue(&self) -> Result<(UfsCmd, Option<usize>)> {
        let inner = self.inner.lock();
        match inner.cmd {
            Some(cmd) => Ok((cmd, inner.hw_queue)),
            None => self.missing_command(),
        }
    }

    fn missing_command<T>(&self) -> Result<T> {
        pr_err!(
            "[RUFS] ufs_queue: request tag={} has no command\n",
            self.tag,
        );
        Err(EIO)
    }

    pub(crate) fn clear(&self) {
        let mut inner = self.inner.lock();
        inner.prdt = None;
        inner.block_rq = None;
        inner.hw_queue = None;
        inner.scsi_completion = None;
        inner.cmd = None;
        inner.state = RequestState::Idle;
    }

    fn timeout(&self) -> bool {
        let (cmd, block_rq, prdt, hw_queue) = {
            let mut inner = self.inner.lock();
            if inner.state != RequestState::Submitted && inner.state != RequestState::Completed {
                return true;
            }

            let Some(block_rq) = inner.block_rq.take() else {
                return true;
            };

            let cmd = inner.cmd;
            let prdt = inner.prdt.take();
            let hw_queue = inner.hw_queue.take();
            inner.scsi_completion = None;
            inner.cmd = None;
            inner.state = RequestState::Idle;
            (cmd, block_rq, prdt, hw_queue)
        };

        if let Some(UfsCmd::SCSI(cmd)) = cmd {
            let cdb = cmd.cdb();
            pr_err!(
                "[RUFS] ufs_queue: SCSI request timeout tag={} lun={} opcode=0x{:02x}\n",
                self.tag,
                cmd.lun(),
                cdb[0],
            );
        } else {
            pr_err!("[RUFS] ufs_queue: request timeout tag={}\n", self.tag);
        }
        self.queue.dump_backend_state(self.tag, "request timeout");

        // This is only a minimum timeout return path. It does not clean the MCQ
        // SQ or prevent a late CQE for the same tag; full error handling will
        // need to quiesce/recover hardware before reusing timed-out tags.
        match OwnableRefCounted::try_from_shared(block_rq) {
            Ok(block_rq) => {
                block_rq.end(bindings::BLK_STS_IOERR as u8);
                drop(prdt);
                true
            },
            Err(block_rq) => {
                let mut inner = self.inner.lock();
                inner.cmd = cmd;
                inner.prdt = prdt;
                inner.block_rq = Some(block_rq);
                inner.hw_queue = hw_queue;
                inner.state = RequestState::Completed;
                false
            },
        }
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
        let cmd = match self.cmd() {
            Ok(cmd) => cmd,
            Err(_) => return true,
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
                        pr_err!(
                            "[RUFS] ufs_queue: no block request for SCSI completion tag={}\n",
                            self.tag,
                        );
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

    fn completion_ready_cached(&self) -> bool {
        if self.inner.lock().state == RequestState::Completed {
            return true;
        }

        self.queue.completion_cached(self.tag)
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
            .config_mcq_max_active_cmds(
                u32::try_from(backend.queue_depth()).map_err(|_| EOVERFLOW)?,
            )?;
        backend.enable();
        backend.reg.enable_mcq_interrupts();

        let layout = backend.layout;
        let queue_depth = backend.queue_depth();
        let allocated_queues = backend.allocated_queues();
        *self.backend.lock() = UfsTransferBackend::Mcq(backend);
        pr_info!(
            "[RUFS] ufs_queue: MCQ backend enabled queues={}/{} interrupt={} poll={} allocated={} depth={}\n",
            layout.total_queues,
            layout.max_queues,
            layout.interrupt_queues,
            layout.poll_queues,
            allocated_queues,
            queue_depth,
        );
        Ok(())
    }

    pub(crate) fn queue_map(&self) -> Result<UfsQueueMap> {
        self.backend.lock().queue_map()
    }

    pub(crate) fn queue_depth(&self) -> usize {
        self.backend.lock().queue_depth()
    }

    fn validate_tag_depth(&self, tag: usize) -> Result<()> {
        if tag < self.queue_depth() {
            Ok(())
        } else {
            Err(EINVAL)
        }
    }

    pub(crate) fn reserve(self: &Arc<Self>) -> Result<Arc<UfsRequest>> {
        let queue_depth = self.queue_depth();
        if queue_depth == 0 {
            return Err(EINVAL);
        }

        let mut slots = self.slot.lock();
        if queue_depth > slots.len() {
            return Err(EINVAL);
        }

        for (tag, slot) in slots.iter_mut().take(queue_depth).enumerate().rev() {
            if slot.is_none() {
                let request = UfsRequest::new(self.clone(), tag)?;
                slot.replace(request.clone());
                return Ok(request);
            }
        }

        Err(ENOMEM)
    }

    pub(crate) fn acquire(
        self: &Arc<Self>,
        tag: usize,
    ) -> Result<Arc<UfsRequest>> {
        self.validate_tag_depth(tag)?;

        let mut binding = self.slot.lock();
        let slot = match binding.get_mut(tag) {
            Some(slot) => slot,
            None => {
                pr_err!("[RUFS] ufs_queue: no slot for tag={}\n", tag);
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

    fn submit_scsi(&self, cmd: UfsSCSICmd, tag: usize, hw_queue: Option<usize>) -> Result<()> {
        self.backend.lock().submit_scsi(cmd, tag, hw_queue)
    }

    fn prepare_dev_wait(&self) {
        self.completion.reinit();
    }

    fn wait_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        match self.completion.wait_for_completion_timeout(cmd.timeout()) {
            0 => {
                self.dump_backend_state(tag, "device request timeout");
                Err(ETIMEDOUT)
            },
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

    fn refresh_backend_completions(&self) -> Result<()> {
        self.backend.lock().refresh_completions()
    }

    fn dump_backend_state(&self, tag: usize, reason: &str) {
        self.backend.lock().dump_state(tag, reason);
    }

    pub(crate) fn timeout(&self, tag: usize) -> bool {
        let request = {
            let mut slots = self.slot.lock();
            slots.get_mut(tag).and_then(|slot| slot.as_ref()).cloned()
        };

        match request {
            Some(request) => request.timeout(),
            None => true,
        }
    }

    fn poll_backend_queue(&self, queue: usize) -> Result<()> {
        self.backend.lock().poll_queue(queue)
    }

    fn completion_cached(&self, tag: usize) -> bool {
        self.backend.lock().completion_cached(tag)
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

    fn next_completable_request_on_queue(
        &self,
        mut tag: usize,
        hw_queue: usize,
    ) -> Option<Arc<UfsRequest>> {
        while let Some(request) = self.next_request(tag) {
            let matches_queue = {
                let inner = request.inner.lock();
                match inner.state {
                    RequestState::Submitted | RequestState::Completed => {
                        inner.hw_queue == Some(hw_queue)
                    },
                    _ => false,
                }
            };
            if matches_queue {
                return Some(request);
            }

            tag = request.tag + 1;
        }
        None
    }

    fn complete_ready_request(&self, request: &Arc<UfsRequest>) -> (bool, bool) {
        if request.completion_ready_cached() {
            if request.complete() {
                return (true, false);
            }

            return (false, true);
        }

        (false, false)
    }

    pub(crate) fn complete(self: &Arc<Self>) -> bool {
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
        if let Err(e) = self.refresh_backend_completions() {
            pr_err!(
                "[RUFS] ufs_queue: refresh completions failed errno={}\n",
                e.to_errno(),
            );
            self.dump_backend_state(0, "refresh completions failed");
            return false;
        }

        let mut tag = 0;
        let mut completed = false;
        let mut retry = false;
        while let Some(request) = self.next_completable_request(tag) {
            let request_tag = request.tag;
            let (request_completed, request_retry) = self.complete_ready_request(&request);
            completed |= request_completed;
            retry |= request_retry;
            tag = request_tag + 1;
        }

        if retry {
            self.wake_completion_thread();
        }

        completed
    }

    pub(crate) fn poll(self: &Arc<Self>, hw_queue: usize) -> bool {
        if let Err(e) = self.poll_backend_queue(hw_queue) {
            pr_err!(
                "[RUFS] ufs_queue: poll queue {} failed errno={}\n",
                hw_queue,
                e.to_errno(),
            );
            return false;
        }

        let mut tag = 0;
        let mut completed = false;
        let mut retry = false;
        while let Some(request) = self.next_completable_request_on_queue(tag, hw_queue) {
            let request_tag = request.tag;
            let (request_completed, request_retry) = self.complete_ready_request(&request);
            completed |= request_completed;
            retry |= request_retry;
            tag = request_tag + 1;
        }

        if retry {
            self.wake_completion_thread();
        }

        completed
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
