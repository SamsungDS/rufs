// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use crate::ufs_dev::*;
use crate::ufs_dma::*;
use crate::ufs_lu::QueueData;
use crate::ufs_lu::TagSetData;
use crate::ufs_lu::UfsLuBlockOps;
use crate::ufs_reg::*;
use kernel::alloc::mempool::MemPool;
use kernel::block::mq;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::block::mq::TagSet;
use kernel::cpu;
use kernel::sync::atomic::{Atomic, Relaxed, Release};
use kernel::sync::{aref::ARef, barrier, Arc, Completion, SpinLock, SpinLockIrq};
use kernel::types::OwnableRefCounted;
use kernel::types::Owned;
use kernel::{bindings, kvec, new_spinlock, new_spinlock_irq, prelude::*};

const READ_10: u8 = 0x28;
const WRITE_10: u8 = 0x2a;
const SYNCHRONIZE_CACHE: u8 = 0x35;
const UNMAP: u8 = 0x42;
const READ_16: u8 = 0x88;
const WRITE_16: u8 = 0x8a;
const UFS_MCQ_DEFAULT_READ_QUEUES: usize = 0;
const UFS_MCQ_DEFAULT_POLL_QUEUES: usize = 1;
const MAX_COMPLETED_TAGS: usize = 256;

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
    // These fields form one ownership unit. Keep them under a single lock so a
    // slot is never visible as idle while old DMA or block request state remains.
    pub(crate) cmd: Option<UfsCmd>,
    prdt: Option<UfsPrdtMapping>,
    hw_queue: Option<u32>,
}

impl Default for UfsRequestInner {
    fn default() -> Self {
        UfsRequestInner {
            cmd: None,
            prdt: None,
            hw_queue: None,
        }
    }
}

impl UfsRequestInner {
    pub(crate) fn clear(&mut self) {
        self.prdt = None;
        self.hw_queue = None;
        self.cmd = None;
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

struct CompletedTags {
    tags: [usize; MAX_COMPLETED_TAGS],
    len: usize,
    pos: usize,
}

impl CompletedTags {
    fn new() -> Self {
        Self {
            tags: [0; MAX_COMPLETED_TAGS],
            len: 0,
            pos: 0,
        }
    }

    fn insert(&mut self, tag: usize) -> Result<()> {
        if self.len == self.tags.len() {
            return Err(ENOMEM);
        }

        self.tags[self.len] = tag;
        self.len += 1;
        Ok(())
    }

    fn insert_sdb_mask(&mut self, mut mask: u32) -> Result<()> {
        while mask != 0 {
            let tag = mask.trailing_zeros();
            mask &= !(1u32 << tag);
            self.insert(tag as usize)?;
        }

        Ok(())
    }

    fn take_next(&mut self) -> Option<usize> {
        if self.pos == self.len {
            return None;
        }

        let tag = self.tags[self.pos];
        self.pos += 1;
        Some(tag)
    }
}

#[pin_data]
struct McqQueueSet {
    #[pin]
    queues: SpinLock<Option<KVec<UfsMcqQueue>>>,

    #[pin]
    completed: SpinLock<KVec<Option<CqEntry>>>,
}

impl McqQueueSet {
    fn new(completed: KVec<Option<CqEntry>>) -> impl PinInit<Self> {
        pin_init!(Self {
            queues <- new_spinlock!(None),
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

    fn queue_index(queue_hint: Option<u32>, tag: u32, nr_queues: u32) -> Result<u32> {
        if nr_queues == 0 {
            return Err(EINVAL);
        }

        Ok(queue_hint.unwrap_or(tag) % nr_queues)
    }

    fn submit(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        tag: u32,
        queue_hint: Option<usize>,
    ) -> Result<()> {
        {
            let mut completed = self.completed.lock();
            *completed.get_mut(tag as usize).ok_or(EINVAL)? = None;
        }

        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;
        let queue_index =
            Self::queue_index(queue_hint.map(|v| v as u32), tag, queues.len() as u32)?;
        let queue = queues.get_mut(queue_index as usize).ok_or(EINVAL)?;
        let queue_id = queue.id() as usize;
        if queue.sq_is_full(reg)? {
            return Err(EBUSY);
        }

        let sqe = dma.transfer_request_desc(tag as usize)?;
        let tail = queue.write_sq_entry(sqe)?;

        barrier::smp_wmb();
        reg.write_mcq_sq_tail(queue.oprs(), queue_id, tail)
    }

    fn poll_completions(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        nr_queues: usize,
        completed_tags: &mut CompletedTags,
    ) -> Result<()> {
        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;

        for queue in queues.iter_mut().take(nr_queues) {
            self.collect_queue_completions(reg, dma, queue, completed_tags)?;
        }

        Ok(())
    }

    fn poll_completions_for_queue(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        queue_id: usize,
        completed_tags: &mut CompletedTags,
    ) -> Result<()> {
        let mut guard = self.queues.lock();
        // SAFETY: While holding the lock, this method only mutates queue state
        // in place and never moves queues out of the vector or grows it.
        let queues = unsafe { core::pin::Pin::get_unchecked_mut(guard.as_mut()) }
            .as_mut()
            .ok_or(EINVAL)?;
        let queue = queues.get_mut(queue_id).ok_or(EINVAL)?;

        self.collect_queue_completions(reg, dma, queue, completed_tags)
    }

    fn collect_queue_completions(
        &self,
        reg: &UfsReg,
        dma: &UfsDma,
        queue: &mut UfsMcqQueue,
        completed_tags: &mut CompletedTags,
    ) -> Result<()> {
        queue.update_cq_tail_slot(reg)?;
        while !queue.cq_is_empty() {
            if let Some(cqe) = queue.consume_cq_entry(reg)? {
                let tag = dma.tag_from_cq_entry(&cqe)?;
                let mut completed = self.completed.lock();
                *completed.get_mut(tag).ok_or(EINVAL)? = Some(cqe);
                completed_tags.insert(tag)?;
            }
        }
        queue.acknowledge_cq_events(reg)?;

        Ok(())
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

trait UfsTransferOps {
    fn queue_depth(&self) -> usize;
    fn queue_map(&self) -> Result<UfsQueueMap>;
    fn compose_dev(&self, cmd: UfsDevCmd, tag: u32) -> Result<()>;
    fn compose_scsi(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>>;
    fn submit(&self, tag: u32) -> Result<()>;
    fn dump_state(&self, tag: usize, reason: &str);
    fn collect_completions(&self, completed_tags: &mut CompletedTags) -> Result<()>;
    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd>;
    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult;
    fn poll_queue(&self, queue: usize, completed_tags: &mut CompletedTags) -> Result<()>;
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

    fn submit_tag(&self, tag: u32) -> Result<()> {
        let mask = Self::tag_mask(tag).ok_or(EINVAL)?;
        let mut state = self.state.completion.lock();

        state.outstanding |= mask;
        self.reg.ring_utrl_doorbell(tag);

        Ok(())
    }

    fn collect_completions(&self, completed_tags: &mut CompletedTags) -> Result<()> {
        let mut state = self.state.completion.lock();
        let doorbell = self.reg.read_utrl_doorbell();
        let completed = !doorbell & state.outstanding;

        state.outstanding &= !completed;
        completed_tags.insert_sdb_mask(completed)
    }
}

impl UfsTransferOps for SdbTransferBackend {
    fn queue_depth(&self) -> usize {
        self.reg.nutrs()
    }

    fn queue_map(&self) -> Result<UfsQueueMap> {
        // TODO: Why is this McqQueueLayout? This operation is not related to MCQ.
        McqQueueLayout::sdb().queue_map()
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

    fn submit(&self, tag: u32) -> Result<()> {
        self.submit_tag(tag)
    }

    fn collect_completions(&self, completed_tags: &mut CompletedTags) -> Result<()> {
        SdbTransferBackend::collect_completions(self, completed_tags)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        self.dma.fetch_devman_upiu(cmd, tag)
    }

    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        self.dma.fetch_scsi_completion(tag)
    }

    fn poll_queue(&self, _queue: usize, completed_tags: &mut CompletedTags) -> Result<()> {
        SdbTransferBackend::collect_completions(self, completed_tags)
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
    fn new(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        if !reg.mcq_supported() {
            return Err(ENOTSUPP);
        }

        let layout = ufs_mcq_queue_layout(&reg)?;
        // TODO: Should not do min here for MCQ?
        let queue_depth = core::cmp::min(reg.nutrs_mcq(), dma.transfer_slots());
        if queue_depth > MAX_COMPLETED_TAGS {
            return Err(EOVERFLOW);
        }
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

    fn queue_depth(&self) -> usize {
        self.queue_depth
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

    fn submit(&self, tag: u32) -> Result<()> {
        // TODO: How to set correct hw queue?
        self.queues.submit(&self.reg, &self.dma, tag, Some(0))
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        self.queues.dump_state(&self.reg, tag, reason);
    }

    // MCQ CQE consumption is destructive because the software CQ head advances.
    // Snapshot each CQE before returning its tag so request finalization can
    // decode the consumed CQE after the backend lock is released.
    fn collect_completions(&self, completed_tags: &mut CompletedTags) -> Result<()> {
        self.queues.poll_completions(
            &self.reg,
            &self.dma,
            self.layout.interrupt_queues,
            completed_tags,
        )
    }

    fn poll_queue(&self, queue: usize, completed_tags: &mut CompletedTags) -> Result<()> {
        if !self.layout.is_poll_queue(queue) {
            return Err(EINVAL);
        }

        self.queues
            .poll_completions_for_queue(&self.reg, &self.dma, queue, completed_tags)
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

impl UfsTransferOps for McqTransferBackend {
    fn queue_depth(&self) -> usize {
        McqTransferBackend::queue_depth(self)
    }

    fn queue_map(&self) -> Result<UfsQueueMap> {
        self.layout.queue_map()
    }

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

    fn submit(&self, tag: u32) -> Result<()> {
        McqTransferBackend::submit(self, tag)
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        McqTransferBackend::dump_state(self, tag, reason);
    }

    fn collect_completions(&self, completed_tags: &mut CompletedTags) -> Result<()> {
        McqTransferBackend::collect_completions(self, completed_tags)
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        McqTransferBackend::fetch_dev(self, cmd, tag)
    }

    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        McqTransferBackend::fetch_scsi_completion(self, tag)
    }

    fn poll_queue(&self, queue: usize, completed_tags: &mut CompletedTags) -> Result<()> {
        McqTransferBackend::poll_queue(self, queue, completed_tags)
    }
}

impl UfsTransferBackend {
    fn sdb(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Self> {
        Ok(Self::Sdb(SdbTransferBackend::new(reg, dma)?))
    }

    fn ops(&self) -> &dyn UfsTransferOps {
        match self {
            Self::Sdb(backend) => backend,
            Self::Mcq(backend) => backend,
        }
    }
}

#[pin_data]
pub(crate) struct UfsRequest {
    // TODO: We should be able to remove these two fields
    queue: Arc<UfsQueue>,
    tag: usize,

    #[pin]
    pub(crate) inner: SpinLock<UfsRequestInner>,
}

impl UfsRequest {
    pub(crate) fn compose_dev_request(rq: &ARef<mq::Request<UfsLuBlockOps>>) -> Result<()> {
        if let QueueData::Dev(queue) = rq.queue_data() {
            let Some(UfsCmd::Device(cmd)) = rq.data_ref().inner.lock().cmd else {
                return Err(EIO);
            };
            if let Err(e) = queue.compose_dev(cmd, rq.tag()) {
                rq.data_ref().inner.lock().clear();
                Err(e)
            } else {
                Ok(())
            }
        } else {
            Err(EIO)
        }
    }

    pub(crate) fn compose_scsi_cmd(
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
    ) -> Result<()> {
        {
            let mut inner = rq.data_ref().inner.lock();
            inner.hw_queue = Some(rq.queue_index());
        }

        let mempool = rq.queue().tag_set().data().dma_vec_mempool.clone();
        let prdt = UfsQueue::compose_scsi(rq, cmd, &mempool)?;

        let mut inner = rq.data_ref().inner.lock();
        inner.prdt = prdt;
        inner.cmd = Some(UfsCmd::SCSI(cmd));
        Ok(())
    }

    pub(crate) fn submit(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
    ) -> core::result::Result<(), (ARef<mq::Request<UfsLuBlockOps>>, Error)> {
        let queue = match rq.queue_data() {
            QueueData::Dev(ufs_queue) => ufs_queue.clone(),
            QueueData::Lu(ufs_lu) => ufs_lu.queue.clone(),
        };
        let queue_id = rq.queue_index();
        let tag = rq.tag();

        // Do not keep a submit-side request reference while making the command
        // visible to hardware. A fast completion may run before `queue_rq()`
        // returns and must be able to recover unique request ownership after
        // dropping the DMA mapping's request reference.
        drop(rq);

        match queue.submit(tag) {
            Err(e) => {
                // A failed submission did not make the request visible to
                // hardware, so it is still owned by the driver and can be
                // recovered from its hctx and tag.
                let rq = queue
                    .tags
                    .tag_to_rq(queue_id, tag)
                    .expect("rufs: submitted request disappeared");
                rq.data_ref().inner.lock().clear();
                Err((rq, e))
            }
            Ok(()) => Ok(()),
        }
    }

    pub(crate) fn clear(rq: &ARef<mq::Request<UfsLuBlockOps>>) {
        let mut inner = rq.data_ref().inner.lock();
        inner.prdt = None;
        inner.hw_queue = None;
        inner.cmd = None;
    }

    pub(crate) fn timeout(rq: ARef<mq::Request<UfsLuBlockOps>>) -> bool {
        let queue = match rq.queue_data() {
            QueueData::Dev(queue) => queue.clone(),
            QueueData::Lu(lu) => lu.queue.clone(),
        };
        queue.dump_backend_state(rq.tag() as usize, "request timeout");

        let (cmd, prdt, hw_queue) = {
            let mut inner = rq.data_ref().inner.lock();
            let cmd = inner.cmd;
            let prdt = inner.prdt.take();
            let hw_queue = inner.hw_queue.take();
            inner.cmd = None;
            (cmd, prdt, hw_queue)
        };

        if let Some(UfsCmd::SCSI(cmd)) = cmd {
            let cdb = cmd.cdb();
            pr_err!(
                "[RUFS] ufs_queue: SCSI request timeout tag={} lun={} opcode=0x{:02x}\n",
                rq.tag(),
                cmd.lun(),
                cdb[0],
            );
        } else {
            pr_err!("[RUFS] ufs_queue: request timeout tag={}\n", rq.tag());
        }
        //rq.queue_data().queue.dump_backend_state(self.tag, "request timeout");

        // This is only a minimum timeout return path. It does not clean the MCQ
        // SQ or prevent a late CQE for the same tag; full error handling will
        // need to quiesce/recover hardware before reusing timed-out tags.
        match OwnableRefCounted::try_from_shared(rq) {
            Ok(rq) => {
                rq.end(bindings::BLK_STS_IOERR as u8);
                drop(prdt);
                true
            }
            Err(rq) => {
                let mut inner = rq.data_ref().inner.lock();
                inner.cmd = cmd;
                inner.prdt = prdt;
                inner.hw_queue = hw_queue;
                false
            }
        }
    }

    fn complete(rq: ARef<mq::Request<UfsLuBlockOps>>) -> bool {
        Self::complete_with(rq, CompletionTarget::Direct)
    }

    fn complete_polled(
        rq: ARef<mq::Request<UfsLuBlockOps>>,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> bool {
        UfsRequest::complete_with(rq, CompletionTarget::Poll(batch))
    }

    fn complete_with(rq: ARef<mq::Request<UfsLuBlockOps>>, target: CompletionTarget<'_>) -> bool {
        let cmd = rq
            .data_ref()
            .inner
            .lock()
            .cmd
            .expect("Command must have cmd");

        match cmd {
            UfsCmd::Device(cmd) => {
                let QueueData::Dev(queue) = rq.queue_data() else {
                    panic!("Invalid context")
                };
                let cmd = queue
                    .fetch_dev(cmd, rq.tag() as usize)
                    .expect("Expected dev cmd");
                rq.data_ref().inner.lock().cmd = Some(cmd);
                let rq = Owned::try_from(rq)
                    .expect("Expected exclusive access")
                    .end_ok();
                true
            }
            UfsCmd::SCSI(cmd) => {
                let QueueData::Lu(lu) = rq.queue_data() else {
                    panic!("Invalid context")
                };
                let queue = &lu.queue;
                let result = queue.fetch_scsi_completion(rq.tag() as usize);
                drop(rq.data_ref().inner.lock().prdt.take());

                queue.clone().complete_scsi(cmd, result, rq, target);
                // TODO: missing a clear() call here
                true
            }
        }
    }
}

#[pin_data]
pub(crate) struct UfsQueue {
    reg: Arc<UfsReg>,
    pub(crate) tags: Arc<TagSet<UfsLuBlockOps>>,

    #[pin]
    backend: SpinLock<UfsTransferBackend>,

    cached_queue_depth: Atomic<usize>,

    #[pin]
    completion: Completion,
}

impl UfsQueue {
    pub(crate) fn new(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Arc<Self>> {
        if reg.mcq_supported() {
            pr_info!(
                "[RUFS] ufs_queue: MCQ supported by controller mcq_depth={}\n",
                reg.nutrs_mcq(),
            );
        }

        // The request table is sized for the allocation, while each backend
        // reports the tag range that is legal for that transport.
        let backend = UfsTransferBackend::sdb(reg.clone(), dma)?;
        let max_concurrent_requests = backend.ops().queue_depth();
        let queue_map = backend.ops().queue_map()?;
        let nr_hw_queues = queue_map.nr_hw_queues();
        // TODO: Do we need this sub by one when we do not reserve a request for UfsDev?
        let blk_mq_tag_count = max_concurrent_requests.checked_sub(1).ok_or(EINVAL)?;
        if blk_mq_tag_count == 0 || nr_hw_queues == 0 {
            return Err(EINVAL);
        }

        let tagset_data = KBox::new(
            TagSetData {
                queue_map,
                // TODO: wrong depth
                dma_vec_mempool: MemPool::new(1)?,
            },
            GFP_KERNEL,
        )?;

        let tagset = Arc::pin_init(
            TagSet::<UfsLuBlockOps>::new(
                nr_hw_queues as u32,
                tagset_data,
                u32::try_from(blk_mq_tag_count).map_err(|_| EOVERFLOW)?,
                queue_map.num_maps(),
                kernel::alloc::NumaNode::NO_NODE,
                kernel::block::mq::tag_set::Flags::default(),
            ),
            GFP_KERNEL,
        )?;

        let queue = Arc::pin_init(
            try_pin_init!(Self {
                reg,
                tags <- tagset,
                backend <- new_spinlock!(backend),
                cached_queue_depth: Atomic::new(max_concurrent_requests),
                completion <- Completion::new(),
            }),
            GFP_KERNEL,
        )?;

        Ok(queue)
    }

    pub(crate) fn enable_mcq_backend(&self, reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<()> {
        let backend = McqTransferBackend::new(reg, dma)?;
        backend.prepare()?;
        backend.reg.config_mcq_max_active_cmds(
            u32::try_from(backend.queue_depth()).map_err(|_| EOVERFLOW)?,
        )?;
        backend.enable();
        backend.reg.enable_mcq_interrupts();

        let layout = backend.layout;
        let queue_depth = backend.queue_depth();
        let allocated_queues = backend.allocated_queues();
        *self.backend.lock() = UfsTransferBackend::Mcq(backend);
        self.cached_queue_depth.store(queue_depth, Relaxed);
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
        self.backend.lock().ops().queue_map()
    }

    pub(crate) fn queue_depth(&self) -> usize {
        self.cached_queue_depth.load(Relaxed)
    }

    // Issuing
    pub(crate) fn compose_dev(&self, cmd: UfsDevCmd, tag: u32) -> Result<()> {
        self.backend.lock().ops().compose_dev(cmd, tag)
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

        queue.backend.lock().ops().compose_scsi(cmd, rq, mempool)
    }

    fn submit(&self, tag: u32) -> Result<()> {
        self.backend.lock().ops().submit(tag)
    }

    fn prepare_dev_wait(&self) {
        self.completion.reinit();
    }

    fn fetch_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        self.backend.lock().ops().fetch_dev(cmd, tag)
    }

    fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        self.backend.lock().ops().fetch_scsi_completion(tag)
    }

    fn collect_backend_completions(&self, completed_tags: &mut CompletedTags) -> Result<()> {
        self.backend.lock().ops().collect_completions(completed_tags)
    }

    fn dump_backend_state(&self, tag: usize, reason: &str) {
        self.backend.lock().ops().dump_state(tag, reason);
    }

    pub(crate) fn timeout(&self, tag: usize) -> bool {
        let rq = self.tags.tag_to_rq(0, tag as u32).expect("Expected to find tag");
        UfsRequest::timeout(rq)
    }

    fn poll_backend_queue(&self, queue: usize, completed_tags: &mut CompletedTags) -> Result<()> {
        self.backend.lock().ops().poll_queue(queue, completed_tags)
    }

    fn request_at_tag(&self, tag: u32) -> ARef<mq::Request<UfsLuBlockOps>> {
        self.tags.tag_to_rq(0, tag).expect("Expected to find tag")
    }

    fn complete_tag(&self, tag: u32) -> bool {
        UfsRequest::complete(self.request_at_tag(tag))
    }

    fn complete_polled_tag(
        &self,
        tag: u32,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> bool {
        UfsRequest::complete_polled(self.request_at_tag(tag), batch)
    }

    pub(crate) fn complete(self: &Arc<Self>) -> bool {
        // Completion is tag-driven: the backend collects completed tags, then
        // the queue finalizes exactly those requests. Finalization still runs
        // from the threaded IRQ path because it takes request, backend, and DMA
        // locks that are shared with submission and hands requests back to
        // blk-mq. Once those lock domains are IRQ-safe or removed by tag-based
        // request lookup, this tag-driven path can move into hard IRQ context.
        let mut completed_tags = CompletedTags::new();
        if let Err(e) = self.collect_backend_completions(&mut completed_tags) {
            pr_err!(
                "[RUFS] ufs_queue: collect completions failed errno={}\n",
                e.to_errno(),
            );
            self.dump_backend_state(0, "collect completions failed");
            return false;
        }

        let mut completed = false;
        while let Some(tag) = completed_tags.take_next() {
            completed |= self.complete_tag(tag as u32);
        }

        completed
    }

    pub(crate) fn poll(
        self: &Arc<Self>,
        hw_queue: usize,
        batch: &mut mq::IoCompletionBatch<UfsLuBlockOps>,
    ) -> bool {
        let mut completed_tags = CompletedTags::new();
        if let Err(e) = self.poll_backend_queue(hw_queue, &mut completed_tags) {
            pr_err!(
                "[RUFS] ufs_queue: poll queue {} failed errno={}\n",
                hw_queue,
                e.to_errno(),
            );
            return false;
        }

        let mut completed = false;
        while let Some(tag) = completed_tags.take_next() {
            completed |= self.complete_polled_tag(tag as u32, batch);
        }

        completed
    }

    fn complete_dev(&self, cmd: UfsDevCmd, tag: usize) {
        self.completion.complete();
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
        if requeue {
            match OwnableRefCounted::try_from_shared(rq) {
                Ok(rq) => rq.requeue(true),
                Err(rq) => {
                    rq.data_ref().status.store(status, Release);
                    mq::Request::complete(rq);
                }
            }
            return;
        }

        match target {
            CompletionTarget::Direct => {
                rq.data_ref().status.store(status, Release);
                mq::Request::complete(rq);
            }
            CompletionTarget::Poll(batch) => {
                let rq = match OwnableRefCounted::try_from_shared(rq) {
                    Ok(rq) => rq,
                    Err(rq) => {
                        rq.data_ref().status.store(status, Release);
                        mq::Request::complete(rq);
                        return;
                    }
                };

                if status != u32::from(bindings::BLK_STS_OK) {
                    rq.end(status as u8);
                    return;
                }

                if let Err(rq) = batch.add_request(rq, false) {
                    rq.end(status as u8);
                }
            }
        }
    }
}
