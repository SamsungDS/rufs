// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use crate::lu::UfsLuBlockOps;
use crate::protocol::query::*;
use crate::protocol::scsi::*;
use crate::protocol::upiu::{Upiu, UpiuTmReq, UpiuTmRsp};
use crate::protocol::UfsCmd;
use crate::reg::*;
use kernel::bits::{genmask_u64, genmask_u8};
use kernel::block::mq::dma_map_iter::DmaMapIterMapped;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::dma;
use kernel::dma::Coherent;
use kernel::io::io_project;
use kernel::io::Io;
use kernel::sync::{aref::ARef, Arc};
use kernel::{
    block::mq,
    device::{self, Bound, Core},
    pci,
    prelude::*,
};

pub(crate) const PRDT_DATA_BYTE_COUNT_MAX: u32 = 0x00040000; // SZ_256K
const PRDT_DATA_BYTE_COUNT_PAD: usize = 4;
const UNMAP_PARAM_LIST_SIZE: usize = 24;

pub(crate) const MAX_PRD_ENTRIES: usize = 256;

const MASK_OCS: u8 = 0x0f;
const CQE_UCD_BASE_ADDR: u64 = genmask_u64(7..=63);
const CQE_SQ_ID: u64 = genmask_u64(0..=4);
const ALIGNED_UPIU_SIZE: usize = 512;

enum UtpCmdType {
    UfsStorage = 0x1,
}

enum UtpDataDirection {
    NoDataTransfer = 0,
    HostToDevice = 1,
    DeviceToHost = 2,
}

impl From<dma::DataDirection> for UtpDataDirection {
    fn from(direction: dma::DataDirection) -> Self {
        match direction {
            dma::DataDirection::ToDevice => Self::HostToDevice,
            dma::DataDirection::FromDevice => Self::DeviceToHost,
            _ => Self::NoDataTransfer,
        }
    }
}

impl From<UfsScsiDataDirection> for UtpDataDirection {
    fn from(direction: UfsScsiDataDirection) -> Self {
        match direction {
            UfsScsiDataDirection::Read => Self::DeviceToHost,
            UfsScsiDataDirection::Write => Self::HostToDevice,
            UfsScsiDataDirection::None => Self::NoDataTransfer,
        }
    }
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy, IntoBytes)]
pub(crate) struct PrdEntry {
    pub(crate) addr: u64,
    pub(crate) reserved: u32,
    pub(crate) size: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) struct Ucd {
    pub(crate) cmd_upiu: Upiu,
    pub(crate) rsp_upiu: Upiu,
    pub(crate) prdt: [PrdEntry; MAX_PRD_ENTRIES],
}

#[derive(Clone, Copy)]
pub(crate) enum UtpOcs {
    Success = 0x0,
    InvalidCmdTableAttr = 0x1,
    InvalidPrdtAttr = 0x2,
    MismatchDataBufSize = 0x3,
    MisMatchRespUpiuSize = 0x4,
    PeerCommFailure = 0x5,
    Aborted = 0x6,
    FatalError = 0x7,
    DeviceFatalError = 0x8,
    InvalidCryptoConfig = 0x9,
    GeneralCryptoError = 0xa,
    InvalidCommandStatus = 0xf,
}

impl From<u8> for UtpOcs {
    fn from(ocs: u8) -> Self {
        match ocs {
            0x0 => Self::Success,
            0x1 => Self::InvalidCmdTableAttr,
            0x2 => Self::InvalidPrdtAttr,
            0x3 => Self::MismatchDataBufSize,
            0x4 => Self::MisMatchRespUpiuSize,
            0x5 => Self::PeerCommFailure,
            0x6 => Self::Aborted,
            0x7 => Self::FatalError,
            0x8 => Self::DeviceFatalError,
            0x9 => Self::InvalidCryptoConfig,
            0xa => Self::GeneralCryptoError,
            _ => Self::InvalidCommandStatus,
        }
    }
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy, FromBytes, IntoBytes)]
struct ReqDescHeader {
    cci: u8,
    ehs_length: u8,
    flags: u8,
    ctrl: u8,
    dunl: u32,
    ocs: u8,
    cds: u8,
    ldbc: u16,
    dunu: u32,
}

impl ReqDescHeader {
    fn set_cmd_type(&mut self, cmd_type: UtpCmdType) {
        self.ctrl &= !genmask_u8(4..=7);
        self.ctrl |= ((cmd_type as u8) << 4) & genmask_u8(4..=7);
    }

    fn set_direction(&mut self, direction: UtpDataDirection) {
        self.ctrl &= !genmask_u8(1..=2);
        self.ctrl |= ((direction as u8) << 1) & genmask_u8(1..=2);
    }

    fn set_interrupt(&mut self, interrupt: bool) {
        self.ctrl &= !genmask_u8(0..=0);
        self.ctrl |= (interrupt as u8) & genmask_u8(0..=0);
    }

    fn device() -> Self {
        let mut header = Self::default();
        header.set_cmd_type(UtpCmdType::UfsStorage);
        header.set_direction(UtpDataDirection::NoDataTransfer);
        header.set_interrupt(true);
        header.ocs = UtpOcs::InvalidCommandStatus as u8;
        header
    }

    fn scsi(cmd: UfsSCSICmd) -> Self {
        let mut header = Self::default();
        header.set_cmd_type(UtpCmdType::UfsStorage);
        header.set_direction(cmd.direction().into());
        header.set_interrupt(true);
        header.ocs = UtpOcs::InvalidCommandStatus as u8;
        header
    }
}

#[repr(C, packed)]
#[derive(Default, FromBytes, IntoBytes)]
pub(crate) struct Utrd {
    header: ReqDescHeader,
    command_desc_base_addr: u64,
    rsp_upiu_length: u16,
    rsp_upiu_offset: u16,
    prd_table_length: u16,
    prd_table_offset: u16,
}

pub(crate) type SqEntry = Utrd;

#[repr(C, packed)]
#[derive(Default, Clone, Copy, FromBytes, IntoBytes)]
pub(crate) struct CqEntry {
    command_desc_base_addr: u64,
    rsp_upiu_length: u16,
    rsp_upiu_offset: u16,
    prd_table_length: u16,
    prd_table_offset: u16,
    overall_status: u8,
    extended_error_code: u8,
    reserved_1: u16,
    task_tag: u8,
    lun: u8,
    iid_ext_iid: u8,
    reserved_2: u8,
    reserved_3: [u32; 2],
}

impl CqEntry {
    pub(crate) fn command_desc_base_addr(&self) -> u64 {
        u64::from_le(self.command_desc_base_addr)
    }

    pub(crate) fn ucd_base_addr(&self) -> u64 {
        self.command_desc_base_addr() & CQE_UCD_BASE_ADDR
    }

    pub(crate) fn matches_ucd_base_addr(&self, addr: u64) -> bool {
        self.ucd_base_addr() == addr & CQE_UCD_BASE_ADDR
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.command_desc_base_addr() == 0
    }

    pub(crate) fn task_tag(&self) -> u8 {
        self.task_tag
    }

    pub(crate) fn submission_queue_id(&self) -> u8 {
        (self.command_desc_base_addr() & CQE_SQ_ID) as u8
    }

    pub(crate) fn overall_status(&self) -> u8 {
        self.overall_status
    }
}

impl Utrd {
    pub(crate) fn set_command_descriptor(mut self, command_desc_base_addr: u64) -> Self {
        self.command_desc_base_addr = command_desc_base_addr.to_le();
        self.rsp_upiu_length = ((ALIGNED_UPIU_SIZE >> 2) as u16).to_le();
        self.rsp_upiu_offset = ((ALIGNED_UPIU_SIZE >> 2) as u16).to_le();
        self.prd_table_offset = ((ALIGNED_UPIU_SIZE >> 1) as u16).to_le();
        self
    }

    pub(crate) fn set_prd_table_length(mut self, entries: usize) -> Result<Self> {
        self.prd_table_length = u16::try_from(entries).map_err(|_| EINVAL)?.to_le();
        Ok(self)
    }

    pub(crate) fn build(&self, cmd: UfsCmd) -> Self {
        let header = match cmd {
            UfsCmd::Device(_) => ReqDescHeader::device(),
            UfsCmd::SCSI(cmd) => ReqDescHeader::scsi(cmd),
        };
        Self { header, ..*self }
    }

    pub(crate) fn check_response(&self) -> Result<()> {
        match self.ocs().into() {
            UtpOcs::Success => Ok(()),
            UtpOcs::InvalidCmdTableAttr
            | UtpOcs::InvalidPrdtAttr
            | UtpOcs::MismatchDataBufSize
            | UtpOcs::MisMatchRespUpiuSize
            | UtpOcs::InvalidCryptoConfig
            | UtpOcs::GeneralCryptoError => Err(EINVAL),
            _ => Err(EIO),
        }
    }

    pub(crate) fn ocs(&self) -> u8 {
        self.header.ocs & MASK_OCS
    }
}

#[repr(C, packed)]
pub(crate) struct Utmrd {
    header: ReqDescHeader,
    upiu_req: UpiuTmReq,
    upiu_rsp: UpiuTmRsp,
}

const _: () = assert!(size_of::<ReqDescHeader>() == 16);
const _: () = assert!(size_of::<PrdEntry>() == 16);
const _: () = assert!(size_of::<CqEntry>() == 32);
const _: () = assert!(size_of::<Ucd>() == 5120);
const _: () = assert!(size_of::<Utrd>() == 32);
const _: () = assert!(size_of::<Utmrd>() == 80);

unsafe impl kernel::transmute::AsBytes for Ucd {}
unsafe impl kernel::transmute::FromBytes for Ucd {}
unsafe impl kernel::transmute::AsBytes for Utrd {}
unsafe impl kernel::transmute::FromBytes for Utrd {}
unsafe impl kernel::transmute::AsBytes for Utmrd {}
unsafe impl kernel::transmute::FromBytes for Utmrd {}
unsafe impl kernel::transmute::AsBytes for CqEntry {}
unsafe impl kernel::transmute::FromBytes for CqEntry {}

pub(crate) struct UfsMcqQueue {
    descriptor: UfsMcqQueueDescriptor,
    submission: UfsMcqSubmissionQueue,
    completion: UfsMcqCompletionQueue,
}

#[derive(Clone, Copy)]
pub(crate) struct UfsMcqQueueDescriptor {
    id: u32,
    max_entries: u32,
    oprs: UfsMcqOprSet,
}

pub(crate) struct UfsMcqSubmissionQueue {
    sqe: dma::Coherent<[SqEntry]>,
    sq_tail_slot: u32,
}

pub(crate) struct UfsMcqCompletionQueue {
    cqe: dma::Coherent<[CqEntry]>,
    cq_tail_slot: u32,
    cq_head_slot: u32,
}

impl UfsMcqQueue {
    pub(crate) fn new(
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

    pub(crate) fn into_parts(
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
    pub(crate) fn id(&self) -> u32 {
        self.id
    }

    pub(crate) fn max_entries(&self) -> u32 {
        self.max_entries
    }

    pub(crate) fn oprs(&self) -> &UfsMcqOprSet {
        &self.oprs
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
    pub(crate) fn dma_addr(&self) -> dma::DmaAddress {
        self.sqe.dma_handle()
    }

    pub(crate) fn sq_tail_slot(&self) -> u32 {
        self.sq_tail_slot
    }

    pub(crate) fn set_sq_tail_slot(&mut self, slot: u32) {
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

    pub(crate) fn is_full(&self, reg: &UfsReg, descriptor: &UfsMcqQueueDescriptor) -> Result<bool> {
        let head = descriptor.offset_to_slot(
            reg.read_mcq_sq_head(descriptor.oprs(), descriptor.id() as usize)?,
            core::mem::size_of::<SqEntry>() as u32,
        )?;
        Ok(self.next_sq_tail_slot(descriptor) == head)
    }

    pub(crate) fn reset(&mut self) {
        self.sq_tail_slot = 0;
    }

    pub(crate) fn write_entry(
        &mut self,
        descriptor: &UfsMcqQueueDescriptor,
        entry: SqEntry,
    ) -> Result<u32> {
        let index = self.sq_tail_index(descriptor)?;
        io_project!(self.sqe, [try: index]).copy_write(entry);

        self.sq_tail_slot = self.next_sq_tail_slot(descriptor);

        Ok(Self::sq_slot_offset(self.sq_tail_slot))
    }
}

impl UfsMcqCompletionQueue {
    pub(crate) fn dma_addr(&self) -> dma::DmaAddress {
        self.cqe.dma_handle()
    }

    pub(crate) fn tail_slot(&self) -> u32 {
        self.cq_tail_slot
    }

    pub(crate) fn head_slot(&self) -> u32 {
        self.cq_head_slot
    }

    pub(crate) fn update_tail(
        &mut self,
        reg: &UfsReg,
        descriptor: &UfsMcqQueueDescriptor,
    ) -> Result<()> {
        self.cq_tail_slot = descriptor.offset_to_slot(
            reg.read_mcq_cq_tail(descriptor.oprs(), descriptor.id() as usize)?,
            core::mem::size_of::<CqEntry>() as u32,
        )?;
        Ok(())
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.cq_head_slot == self.cq_tail_slot
    }

    pub(crate) fn acknowledge_events(
        &self,
        reg: &UfsReg,
        descriptor: &UfsMcqQueueDescriptor,
    ) -> Result<()> {
        let status = reg.read_mcq_cqis(descriptor.oprs(), descriptor.id() as usize)?;
        if status != 0 {
            reg.write_mcq_cqis(descriptor.oprs(), descriptor.id() as usize, status)?;
        }

        Ok(())
    }

    pub(crate) fn reset(&mut self) {
        self.cq_tail_slot = 0;
        self.cq_head_slot = 0;
    }

    pub(crate) fn consume_entry(
        &mut self,
        descriptor: &UfsMcqQueueDescriptor,
    ) -> Result<Option<CqEntry>> {
        let index = self.cq_head_slot as usize;
        if index >= descriptor.max_entries as usize {
            return Err(EINVAL);
        }

        let cqe = io_project!(self.cqe, [try: index]).copy_read();
        io_project!(self.cqe, [try: index]).copy_write(CqEntry::default());

        self.cq_head_slot += 1;
        if self.cq_head_slot == descriptor.max_entries {
            self.cq_head_slot = 0;
        }

        if cqe.is_empty() {
            Ok(None)
        } else {
            Ok(Some(cqe))
        }
    }

    pub(crate) fn commit_head(
        &self,
        reg: &UfsReg,
        descriptor: &UfsMcqQueueDescriptor,
    ) -> Result<()> {
        reg.write_mcq_cq_head(
            descriptor.oprs(),
            descriptor.id() as usize,
            self.cq_head_slot * core::mem::size_of::<CqEntry>() as u32,
        )
    }
}

pub(crate) struct UfsDma {
    reg: Arc<UfsReg>,
    dev: ARef<device::Device>,
    transfer_slots: usize,
    ucdl: dma::Coherent<[Ucd]>,
    utrdl: dma::Coherent<[Utrd]>,
    utmrdl: dma::Coherent<[Utmrd]>,
}

pub(crate) enum UfsPrdtMapping {
    Sg(DmaMapIterMapped<MAX_PRD_ENTRIES, UfsLuBlockOps>),
    Unmap(UfsUnmapMapping),
}

pub(crate) struct UfsUnmapMapping {
    dev: ARef<device::Device>,
    buffer: Coherent<[u8]>,
}

struct UfsPrdt {
    mapping: Option<UfsPrdtMapping>,
    entries: KVec<PrdEntry>,
}

fn append_prd_entries(
    entries: &mut KVec<PrdEntry>,
    segment_address: u64,
    segment_length: u32,
) -> Result<()> {
    if segment_length == 0 || segment_length % PRDT_DATA_BYTE_COUNT_PAD as u32 != 0 {
        return Err(EINVAL);
    }

    let mut segment_offset = 0;
    while segment_offset < segment_length {
        if entries.len() == MAX_PRD_ENTRIES {
            return Err(EINVAL);
        }

        let prd_len = core::cmp::min(PRDT_DATA_BYTE_COUNT_MAX, segment_length - segment_offset);
        let addr = segment_address
            .checked_add(u64::from(segment_offset))
            .ok_or(EOVERFLOW)?;

        entries.push(
            PrdEntry {
                addr: addr.to_le(),
                reserved: 0,
                size: (prd_len - 1).to_le(),
            },
            GFP_ATOMIC,
        )?;
        segment_offset += prd_len;
    }

    Ok(())
}

impl UfsDma {
    pub(crate) fn dev(&self) -> &device::Device<Bound> {
        // SAFETY: `UfsDma` is owned by the bound RUFS driver instance. MCQ queue
        // allocations only use this reference while the driver owns the device.
        unsafe { self.dev.as_bound() }
    }

    pub(crate) fn new(
        pdev: &pci::Device<Core<'_>>,
        reg: Arc<UfsReg>,
        transfer_slots: usize,
    ) -> Result<Arc<Self>> {
        if transfer_slots == 0 {
            return Err(EINVAL);
        }
        let ucdl = dma::Coherent::<Ucd>::zeroed_slice(pdev.as_ref(), transfer_slots, GFP_KERNEL)?;

        let utrdl = dma::Coherent::<Utrd>::zeroed_slice(pdev.as_ref(), transfer_slots, GFP_KERNEL)?;

        for tag in 0..transfer_slots {
            // The controller DMA-reads the UTP command descriptor for this tag,
            // so this must be the descriptor's DMA (bus) address, not its CPU
            // virtual address. `ucdl` is a contiguous slice, so element `tag`
            // sits at `tag * size_of::<Ucd>()` bytes from the DMA base.
            let command_desc_base_addr = io_project!(ucdl, [try: tag]).dma_handle();

            let utrd = io_project!(utrdl, [try: tag])
                .copy_read()
                .set_command_descriptor(command_desc_base_addr as u64);
            io_project!(
                utrdl,
                [try: tag]
            )
            .copy_write(utrd);
        }

        let nutmrs = reg.nutmrs();
        let utmrdl = dma::Coherent::<Utmrd>::zeroed_slice(pdev.as_ref(), nutmrs, GFP_KERNEL)?;

        Ok(Arc::new(
            Self {
                reg,
                dev: pdev.as_ref().into(),
                transfer_slots,
                ucdl,
                utrdl,
                utmrdl,
            },
            GFP_KERNEL,
        )?)
    }

    pub(crate) fn transfer_slots(&self) -> usize {
        self.transfer_slots
    }

    pub(crate) fn make_hba_operational(&self) -> Result<()> {
        self.reg.enable_interrupts();
        self.reg.disable_transfer_req_int_aggr();

        self.reg.set_utrdl_base(self.utrdl.dma_handle() as u64);
        self.reg.set_utmrdl_base(self.utmrdl.dma_handle() as u64);

        self.reg.wait_for_request_ready(1000, 50)?;
        self.reg.enable_run_stop();

        Ok(())
    }

    pub(crate) fn compose_devman_upiu(&self, cmd: UfsDevCmd, tag: u32) -> Result<()> {
        let tag: usize = tag as _;
        io_project!(self.ucdl, [try: tag].cmd_upiu).copy_write(Upiu::device(cmd, tag));
        io_project!(self.ucdl, [try: tag].rsp_upiu).copy_write(Upiu::default());

        let utrd = io_project!(self.utrdl, [try: tag]).copy_read();
        io_project!(self.utrdl, [try: tag]).copy_write(utrd.build(UfsCmd::Device(cmd)));
        Ok(())
    }

    pub(crate) fn compose_scsi_upiu(
        &self,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        cmd: UfsSCSICmd,
        task_tag: u8,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Option<UfsPrdtMapping>> {
        let prdt = self.map_request_prdt(cmd, rq, mempool)?;
        let tag = usize::from(task_tag);

        io_project!(self.ucdl, [try: tag].cmd_upiu).copy_write(Upiu::command(cmd, tag));
        io_project!(self.ucdl, [try: tag].rsp_upiu).copy_write(Upiu::default());

        for (i, entry) in prdt.entries.iter().enumerate() {
            io_project!(self.ucdl, [try: tag].prdt[try: i]).copy_write(*entry);
        }

        let utrd = io_project!(self.utrdl, [try: tag]).copy_read();
        let utrd = utrd
            .build(UfsCmd::SCSI(cmd))
            .set_prd_table_length(prdt.entries.len())?;
        io_project!(self.utrdl, [try: tag]).copy_write(utrd);

        Ok(prdt.mapping)
    }

    pub(crate) fn transfer_request_desc(&self, tag: usize) -> Result<Utrd> {
        Ok(io_project!(self.utrdl, [try: tag]).copy_read())
    }

    pub(crate) fn tag_from_cq_entry(&self, cqe: &CqEntry, queue_id: u32) -> Result<usize> {
        let tag = usize::from(cqe.task_tag());
        if tag >= self.transfer_slots {
            return Err(EINVAL);
        }
        if u32::from(cqe.submission_queue_id()) != queue_id {
            return Err(EINVAL);
        }

        let expected = io_project!(self.ucdl, [try: tag]).dma_handle() as u64;
        if !cqe.matches_ucd_base_addr(expected) {
            return Err(EIO);
        }

        Ok(tag)
    }

    fn map_request_prdt(
        &self,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<UfsPrdt> {
        let entries = KVec::new();
        if cmd.data_len() == 0 {
            return Ok(UfsPrdt {
                mapping: None,
                entries,
            });
        }

        if cmd.is_unmap() {
            return self.map_unmap_prdt(cmd);
        }

        let mut iter = rq
            .clone()
            .dma_map_iter(&self.dev, mempool.clone())
            .map_err(|_error| ENOMEM)?;
        let mut remaining = cmd.data_len();
        let mut entries = KVec::new();
        loop {
            let segment_address = iter.address();
            let segment_length = iter.length();
            if segment_length == 0 || segment_length > remaining {
                return Err(EINVAL);
            }

            append_prd_entries(&mut entries, segment_address, segment_length)?;

            remaining = remaining.saturating_sub(segment_length);
            if remaining == 0 {
                break;
            }

            iter.next()?;
        }
        // SAFETY: The mapping is stored in this request's private data. blk-mq
        // keeps the request alive by its tag until RUFS takes and drops the
        // mapping before completing or requeuing the request.
        let iter = unsafe { iter.finish_detached() };

        Ok(UfsPrdt {
            mapping: Some(UfsPrdtMapping::Sg(iter)),
            entries,
        })
    }

    fn map_unmap_prdt(&self, cmd: UfsSCSICmd) -> Result<UfsPrdt> {
        if cmd.unmap_blocks() == 0 {
            return Err(EINVAL);
        }

        let mut data = [0u8; UNMAP_PARAM_LIST_SIZE];

        // TODO: Define a type for this
        data[0..2].copy_from_slice(&22u16.to_be_bytes());
        data[2..4].copy_from_slice(&16u16.to_be_bytes());
        data[8..16].copy_from_slice(&cmd.unmap_lba().to_be_bytes());
        data[16..20].copy_from_slice(&cmd.unmap_blocks().to_be_bytes());

        // TODO: Consider using a dma pool instead of allocating for each unmap
        let buffer: Coherent<[u8]> = Coherent::from_slice(self.dev(), &data, GFP_ATOMIC)?;

        let mapping = UfsUnmapMapping {
            dev: self.dev.clone(),
            buffer,
        };

        let mut entries = KVec::with_capacity(1, GFP_ATOMIC)?;
        entries.push(
            PrdEntry {
                addr: mapping.buffer.dma_handle().to_le(),
                reserved: 0,
                size: ((UNMAP_PARAM_LIST_SIZE as u32) - 1).to_le(),
            },
            GFP_ATOMIC,
        )?;

        Ok(UfsPrdt {
            mapping: Some(UfsPrdtMapping::Unmap(mapping)),
            entries,
        })
    }

    pub(crate) fn fetch_devman_upiu(&self, cmd: UfsDevCmd, tag: usize) -> Result<UfsCmd> {
        let utrd = io_project!(self.utrdl, [try: tag]).copy_read();
        utrd.check_response()?;

        let rsp_upiu = io_project!(self.ucdl, [try: tag].rsp_upiu).copy_read();
        let cmd = rsp_upiu.fetch_dev(cmd)?;

        Ok(UfsCmd::Device(cmd))
    }

    pub(crate) fn fetch_mcq_devman_upiu(
        &self,
        cmd: UfsDevCmd,
        tag: usize,
        cqe: CqEntry,
    ) -> Result<UfsCmd> {
        match cqe.overall_status().into() {
            UtpOcs::Success => {}
            UtpOcs::InvalidCmdTableAttr => return Err(EINVAL),
            UtpOcs::InvalidPrdtAttr => return Err(EINVAL),
            UtpOcs::MismatchDataBufSize => return Err(EINVAL),
            UtpOcs::MisMatchRespUpiuSize => return Err(EINVAL),
            UtpOcs::InvalidCryptoConfig => return Err(EINVAL),
            UtpOcs::GeneralCryptoError => return Err(EINVAL),
            _ => return Err(EIO),
        }

        let rsp_upiu = io_project!(self.ucdl, [try: tag].rsp_upiu).copy_read();
        let cmd = rsp_upiu.fetch_dev(cmd)?;

        Ok(UfsCmd::Device(cmd))
    }

    pub(crate) fn fetch_scsi_completion(&self, tag: usize) -> UfsScsiResult {
        let utrd = match (|| -> Result<_> { Ok(io_project!(self.utrdl, [try: tag]).copy_read()) })()
        {
            Ok(utrd) => utrd,
            Err(_) => return UfsScsiResult::error(UtpOcs::InvalidCommandStatus as u8),
        };
        let ocs = utrd.ocs();

        if utrd.check_response().is_err() {
            return match utrd.ocs().into() {
                UtpOcs::Aborted | UtpOcs::InvalidCommandStatus => UfsScsiResult::requeue(ocs),
                _ => UfsScsiResult::error(ocs),
            };
        }

        match (|| -> Result<_> { Ok(io_project!(self.ucdl, [try: tag].rsp_upiu).copy_read()) })() {
            Ok(rsp_upiu) => rsp_upiu.scsi_result(ocs),
            Err(_) => UfsScsiResult::error(ocs),
        }
    }

    pub(crate) fn fetch_mcq_scsi_completion(&self, tag: usize, cqe: CqEntry) -> UfsScsiResult {
        let ocs = cqe.overall_status();

        if !matches!(ocs.into(), UtpOcs::Success) {
            return match ocs.into() {
                UtpOcs::Aborted | UtpOcs::InvalidCommandStatus => UfsScsiResult::requeue(ocs),
                _ => UfsScsiResult::error(ocs),
            };
        }

        match (|| -> Result<_> { Ok(io_project!(self.ucdl, [try: tag].rsp_upiu).copy_read()) })() {
            Ok(rsp_upiu) => rsp_upiu.scsi_result(ocs),
            Err(_) => UfsScsiResult::error(UtpOcs::InvalidCommandStatus as u8),
        }
    }
}
