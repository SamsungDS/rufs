// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

mod prdt;

use crate::lu::UfsLuBlockOps;
use crate::protocol::query::*;
use crate::protocol::scsi::*;
use crate::protocol::upiu::Upiu;
use crate::protocol::UfsCmd;
use crate::reg::*;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::dma;
use kernel::io::io_project;
use kernel::io::Io;
use kernel::sync::{aref::ARef, Arc};
use kernel::{
    block::mq,
    device::{self, Bound, Core},
    pci,
    prelude::*,
};

pub(crate) use crate::hci::descriptor::{CqEntry, MAX_PRD_ENTRIES};
use crate::hci::descriptor::{SqEntry, Ucd, Utmrd, UtpOcs, Utrd};
use prdt::UfsPrdt;
pub(crate) use prdt::{UfsPrdtMapping, PRDT_DATA_BYTE_COUNT_MAX};

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
        let prdt = UfsPrdt::map(&self.dev, cmd, rq, mempool)?;
        let tag = usize::from(task_tag);

        io_project!(self.ucdl, [try: tag].cmd_upiu).copy_write(Upiu::command(cmd, tag));
        io_project!(self.ucdl, [try: tag].rsp_upiu).copy_write(Upiu::default());

        for (i, entry) in prdt.entries().iter().enumerate() {
            io_project!(self.ucdl, [try: tag].prdt[try: i]).copy_write(*entry);
        }

        let prd_entries = prdt.entries().len();
        let utrd = io_project!(self.utrdl, [try: tag]).copy_read();
        let utrd = utrd
            .build(UfsCmd::SCSI(cmd))
            .set_prd_table_length(prd_entries)?;
        io_project!(self.utrdl, [try: tag]).copy_write(utrd);

        Ok(prdt.into_mapping())
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
