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
    device::{self, Bound},
    prelude::*,
};

use crate::hci::descriptor::{Ucd, Utmrd, UtpOcs, Utrd};
use prdt::UfsPrdt;
pub(crate) use crate::hci::descriptor::{CqEntry, MAX_PRD_ENTRIES};
pub(crate) use prdt::{UfsPrdtMapping, PRDT_DATA_BYTE_COUNT_MAX};

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
        dev: &device::Device<Bound>,
        reg: Arc<UfsReg>,
        transfer_slots: usize,
    ) -> Result<Arc<Self>> {
        if transfer_slots == 0 {
            return Err(EINVAL);
        }
        let ucdl = dma::Coherent::<Ucd>::zeroed_slice(dev, transfer_slots, GFP_KERNEL)?;

        let utrdl = dma::Coherent::<Utrd>::zeroed_slice(dev, transfer_slots, GFP_KERNEL)?;

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
        let utmrdl = dma::Coherent::<Utmrd>::zeroed_slice(dev, nutmrs, GFP_KERNEL)?;

        Ok(Arc::new(
            Self {
                reg,
                dev: dev.into(),
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

    pub(crate) fn validate_cq_entry(&self, cqe: &CqEntry, queue_id: u32) -> Result<()> {
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

        Ok(())
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
