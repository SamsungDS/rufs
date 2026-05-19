// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use kernel::block::mq;
use kernel::{bindings, prelude::*, kvec, new_spinlock};
use kernel::sync::{Arc, SpinLock, Completion};
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

#[derive(PartialEq, Copy, Clone)]
pub(crate) enum UfsScsiDataDirection {
    None,
    Read,
    Write,
}

#[derive(Copy, Clone)]
pub(crate) struct UfsSCSICmd {
    lun: u8,
    direction: UfsScsiDataDirection,
    data_len: u32,
    cdb: [u8; 16],
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
        }
    }

    pub(crate) fn unmap(lun: u8, data_len: u32) -> Self {
        let mut cdb = [0u8; 16];
        cdb[0] = UNMAP;
        cdb[8] = data_len as u8;

        Self {
            lun,
            direction: UfsScsiDataDirection::Write,
            data_len,
            cdb,
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

    pub(crate) fn get_scsi(&self) -> Result<UfsSCSICmd> {
        match *self {
            Self::SCSI(cmd) => Ok(cmd),
            _ => Err(EINVAL),
        }
    }
}

#[derive(PartialEq, Copy, Clone)]
enum RequestState {
    Idle,
    Submitted,
    Completed,
}

#[pin_data]
pub(crate) struct UfsRequest {
    queue: Arc<UfsQueue>,
    tag: usize,

    #[pin]
    cmd: SpinLock<Option<UfsCmd>>,
    #[pin]
    prdt: SpinLock<Option<UfsPrdtMapping>>,
    #[pin]
    block_rq: SpinLock<Option<ARef<mq::Request<UfsLuBlockOps>>>>,
    #[pin]
    state: SpinLock<RequestState>,
}

impl UfsRequest {
    fn new(queue: Arc<UfsQueue>, tag: usize) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                queue,
                tag,
                cmd <- new_spinlock!(None),
                prdt <- new_spinlock!(None),
                block_rq <- new_spinlock!(None),
                state <- new_spinlock!(RequestState::Idle),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn set_block_request(&self, rq: ARef<mq::Request<UfsLuBlockOps>>) -> Result<()> {
        let mut current = self.block_rq.lock();
        if current.is_some() {
            return Err(EBUSY);
        }

        current.replace(rq);
        Ok(())
    }

    pub(crate) fn issue(&self, cmd: UfsCmd) -> Result<UfsCmd> {
        self.compose(cmd)?;
        self.submit()?;
        self.wait()?;
        self.fetch()
    }

    pub(crate) fn compose(&self, cmd: UfsCmd) -> Result<()> {
        if self.cmd.lock().is_some() {
            pr_err!("command already exist in UfsRequest");
            return Err(EIO);
        }

        match cmd {
            UfsCmd::Device(cmd) => {
                self.queue.compose_dev(cmd, self.tag)?;
            },
            UfsCmd::SCSI(cmd) => return self.compose_scsi_cmd(cmd),
        }

        *self.prdt.lock() = None;
        self.cmd.lock().replace(cmd);
        Ok(())
    }

    #[inline(never)]
    fn compose_scsi_cmd(&self, cmd: UfsSCSICmd) -> Result<()> {
        let prdt = {
            let block_rq = self.block_rq.lock();
            let block_rq = block_rq.as_ref().ok_or(EINVAL)?;
            self.queue.compose_scsi(cmd, self.tag, block_rq)?
        };

        *self.prdt.lock() = prdt;
        self.cmd.lock().replace(UfsCmd::SCSI(cmd));
        Ok(())
    }

    pub(crate) fn submit(&self) -> Result<()> {
        let cmd = match *self.cmd.lock() {
            Some(cmd) => cmd,
            None => {
                pr_err!("no command in UfsRequest");
                return Err(EIO);
            },
        };

        *self.state.lock() = RequestState::Submitted;

        let result = match cmd {
            UfsCmd::Device(cmd) => {
                self.queue.prepare_dev_wait();
                self.queue.submit_dev(cmd, self.tag)
            },
            UfsCmd::SCSI(cmd) => self.queue.submit_scsi(cmd, self.tag),
        };

        match result {
            Err(e) => {
                *self.state.lock() = RequestState::Idle;
                *self.prdt.lock() = None;
                *self.block_rq.lock() = None;
                *self.cmd.lock() = None;
                Err(e)
            },
            Ok(()) => Ok(())
        }
    }

    pub(crate) fn wait(&self) -> Result<()> {
        let cmd = match *self.cmd.lock() {
            Some(cmd) => cmd,
            None => {
                pr_err!("no command in UfsRequest");
                return Err(EIO);
            },
        };

        if *self.state.lock() == RequestState::Idle {
            pr_err!("UfsRequest is not submitted");
            return Err(EIO);
        }

        let result = match cmd {
            UfsCmd::Device(cmd) => self.queue.wait_dev(cmd, self.tag),
            UfsCmd::SCSI(_) => Err(ENOTSUPP),
        };

        match result {
            Err(e) => {
                *self.state.lock() = RequestState::Idle;
                *self.prdt.lock() = None;
                *self.block_rq.lock() = None;
                *self.cmd.lock() = None;
                Err(e)
            },
            Ok(()) => Ok(()),
        }
    }

    pub(crate) fn fetch(&self) -> Result<UfsCmd> {
        let cmd = match *self.cmd.lock() {
            Some(cmd) => cmd,
            None => {
                pr_err!("no command in UfsRequest");
                return Err(EIO);
            },
        };

        if *self.state.lock() != RequestState::Completed {
            pr_err!("UfsRequest is not completed");
            return Err(EIO);
        }

        let result = match cmd {
            UfsCmd::Device(cmd) => self.queue.fetch_dev(cmd, self.tag),
            UfsCmd::SCSI(_) => Err(ENOTSUPP),
        };

        match result {
            Err(e) => {
                *self.state.lock() = RequestState::Idle;
                *self.prdt.lock() = None;
                *self.block_rq.lock() = None;
                *self.cmd.lock() = None;
                Err(e)
            },
            Ok(cmd) => {
                *self.state.lock() = RequestState::Idle;
                *self.prdt.lock() = None;
                *self.block_rq.lock() = None;
                *self.cmd.lock() = None;
                Ok(cmd)
            },
        }
    }

    pub(crate) fn clear(&self) {
        *self.state.lock() = RequestState::Idle;
        *self.prdt.lock() = None;
        *self.block_rq.lock() = None;
        *self.cmd.lock() = None;
    }

    // Interrupt Context
    fn complete(&self) {
        let cmd = match *self.cmd.lock() {
            Some(cmd) => cmd,
            None => {
                pr_err!("No command in UfsRequest");
                return;
            },
        };

        match cmd {
            UfsCmd::Device(cmd) => {
                *self.state.lock() = RequestState::Completed;
                self.queue.complete_dev(cmd, self.tag);
            },
            UfsCmd::SCSI(cmd) => {
                let block_rq = self.block_rq.lock().take();
                *self.state.lock() = RequestState::Idle;
                *self.prdt.lock() = None;
                *self.cmd.lock() = None;
                self.queue.complete_scsi(cmd, self.tag, block_rq);
            },
        };
    }
}

#[pin_data]
pub(crate) struct UfsQueue {
    reg: Arc<UfsReg>,
    irq: Arc<UfsIrq>,
    dma: Arc<UfsDma>,

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
        let slot = kvec![None; reg.nutrs()]?;

        Arc::pin_init(
            try_pin_init!(Self {
                reg,
                irq,
                dma,
                slot <- new_spinlock!(slot),
                completion <- Completion::new(),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn reserve(self: &Arc<Self>) -> Result<Arc<UfsRequest>> {
        let mut slots = self.slot.lock();
        let mut tag = slots.len() - 1;
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
                if *request.state.lock() == RequestState::Idle {
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

    fn submit_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        //self.outstanding_slots.set_bit_atomic(tag);
        self.reg.ring_utrl_doorbell(tag);
        Ok(())
    }

    fn submit_scsi(&self, cmd: UfsSCSICmd, tag: usize) -> Result<()> {
        self.reg.ring_utrl_doorbell(tag);
        Ok(())
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
        self.dma.fetch_devman_upiu(cmd, tag)
    }

    // Completion
    fn next_request(&self, mut tag: usize) -> Option<Arc<UfsRequest>> {
        while let Some(slot) = self.slot.lock().get_mut(tag) {
            match slot {
                Some(request) => { return Some(request.clone()); },
                None => { tag += 1; },
            }
        }
        None
    }

    fn next_submitted_request(&self, mut tag: usize) -> Option<Arc<UfsRequest>> {
        while let Some(request) = self.next_request(tag) {
            match *request.state.lock() {
                RequestState::Submitted => { return Some(request.clone()); },
                _ => { tag += 1; },
            }
        }
        None
    }

    pub(crate) fn complete(&self) {
        let doorbell = self.reg.read_utrl_doorbell();
        let mut tag = 0 as usize;
        while let Some(request) = self.next_submitted_request(tag) {
            let request_tag = request.tag;
            if (doorbell & (1 << request_tag)) == 0 {
                request.complete();
            }
            tag = request_tag + 1;
        }
    }

    fn complete_dev(&self, cmd: UfsDevCmd, tag: usize) {
        self.completion.complete();
    }

    fn complete_scsi(
        &self,
        cmd: UfsSCSICmd,
        tag: usize,
        rq: Option<ARef<mq::Request<UfsLuBlockOps>>>,
    ) {
        let Some(rq) = rq else {
            pr_err!("No block request for SCSI completion tag {}", tag);
            return;
        };

        let status = match self.dma.fetch_scsi_completion(tag) {
            UfsScsiCompletion::Good => bindings::BLK_STS_OK,
            UfsScsiCompletion::Busy
            | UfsScsiCompletion::TaskSetFull
            | UfsScsiCompletion::Requeue => bindings::BLK_STS_RESOURCE,
            UfsScsiCompletion::TaskAborted => bindings::BLK_STS_TARGET,
            UfsScsiCompletion::ReservationConflict => bindings::BLK_STS_RESV_CONFLICT,
            UfsScsiCompletion::CheckCondition | UfsScsiCompletion::Error => {
                pr_err!("SCSI request failed tag {}\n", tag);
                bindings::BLK_STS_IOERR
            }
        };

        // Take exclusive ownership of the request so it can be handed back to
        // the block layer.
        let Ok(rq) = OwnableRefCounted::try_from_shared(rq) else {
            pr_err!("Failed to complete SCSI request tag {}\n", tag);
            return;
        };
        rq.end(status as u8);
    }
}
