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
    state: RequestState,
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
                if (self.queue.reg.read_utrl_doorbell() & (1 << self.tag)) == 0 {
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
        inner.cmd = None;
        inner.state = RequestState::Idle;
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
                let (block_rq, prdt) = {
                    let mut inner = self.inner.lock();
                    let Some(block_rq) = inner.block_rq.take() else {
                        pr_err!("No block request for SCSI completion tag {}", self.tag);
                        return true;
                    };

                    (block_rq, inner.prdt.take())
                };

                drop(prdt);

                match self.queue.complete_scsi(cmd, self.tag, block_rq) {
                    Ok(()) => {
                        self.clear();
                        true
                    },
                    Err(block_rq) => {
                        let mut inner = self.inner.lock();
                        inner.block_rq = Some(block_rq);
                        false
                    },
                }
            },
        }
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

    fn next_submitted_request(&self, mut tag: usize) -> Option<Arc<UfsRequest>> {
        while let Some(request) = self.next_request(tag) {
            match request.inner.lock().state {
                RequestState::Submitted => { return Some(request.clone()); },
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
        while let Some(request) = self.next_submitted_request(tag) {
            let request_tag = request.tag;
            let doorbell = self.reg.read_utrl_doorbell();
            if (doorbell & (1 << request_tag)) == 0 {
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
        rq: ARef<mq::Request<UfsLuBlockOps>>,
    ) -> Result<(), ARef<mq::Request<UfsLuBlockOps>>> {
        let result = self.dma.fetch_scsi_completion(tag);
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
