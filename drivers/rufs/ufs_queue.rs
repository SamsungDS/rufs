// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use kernel::{prelude::*, kvec, new_spinlock};
use kernel::sync::{Arc, SpinLock, Completion};
use kernel::time::Delta;
use crate::ufs_reg::*;
use crate::ufs_dma::*;
use crate::ufs_irq::*;

#[derive(Copy, Clone)]
pub(crate) enum UfsDevCmd {

}

impl UfsDevCmd {
    fn timeout(&self) -> Delta {
        Delta::from_millis(1000) //TEMP
    }
}

#[derive(Copy, Clone)]
pub(crate) enum UfsSCSICmd {}

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
    state: SpinLock<RequestState>,
}

impl UfsRequest {
    fn new(queue: Arc<UfsQueue>, tag: usize) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                queue,
                tag,
                cmd <- new_spinlock!(None),
                state <- new_spinlock!(RequestState::Idle),
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
        if self.cmd.lock().is_some() {
            pr_err!("command already exist in UfsRequest");
            return Err(EIO);
        }

        let result = match cmd {
            UfsCmd::Device(cmd) => self.queue.compose_dev(cmd, self.tag),
            UfsCmd::SCSI(_) => Err(ENOTSUPP),
        };

        match result {
            Err(e) => Err(e),
            Ok(()) => { self.cmd.lock().replace(cmd); Ok(()) },
        }
    }

    pub(crate) fn submit(&self) -> Result<()> {
        let cmd = match *self.cmd.lock() {
            Some(cmd) => cmd,
            None => {
                pr_err!("no command in UfsRequest");
                return Err(EIO);
            },
        };

        let result = match cmd {
            UfsCmd::Device(cmd) => self.queue.submit_dev(cmd, self.tag),
            UfsCmd::SCSI(_) => Err(ENOTSUPP),
        };

        match result {
            Err(e) => { *self.cmd.lock() = None; Err(e) },
            Ok(()) => { *self.state.lock() = RequestState::Submitted; Ok(()) }
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
            Err(e) => { *self.cmd.lock() = None; Err(e) },
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
            Err(e) => { *self.cmd.lock() = None; Err(e) },
            Ok(cmd) => { *self.cmd.lock() = None; Ok(cmd) },
        }
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
            UfsCmd::Device(cmd) => self.queue.complete_dev(cmd, self.tag),
            UfsCmd::SCSI(cmd) => {},
        };

        *self.state.lock() = RequestState::Completed;
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
            Some(_) => Err(EINVAL),
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

    fn submit_dev(&self, cmd: UfsDevCmd, tag: usize) -> Result<()> {
        //self.outstanding_slots.set_bit_atomic(tag);
        self.reg.ring_utrl_doorbell(tag);
        Ok(())
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
            if (doorbell & (1 << tag)) == 0 {
                request.complete();
            }
            tag += 1;
        }
    }

    fn complete_dev(&self, cmd: UfsDevCmd, tag: usize) {
        self.completion.complete();
    }
}
