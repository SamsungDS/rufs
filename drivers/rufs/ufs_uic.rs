// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::{prelude::*, new_mutex, new_spinlock};
use kernel::sync::{Arc, Mutex, SpinLock, Completion};
use kernel::time::Delta;
use crate::ufs_reg::*;
use crate::ufs_irq::*;

#[derive(Copy, Clone)]
enum UicCmdDme {
    Get         = 0x01,
    Set         = 0x02,
    PeerGet     = 0x03,
    PeerSet     = 0x04,
    PowerOn     = 0x10,
    PowerOff    = 0x11,
    Enable      = 0x12,
    Reset       = 0x14,
    EndPtRst    = 0x15,
    LinkStartup = 0x16,
    HibernEnter = 0x17,
    HibernExit  = 0x18,
    TestMode    = 0x1A,
}

enum UicCmdTimeoutMs {
    DEFAULT = 500,
    Max = 5000,
}

#[derive(Copy, Clone)]
struct UfsUicCmd {
    command: UicCmdDme,
    argument1: u32,
    argument2: u32,
    argument3: u32,
}

#[derive(Copy, Clone)]
enum UicCmdResult {
    Success = 0x00,
    InvalidAttr = 0x01,
    InvalidAttrValue = 0x02,
    ReadOnlyAttr = 0x03,
    WriteOnlyAttr = 0x04,
    BadIndex = 0x05,
    LockedAttr = 0x06,
    BadTestFeatureIndex = 0x07,
    PeerCommFailure = 0x08,
    Busy = 0x09,
    DmeFailure = 0x0A,
}

impl From<u32> for UicCmdResult {
    fn from(result: u32) -> Self {
        match result {
            0x00 => UicCmdResult::Success,
            0x01 => UicCmdResult::InvalidAttr,
            0x02 => UicCmdResult::InvalidAttrValue,
            0x03 => UicCmdResult::ReadOnlyAttr,
            0x04 => UicCmdResult::WriteOnlyAttr,
            0x05 => UicCmdResult::BadIndex,
            0x06 => UicCmdResult::LockedAttr,
            0x07 => UicCmdResult::BadTestFeatureIndex,
            0x08 => UicCmdResult::PeerCommFailure,
            0x09 => UicCmdResult::Busy,
            _ => UicCmdResult::DmeFailure,
        }
    }
}

#[derive(Copy, Clone)]
struct UfsUicRsp {
    result: UicCmdResult,
    value: u32,
}

#[pin_data]
pub(crate) struct UfsUic {
    reg: Arc<UfsReg>,
    irq: Arc<UfsIrq>,

    #[pin]
    cmd: Mutex<Option<UfsUicCmd>>,
    #[pin]
    rsp: SpinLock<Option<UfsUicRsp>>,
    #[pin]
    completion: Completion,
}

impl UfsUic {
    pub(crate) fn new(
        reg: Arc<UfsReg>,
        irq: Arc<UfsIrq>,
    ) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                reg,
                irq,
                cmd <- new_mutex!(None),
                rsp <- new_spinlock!(None),
                completion <- Completion::new(),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn link_startup(self: &Arc<Self>) -> Result<()> {
        /* Get SpinLock for UIC command */
        let mut locked_cmd = self.cmd.lock();

        let cmd = UfsUicCmd {
            command: UicCmdDme::LinkStartup,
            argument1: 0,
            argument2: 0,
            argument3: 0,
        };

        locked_cmd.replace(cmd);

        self.reg.enable_uic_interrupts();
        self.reg.wait_for_uic_cmd_ready(500, UicCmdTimeoutMs::DEFAULT as i64)?;

        self.dispatch_uic_cmd(cmd);
        self.wait_for_uic_cmd()?;

        self.reg.read_uic_error_phy();

        Ok(())
    }

    fn dispatch_uic_cmd(&self, cmd: UfsUicCmd) {
        self.reg.write_uic_arg1(cmd.argument1);
        self.reg.write_uic_arg2(cmd.argument2);
        self.reg.write_uic_arg3(cmd.argument3);
        self.reg.write_uic_cmd(cmd.command as u32);
    }

    fn wait_for_uic_cmd(&self) -> Result<()> {
        let delta = Delta::from_millis(UicCmdTimeoutMs::DEFAULT as i64);
        match self.completion.wait_for_completion_timeout(delta) {
            0 => Err(ETIMEDOUT),
            _ => {
                match *self.rsp.lock() {
                    None => Err(ENOMEM),
                    Some(rsp) => {
                        match rsp.result {
                           UicCmdResult:: Success => Ok(()),
                           _ => Err(EIO),
                        }
                    },
                }
            },
        }
    }

    pub(crate) fn get_uic_cmd_response(&self, interrupt_status: u32) {
        if is_uic_command_completion(interrupt_status) {
            let rsp = UfsUicRsp {
                result: self.reg.get_uic_cmd_result().into(),
                value: self.reg.get_dme_attr_val(),
            };
            self.rsp.lock().replace(rsp);

        } else if is_uic_power_mode(interrupt_status) {
            let rsp = UfsUicRsp {
                result: UicCmdResult::Success,
                value: 0,
            };
            self.rsp.lock().replace(rsp);
        };
    }

    pub(crate) fn complete_uic_cmd(&self) {
        self.completion.complete();
    }
}
