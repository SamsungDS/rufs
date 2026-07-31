// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use crate::reg::*;
use kernel::sync::{Arc, Completion, Mutex, SpinLock};
use kernel::time::Delta;
use kernel::{new_mutex, new_spinlock, prelude::*};

#[derive(Copy, Clone)]
enum UicCmdDme {
    Get = 0x01,
    Set = 0x02,
    PeerGet = 0x03,
    PeerSet = 0x04,
    PowerOn = 0x10,
    PowerOff = 0x11,
    Enable = 0x12,
    Reset = 0x14,
    EndPtRst = 0x15,
    LinkStartup = 0x16,
    HibernEnter = 0x17,
    HibernExit = 0x18,
    TestMode = 0x1A,
}

enum UicCmdTimeoutMs {
    DEFAULT = 500,
    Max = 5000,
}

#[derive(Copy, Clone, Debug)]
pub(crate) enum UfsPaPwrMode {
    Fast = 1,
    Slow = 2,
}

#[derive(Copy, Clone)]
pub(crate) struct UfsPaLayerAttr {
    pub(crate) gear_rx: u32,
    pub(crate) gear_tx: u32,
    pub(crate) lane_rx: u32,
    pub(crate) lane_tx: u32,
    pub(crate) pwr_rx: UfsPaPwrMode,
    pub(crate) pwr_tx: UfsPaPwrMode,
    pub(crate) hs_rate: u32,
}

#[derive(Copy, Clone)]
struct UfsUicCmd {
    command: UicCmdDme,
    argument1: u32,
    argument2: u32,
    argument3: u32,
    expected_completion: UicCompletion,
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum UicCompletion {
    Command,
    PowerMode,
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
    PowerModeChange,
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

    #[pin]
    cmd: Mutex<Option<UfsUicCmd>>,
    #[pin]
    rsp: SpinLock<Option<UfsUicRsp>>,
    #[pin]
    completion: Completion,
}

impl UfsUic {
    pub(crate) fn new(reg: Arc<UfsReg>) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                reg,
                cmd <- new_mutex!(None),
                rsp <- new_spinlock!(None),
                completion <- Completion::new(),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn link_startup(self: &Arc<Self>) -> Result<()> {
        self.send_uic_cmd(UfsUicCmd {
            command: UicCmdDme::LinkStartup,
            argument1: 0,
            argument2: 0,
            argument3: 0,
            expected_completion: UicCompletion::Command,
        })?;

        self.reg.read_uic_error_phy();
        self.reg.confirm_uic_error();

        Ok(())
    }

    pub(crate) fn max_power_mode(&self) -> Result<UfsPaLayerAttr> {
        let lane_rx = self.dme_get(PA_CONNECTEDRXDATALANES)?;
        let lane_tx = self.dme_get(PA_CONNECTEDTXDATALANES)?;
        if lane_rx == 0 || lane_tx == 0 || lane_rx != lane_tx {
            pr_err!(
                "[RUFS] ufs_uic: invalid connected lanes rx={} tx={}\n",
                lane_rx,
                lane_tx
            );
            return Err(EINVAL);
        }

        let mut pwr_rx = UfsPaPwrMode::Fast;
        let mut pwr_tx = UfsPaPwrMode::Fast;
        let mut gear_rx = self.dme_get(PA_MAXRXHSGEAR)?;
        let mut gear_tx = self.dme_peer_get(PA_MAXRXHSGEAR)?;

        if gear_rx == 0 {
            gear_rx = self.dme_get(PA_MAXRXPWMGEAR)?;
            pwr_rx = UfsPaPwrMode::Slow;
        }
        if gear_tx == 0 {
            gear_tx = self.dme_peer_get(PA_MAXRXPWMGEAR)?;
            pwr_tx = UfsPaPwrMode::Slow;
        }
        if gear_rx == 0 || gear_tx == 0 {
            pr_err!(
                "[RUFS] ufs_uic: invalid max gear rx={} tx={}\n",
                gear_rx,
                gear_tx
            );
            return Err(EINVAL);
        }

        Ok(UfsPaLayerAttr {
            gear_rx,
            gear_tx,
            lane_rx,
            lane_tx,
            pwr_rx,
            pwr_tx,
            hs_rate: PA_HS_MODE_B,
        })
    }

    pub(crate) fn change_power_mode(&self, pwr_mode: UfsPaLayerAttr) -> Result<()> {
        pr_info!(
            "[RUFS] ufs_uic: configure power mode gear_rx={} gear_tx={} lane_rx={} lane_tx={} pwr_rx={:?} pwr_tx={:?} hs_rate={}\n",
            pwr_mode.gear_rx,
            pwr_mode.gear_tx,
            pwr_mode.lane_rx,
            pwr_mode.lane_tx,
            pwr_mode.pwr_rx,
            pwr_mode.pwr_tx,
            pwr_mode.hs_rate,
        );

        self.dme_set(PA_RXGEAR, pwr_mode.gear_rx)?;
        self.dme_set(PA_ACTIVERXDATALANES, pwr_mode.lane_rx)?;
        self.dme_set(PA_RXTERMINATION, pwr_mode.pwr_rx.uses_termination() as u32)?;

        self.dme_set(PA_TXGEAR, pwr_mode.gear_tx)?;
        self.dme_set(PA_ACTIVETXDATALANES, pwr_mode.lane_tx)?;
        self.dme_set(PA_TXTERMINATION, pwr_mode.pwr_tx.uses_termination() as u32)?;

        if pwr_mode.pwr_rx.uses_termination() || pwr_mode.pwr_tx.uses_termination() {
            self.dme_set(PA_HSSERIES, pwr_mode.hs_rate)?;
        }

        self.dme_set(PA_PWRMODEUSERDATA0, DL_FC0_PROTECTION_TIMEOUT_VAL_DEFAULT)?;
        self.dme_set(PA_PWRMODEUSERDATA1, DL_TC0_REPLAY_TIMEOUT_VAL_DEFAULT)?;
        self.dme_set(PA_PWRMODEUSERDATA2, DL_AFC0_REQ_TIMEOUT_VAL_DEFAULT)?;
        self.dme_set(PA_PWRMODEUSERDATA3, DL_FC1_PROTECTION_TIMEOUT_VAL_DEFAULT)?;
        self.dme_set(PA_PWRMODEUSERDATA4, DL_TC1_REPLAY_TIMEOUT_VAL_DEFAULT)?;
        self.dme_set(PA_PWRMODEUSERDATA5, DL_AFC1_REQ_TIMEOUT_VAL_DEFAULT)?;

        self.dme_set(
            DME_LOCAL_FC0_PROTECTION_TIMEOUT_VAL,
            DL_FC0_PROTECTION_TIMEOUT_VAL_DEFAULT,
        )?;
        self.dme_set(
            DME_LOCAL_TC0_REPLAY_TIMEOUT_VAL,
            DL_TC0_REPLAY_TIMEOUT_VAL_DEFAULT,
        )?;
        self.dme_set(
            DME_LOCAL_AFC0_REQ_TIMEOUT_VAL,
            DL_AFC0_REQ_TIMEOUT_VAL_DEFAULT,
        )?;

        self.dme_set(PA_PWRMODE, pwr_mode.pwrmode_value())
    }

    fn dme_get(&self, attr: u32) -> Result<u32> {
        self.send_uic_cmd(UfsUicCmd {
            command: UicCmdDme::Get,
            argument1: uic_arg_mib(attr),
            argument2: 0,
            argument3: 0,
            expected_completion: UicCompletion::Command,
        })
    }

    fn dme_peer_get(&self, attr: u32) -> Result<u32> {
        self.send_uic_cmd(UfsUicCmd {
            command: UicCmdDme::PeerGet,
            argument1: uic_arg_mib(attr),
            argument2: 0,
            argument3: 0,
            expected_completion: UicCompletion::Command,
        })
    }

    fn dme_set(&self, attr: u32, value: u32) -> Result<()> {
        self.send_uic_cmd(UfsUicCmd {
            command: UicCmdDme::Set,
            argument1: uic_arg_mib(attr),
            argument2: uic_arg_attr_type(0),
            argument3: value,
            expected_completion: if attr == PA_PWRMODE {
                UicCompletion::PowerMode
            } else {
                UicCompletion::Command
            },
        })?;

        Ok(())
    }

    fn send_uic_cmd(&self, cmd: UfsUicCmd) -> Result<u32> {
        self.cmd.lock().take();
        self.completion.reinit();
        self.rsp.lock().take();

        self.reg.enable_uic_interrupts();
        self.reg
            .wait_for_uic_cmd_ready(500, UicCmdTimeoutMs::DEFAULT as i64)?;

        self.cmd.lock().replace(cmd);
        self.dispatch_uic_cmd(cmd);
        let result = self.wait_for_uic_cmd();
        self.cmd.lock().take();
        result
    }

    fn dispatch_uic_cmd(&self, cmd: UfsUicCmd) {
        self.reg.write_uic_arg1(cmd.argument1);
        self.reg.write_uic_arg2(cmd.argument2);
        self.reg.write_uic_arg3(cmd.argument3);
        self.reg.write_uic_cmd(cmd.command as u32);
    }

    fn wait_for_uic_cmd(&self) -> Result<u32> {
        let delta = Delta::from_millis(UicCmdTimeoutMs::DEFAULT as i64);
        match self.completion.wait_for_completion_timeout(delta) {
            0 => Err(ETIMEDOUT),
            _ => {
                let rsp = self.rsp.lock().take();
                match rsp {
                    Some(UfsUicRsp {
                        result: UicCmdResult::Success,
                        value,
                    }) => Ok(value),
                    Some(UfsUicRsp {
                        result: UicCmdResult::PowerModeChange,
                        value: PWR_LOCAL,
                    }) => Ok(PWR_LOCAL),
                    Some(UfsUicRsp {
                        result: UicCmdResult::PowerModeChange,
                        value,
                    }) => {
                        pr_err!(
                            "[RUFS] ufs_uic: power mode change failed status={}\n",
                            value
                        );
                        Err(EIO)
                    }
                    Some(_) => Err(EIO),
                    None => Err(ENOMEM),
                }
            }
        }
    }

    pub(crate) fn handle_uic_completion(&self, interrupt_status: u32) -> bool {
        let expected_completion = self.cmd.lock().as_ref().map(|cmd| cmd.expected_completion);

        if expected_completion == Some(UicCompletion::Command)
            && is_uic_command_completion(interrupt_status)
        {
            let rsp = UfsUicRsp {
                result: self.reg.get_uic_cmd_result().into(),
                value: self.reg.get_dme_attr_val(),
            };
            self.rsp.lock().replace(rsp);
            true
        } else if expected_completion == Some(UicCompletion::PowerMode)
            && is_uic_power_mode(interrupt_status)
        {
            let rsp = UfsUicRsp {
                result: UicCmdResult::PowerModeChange,
                value: self.reg.get_power_mode_change_status(),
            };
            self.rsp.lock().replace(rsp);
            true
        } else {
            false
        }
    }

    pub(crate) fn complete_uic_cmd(&self) {
        self.completion.complete();
    }
}

impl UfsPaPwrMode {
    fn uses_termination(self) -> bool {
        matches!(self, Self::Fast)
    }
}

impl UfsPaLayerAttr {
    fn pwrmode_value(self) -> u32 {
        ((self.pwr_rx as u32) << PWRMODE_RX_OFFSET) | self.pwr_tx as u32
    }
}

const PA_CONNECTEDRXDATALANES: u32 = 0x1581;
const PA_CONNECTEDTXDATALANES: u32 = 0x1561;
const PA_MAXRXHSGEAR: u32 = 0x1587;
const PA_MAXRXPWMGEAR: u32 = 0x1586;
const PA_RXGEAR: u32 = 0x1583;
const PA_ACTIVERXDATALANES: u32 = 0x1580;
const PA_RXTERMINATION: u32 = 0x1584;
const PA_TXGEAR: u32 = 0x1568;
const PA_ACTIVETXDATALANES: u32 = 0x1560;
const PA_TXTERMINATION: u32 = 0x1569;
const PA_HSSERIES: u32 = 0x156A;
const PA_PWRMODE: u32 = 0x1571;
const PA_PWRMODEUSERDATA0: u32 = 0x15B0;
const PA_PWRMODEUSERDATA1: u32 = 0x15B1;
const PA_PWRMODEUSERDATA2: u32 = 0x15B2;
const PA_PWRMODEUSERDATA3: u32 = 0x15B3;
const PA_PWRMODEUSERDATA4: u32 = 0x15B4;
const PA_PWRMODEUSERDATA5: u32 = 0x15B5;
const PA_HS_MODE_B: u32 = 2;
const PWRMODE_RX_OFFSET: u32 = 4;
const PWR_LOCAL: u32 = 1;

const DL_FC0_PROTECTION_TIMEOUT_VAL_DEFAULT: u32 = 8191;
const DL_TC0_REPLAY_TIMEOUT_VAL_DEFAULT: u32 = 65535;
const DL_AFC0_REQ_TIMEOUT_VAL_DEFAULT: u32 = 32767;
const DL_FC1_PROTECTION_TIMEOUT_VAL_DEFAULT: u32 = 8191;
const DL_TC1_REPLAY_TIMEOUT_VAL_DEFAULT: u32 = 65535;
const DL_AFC1_REQ_TIMEOUT_VAL_DEFAULT: u32 = 32767;

const DME_LOCAL_FC0_PROTECTION_TIMEOUT_VAL: u32 = 0xD041;
const DME_LOCAL_TC0_REPLAY_TIMEOUT_VAL: u32 = 0xD042;
const DME_LOCAL_AFC0_REQ_TIMEOUT_VAL: u32 = 0xD043;

const fn uic_arg_mib(attr: u32) -> u32 {
    (attr & 0xFFFF) << 16
}

const fn uic_arg_attr_type(attr_type: u32) -> u32 {
    (attr_type & 0xFF) << 16
}
