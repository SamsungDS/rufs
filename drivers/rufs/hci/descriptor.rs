// SPDX-License-Identifier: GPL-2.0

//! UFSHCI request and queue descriptor layouts.

use crate::protocol::scsi::*;
use crate::protocol::upiu::{Upiu, UpiuTmReq, UpiuTmRsp};
use crate::protocol::UfsCmd;
use kernel::bits::{genmask_u64, genmask_u8};
use kernel::{dma, prelude::*};

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
