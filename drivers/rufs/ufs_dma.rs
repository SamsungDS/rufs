// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use kernel::{bindings, device, pci, device::{Bound, Core}, prelude::*, new_spinlock};
use kernel::{dma, dma_read, dma_write};
use kernel::bits::genmask_u8;
use kernel::sync::{Arc, SpinLock, aref::ARef};
use crate::ufs_reg::*;
use crate::ufs_queue::*;
use crate::ufs_dev::*;

const PRDT_DATA_BYTE_COUNT_MAX: u32 = 0x00040000; // SZ_256K
const PRDT_DATA_BYTE_COUNT_PAD: usize = 4;
const UNMAP_PARAM_LIST_SIZE: usize = 24;
const ALIGNED_UPIU_SIZE: usize = 512;
const MAX_PRD_ENTRIES: usize = 256;
const UFS_CDB_SIZE: usize = 16;
const UFS_SENSE_SIZE: usize = 18;
const MASK_OCS: u8 = 0x0F;
const SAM_STAT_GOOD: u8 = 0x00;
const SAM_STAT_CHECK_CONDITION: u8 = 0x02;
const SAM_STAT_BUSY: u8 = 0x08;
const SAM_STAT_RESERVATION_CONFLICT: u8 = 0x18;
const SAM_STAT_TASK_SET_FULL: u8 = 0x28;
const SAM_STAT_TASK_ABORTED: u8 = 0x40;

#[derive(Clone, Copy, Debug)]
pub(crate) enum UfsScsiCompletion {
    Good,
    CheckCondition,
    Busy,
    ReservationConflict,
    TaskSetFull,
    TaskAborted,
    Requeue,
    Error,
}

#[derive(Clone, Copy)]
pub(crate) struct UfsScsiResult {
    pub(crate) completion: UfsScsiCompletion,
    pub(crate) ocs: u8,
    pub(crate) transaction: u8,
    pub(crate) response: u8,
    pub(crate) status: u8,
    pub(crate) residual_transfer_count: u32,
    pub(crate) sense_data_len: usize,
    pub(crate) sense_data: [u8; UFS_SENSE_SIZE],
}

impl UfsScsiResult {
    pub(crate) fn error(ocs: u8) -> Self {
        Self {
            completion: UfsScsiCompletion::Error,
            ocs,
            transaction: 0,
            response: 0,
            status: 0,
            residual_transfer_count: 0,
            sense_data_len: 0,
            sense_data: [0; UFS_SENSE_SIZE],
        }
    }

    fn requeue(ocs: u8) -> Self {
        Self {
            completion: UfsScsiCompletion::Requeue,
            ..Self::error(ocs)
        }
    }
}

// UPIU
enum UpiuFlag {
    None    = 0x00,
    CP      = 0x04,
    Write   = 0x20,
    Read    = 0x40,
}

enum UpiuTransaction {
    NopOut      = 0x00,
    Command     = 0x01,
    DataOut     = 0x02,
    TaskReq     = 0x04,
    QueryReq    = 0x16,
    NopIn       = 0x20,
    Response    = 0x21,
    DataIn      = 0x22,
    TaskRsp     = 0x24,
    ReadyXfer   = 0x31,
    QueryRsp    = 0x36,
    Reject  = 0x3F,
}

impl From<u8> for UpiuTransaction {
    fn from(code: u8) -> Self {
        match code {
            0x00 => Self::NopOut,
            0x01 => Self::Command,
            0x02 => Self::DataOut,
            0x04 => Self::TaskReq,
            0x16 => Self::QueryReq,
            0x20 => Self::NopIn,
            0x21 => Self::Response,
            0x22 => Self::DataIn,
            0x24 => Self::TaskRsp,
            0x31 => Self::ReadyXfer,
            0x36 => Self::QueryRsp,
            _ => Self::Reject,
        }
    }
}

enum UpiuQueryFunction {
    StandardRead    = 0x01,
    StandardWrite   = 0x81,
}

enum UpiuResponse {
    Success = 0x00,
    ParamNotReadable = 0xF6,
    ParamNotWritable = 0xF7,
    ParamAlreadyWritten = 0xF8,
    InvalidLen = 0xF9,
    InvalidVal = 0xFA,
    InvalidSel = 0xFB,
    InvalidIndex = 0xFc,
    InvalidIdn = 0xFD,
    InvalidOp = 0xFE,
    Failure = 0xFF,
}

impl From<u8> for UpiuResponse {
    fn from(code: u8) -> Self {
        match code {
            0x00 => Self::Success,
            0xF6 => Self::ParamNotReadable,
            0xF7 => Self::ParamNotWritable,
            0xF8 => Self::ParamAlreadyWritten,
            0xF9 => Self::InvalidLen,
            0xFA => Self::InvalidVal,
            0xFB => Self::InvalidSel,
            0xFC => Self::InvalidIndex,
            0xFD => Self::InvalidIdn,
            0xFE => Self::InvalidOp,
            _ => Self::Failure,
        }
    }
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy)]
struct UpiuHeader {
    transaction_code: u8,
    flags: u8,
    lun: u8,
    task_tag: u8,
    cmd_set: u8,
    query_func: u8,
    response: u8,
    status: u8,
    ehs_length: u8,
    dev_info: u8,
    data_seg_len: u16, // (BE)
}

impl UpiuHeader {
    fn nop_out(tag: usize) -> Self {
        Self {
            transaction_code: UpiuTransaction::NopOut as u8,
            task_tag: tag as u8,
            ..Default::default()
        }
    }

    fn query_read(tag: usize) -> Self {
        Self {
            transaction_code: UpiuTransaction::QueryReq as u8,
            task_tag: tag as u8,
            query_func: UpiuQueryFunction::StandardRead as u8,
            ..Default::default()
        }
    }

    fn query_write(tag: usize) -> Self {
        Self {
            transaction_code: UpiuTransaction::QueryReq as u8,
            task_tag: tag as u8,
            query_func: UpiuQueryFunction::StandardWrite as u8,
            ..Default::default()
        }
    }

    fn query_write_data(tag: usize, length: usize) -> Self {
        Self {
            transaction_code: UpiuTransaction::QueryReq as u8,
            task_tag: tag as u8,
            query_func: UpiuQueryFunction::StandardWrite as u8,
            data_seg_len: (length as u16).to_be(),
            ..Default::default()
        }
    }

    fn command(cmd: UfsSCSICmd, tag: usize) -> Self {
        let flags = match cmd.direction() {
            UfsScsiDataDirection::Read => UpiuFlag::Read,
            UfsScsiDataDirection::Write => UpiuFlag::Write,
            UfsScsiDataDirection::None => UpiuFlag::None,
        };

        Self {
            transaction_code: UpiuTransaction::Command as u8,
            flags: flags as u8,
            lun: cmd.lun(),
            task_tag: tag as u8,
            cmd_set: 0,
            ..Default::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuCmd {
    exp_data_transfer_len: u32, // (BE)
    cdb: [u8; UFS_CDB_SIZE],
    _padding: [u8; 480],
}

impl Default for UpiuCmd {
    fn default() -> Self {
        Self {
            exp_data_transfer_len: 0,
            cdb: [0; UFS_CDB_SIZE],
            _padding: [0; 480],
        }
    }
}

impl UpiuCmd {
    fn command(cmd: UfsSCSICmd) -> Self {
        Self {
            exp_data_transfer_len: cmd.data_len().to_be(),
            cdb: cmd.cdb(),
            ..Default::default()
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuRsp {
    residual_transfer_count: u32, // (BE)
    reserved: [u32; 4],
    sendse_data_len: u16, // (BE)
    sense_data: [u8; UFS_SENSE_SIZE],
    _padding: [u8; 460],
}

impl Default for UpiuRsp {
    fn default() -> Self {
        Self {
            residual_transfer_count: 0,
            reserved: [0; 4],
            sendse_data_len: 0,
            sense_data: [0; UFS_SENSE_SIZE],
            _padding: [0; 460],
        }
    }
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy)]
struct UpiuTmReq {
    header: UpiuHeader,
    input_param1: u32,  // (BE)
    input_param2: u32,  // (BE)
    input_param3: u32,  // (BE)
    reserved: [u32; 2],
    // Task Management doesn't have padding
    // because it is pre-allocated in UTMRD DMA aree directly
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy)]
struct UpiuTmRsp {
    header: UpiuHeader,
    output_param1: u32, // (BE)
    output_param2: u32, // (BE)
    reserved: [u32; 3],
    // Task Management doesn't have padding
    // because it is pre-allocated in UTMRD DMA aree directly
}

enum QueryOpcode {
    Nop         = 0x0,
    ReadDesc    = 0x1,
    WriteDesc   = 0x2,
    ReadAttr    = 0x3,
    WriteAttr   = 0x4,
    ReadFlag    = 0x5,
    SetFlag     = 0x6,
    ClearFlag   = 0x7,
    ToggleFlag  = 0x8,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct DescBuffer {
    pub(crate) data: [u8; QUERY_DESC_MAX_SIZE],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuReadDescReq {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: u16,
    length: u16, // (BE)
    _reserved2: [u32; 3],
    _padding: [u8; 480],
}

impl UpiuReadDescReq {
    fn build(cmd: UfsDescCmd) -> Self {
        Self {
            opcode: QueryOpcode::ReadDesc as u8,
            idn: cmd.idn as u8,
            index: cmd.index,
            selector: cmd.selector,
            _reserved: 0,
            length: cmd.length.to_be(),
            _reserved2: [0; 3],
            _padding: [0; 480],
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuWriteDescReq {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: u16,
    length: u16, // (BE)
    _reserved2: [u32; 3],
    buffer: DescBuffer,
    _padding: [u8; 225],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuReadAttrReq {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: [u32; 4],
    _padding: [u8; 480],
}

impl UpiuReadAttrReq {
    fn build(cmd: UfsAttrCmd) -> Self {
        Self {
            opcode: QueryOpcode::ReadAttr as u8,
            idn: cmd.idn as u8,
            index: cmd.index,
            selector: cmd.selector,
            _reserved: [0; 4],
            _padding: [0; 480],
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuWriteAttrReq {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    value: u64, // (BE)
    _reserved: [u32; 2],
    _padding: [u8; 480],
}

impl UpiuWriteAttrReq {
    fn build(cmd: UfsAttrCmd) -> Self {
        Self {
            opcode: QueryOpcode::WriteAttr as u8,
            idn: cmd.idn as u8,
            index: cmd.index,
            selector: cmd.selector,
            value: cmd.value.to_be(),
            _reserved: [0; 2],
            _padding: [0; 480],
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuFlagReq {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: [u32; 4],
    _padding: [u8; 480],
}

impl UpiuFlagReq {
    fn build(cmd: UfsFlagCmd, opcode: QueryOpcode) -> Self {
        Self {
            opcode: opcode as u8,
            idn: cmd.idn as u8,
            index: cmd.index,
            selector: cmd.selector,
            _reserved: [0; 4],
            _padding: [0; 480],
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
union UpiuQueryReq {
    read_desc: UpiuReadDescReq,
    write_desc: UpiuWriteDescReq,
    read_attr: UpiuReadAttrReq,
    write_attr: UpiuWriteAttrReq,
    read_flag: UpiuFlagReq,
    set_flag: UpiuFlagReq,
    clear_flag: UpiuFlagReq,
    toggle_flag: UpiuFlagReq,
}

impl UpiuQueryReq {
    fn read_desc(cmd: UfsDescCmd) -> Self {
        Self { read_desc: UpiuReadDescReq::build(cmd) }
    }

    fn read_attr(cmd: UfsAttrCmd) -> Self {
        Self { read_attr: UpiuReadAttrReq::build(cmd) }
    }

    fn write_attr(cmd: UfsAttrCmd) -> Self {
        Self { write_attr: UpiuWriteAttrReq::build(cmd) }
    }

    fn read_flag(cmd: UfsFlagCmd) -> Self {
        Self { read_flag: UpiuFlagReq::build(cmd, QueryOpcode::ReadFlag) }
    }

    fn set_flag(cmd: UfsFlagCmd) -> Self {
        Self { set_flag: UpiuFlagReq::build(cmd, QueryOpcode::SetFlag) }
    }

    fn clear_flag(cmd: UfsFlagCmd) -> Self {
        Self { clear_flag: UpiuFlagReq::build(cmd, QueryOpcode::ClearFlag) }
    }

    fn toggle_flag(cmd: UfsFlagCmd) -> Self {
        Self { toggle_flag: UpiuFlagReq::build(cmd, QueryOpcode::ToggleFlag) }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuReadDescRsp {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: u16,
    length: u16, // (BE)
    _reserved2: [u32; 3],
    buffer: DescBuffer,
    _padding: [u8; 225],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuWriteDescRsp {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: u16,
    length: u16, // (BE)
    _reserved2: [u32; 3],
    _padding: [u8; 480],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuAttrRsp {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    value: u64, // (BE)
    _reserved: [u32; 2],
    _padding: [u8; 480],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuFlagRsp {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: [u8; 7],
    value: u8,
    _reserved2: [u32; 2],
    _padding: [u8; 480],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
union UpiuQueryRsp {
    read_desc: UpiuReadDescRsp,
    write_desc: UpiuWriteDescRsp,
    attr: UpiuAttrRsp,
    flag: UpiuFlagRsp,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UpiuNop {
    _reserved: [u32; 5],
    _padding: [u8; 480],
}

impl UpiuNop {
    fn build() -> Self {
        Self {
            _reserved: [0; 5],
            _padding: [0; 480],
        }
    }
}

// DATA OUT UPIU is automatically generated by the UTP Engine.
// It works without any involvement of software operation,
// so DATA OUT UPIU is not declared ans used in UFS Driver.
#[repr(C, packed)]
#[derive(Clone, Copy)]
union UpiuBody {
    cmd: UpiuCmd,
    rsp: UpiuRsp,
    tm_req: UpiuTmReq,
    tm_rsp: UpiuTmRsp,
    query_req: UpiuQueryReq,
    query_rsp: UpiuQueryRsp,
    nop_out: UpiuNop,
    nop_in: UpiuNop,
}

impl UpiuBody {
    fn nop_out() -> Self {
        Self { nop_out: UpiuNop::build() }
    }

    fn read_desc(cmd: UfsDescCmd) -> Self {
        Self { query_req: UpiuQueryReq::read_desc(cmd) }
    }

    fn read_attr(cmd: UfsAttrCmd) -> Self {
        Self { query_req: UpiuQueryReq::read_attr(cmd) }
    }

    fn write_attr(cmd: UfsAttrCmd) -> Self {
        Self { query_req: UpiuQueryReq::write_attr(cmd) }
    }

    fn read_flag(cmd: UfsFlagCmd) -> Self {
        Self { query_req: UpiuQueryReq::read_flag(cmd) }
    }

    fn set_flag(cmd: UfsFlagCmd) -> Self {
        Self { query_req: UpiuQueryReq::set_flag(cmd) }
    }

    fn clear_flag(cmd: UfsFlagCmd) -> Self {
        Self { query_req: UpiuQueryReq::clear_flag(cmd) }
    }

    fn toggle_flag(cmd: UfsFlagCmd) -> Self {
        Self { query_req: UpiuQueryReq::toggle_flag(cmd) }
    }

    fn command(cmd: UfsSCSICmd) -> Self {
        Self { cmd: UpiuCmd::command(cmd) }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct Upiu {
    header: UpiuHeader,
    body: UpiuBody,
}

impl Default for Upiu {
    fn default() -> Self {
        Self {
            header: UpiuHeader::default(),
            body: UpiuBody::nop_out()
        }
    }
}

impl Upiu {
    fn nop_out(tag: usize) -> Self {
        Self {
            header: UpiuHeader::nop_out(tag),
            body: UpiuBody::nop_out(),
        }
    }

    fn read_desc(cmd: UfsDescCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_read(tag),
            body: UpiuBody::read_desc(cmd),
        }
    }

    fn read_attr(cmd: UfsAttrCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_read(tag),
            body: UpiuBody::read_attr(cmd),
        }
    }

    fn write_attr(cmd: UfsAttrCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: UpiuBody::write_attr(cmd),
        }
    }

    fn read_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_read(tag),
            body: UpiuBody::read_flag(cmd),
        }
    }

    fn set_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: UpiuBody::set_flag(cmd),
        }
    }

    fn clear_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: UpiuBody::clear_flag(cmd),
        }
    }

    fn toggle_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: UpiuBody::toggle_flag(cmd),
        }
    }

    fn command(cmd: UfsSCSICmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::command(cmd, tag),
            body: UpiuBody::command(cmd),
        }
    }

    fn device(cmd: UfsDevCmd, tag: usize) -> Self {
        match cmd {
            UfsDevCmd::Nop => Self::nop_out(tag),
            UfsDevCmd::Query(cmd) => Self::query(cmd, tag),
            UfsDevCmd::RPMB(_) => Self::nop_out(tag),
        }
    }

    fn query(cmd: UfsQueryCmd, tag: usize) -> Self {
        match cmd {
            UfsQueryCmd::Nop => Self::nop_out(tag),
            UfsQueryCmd::ReadDesc(cmd) => Self::read_desc(cmd, tag),
            UfsQueryCmd::WriteDesc(cmd) => Self::nop_out(tag),
            UfsQueryCmd::ReadAttr(cmd) => Self::read_attr(cmd, tag),
            UfsQueryCmd::WriteAttr(cmd) => Self::write_attr(cmd, tag),
            UfsQueryCmd::ReadFlag(cmd) => Self::read_flag(cmd, tag),
            UfsQueryCmd::SetFlag(cmd) => Self::set_flag(cmd, tag),
            UfsQueryCmd::ClearFlag(cmd) => Self::clear_flag(cmd, tag),
            UfsQueryCmd::ToggleFlag(cmd) => Self::toggle_flag(cmd, tag),
        }
    }

    fn transaction(&self) -> UpiuTransaction {
        self.header.transaction_code.into()
    }

    fn response(&self) -> UpiuResponse {
        self.header.response.into()
    }

    fn scsi_result(&self, ocs: u8) -> UfsScsiResult {
        let upiu = self.body;
        let rsp = unsafe { upiu.rsp };
        let sense_data_len = usize::from(u16::from_be(rsp.sendse_data_len))
            .min(UFS_SENSE_SIZE);

        let mut result = UfsScsiResult {
            completion: UfsScsiCompletion::Error,
            ocs,
            transaction: self.header.transaction_code,
            response: self.header.response,
            status: self.header.status,
            residual_transfer_count: u32::from_be(rsp.residual_transfer_count),
            sense_data_len,
            sense_data: rsp.sense_data,
        };

        match self.transaction() {
            UpiuTransaction::Response => {},
            _ => return result,
        }

        result.completion = match self.header.status {
            SAM_STAT_GOOD => UfsScsiCompletion::Good,
            SAM_STAT_CHECK_CONDITION => UfsScsiCompletion::CheckCondition,
            SAM_STAT_BUSY => UfsScsiCompletion::Busy,
            SAM_STAT_RESERVATION_CONFLICT => UfsScsiCompletion::ReservationConflict,
            SAM_STAT_TASK_SET_FULL => UfsScsiCompletion::TaskSetFull,
            SAM_STAT_TASK_ABORTED => UfsScsiCompletion::TaskAborted,
            _ => UfsScsiCompletion::Error,
        };

        result
    }

    fn fetch_dev(&self, cmd: UfsDevCmd) -> Result<UfsDevCmd> {
        match cmd {
            UfsDevCmd::Nop => {
                match self.transaction() {
                    UpiuTransaction::NopIn => Ok(cmd),
                    _ => Err(EIO),
                }
            },
            UfsDevCmd::Query(cmd) => {
                match self.transaction() {
                    UpiuTransaction::QueryRsp => {
                        match self.response() {
                            UpiuResponse::Success => self.fetch_query(cmd),
                            _ => Err(EIO),
                        }
                    },
                    _ => Err(EIO),
                }
            },
            UfsDevCmd::RPMB(cmd) => {
                match self.transaction() {
                    UpiuTransaction::Response => Ok(UfsDevCmd::RPMB(cmd)),
                    _ => Err(EIO),
                }
            },
        }
    }

    fn fetch_query(&self, cmd: UfsQueryCmd) -> Result<UfsDevCmd> {
        match cmd {
            UfsQueryCmd::ReadDesc(cmd) => {
                let upiu = self.body;
                let upiu = unsafe { upiu.query_rsp.read_desc };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadDesc(UfsDescCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    length: u16::from_be(upiu.length),
                    desc: Desc::from_buffer(upiu.idn, upiu.buffer),
                })))
            },
            UfsQueryCmd::ReadAttr(cmd) => {
                let upiu = self.body;
                let upiu = unsafe { upiu.query_rsp.attr };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadAttr(UfsAttrCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: u64::from_be(upiu.value),
                })))
            },
            UfsQueryCmd::WriteAttr(cmd) => {
                let upiu = self.body;
                let upiu = unsafe { upiu.query_rsp.attr };
                Ok(UfsDevCmd::Query(UfsQueryCmd::WriteAttr(UfsAttrCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: u64::from_be(upiu.value),
                })))
            },
            UfsQueryCmd::ReadFlag(cmd) => {
                let upiu = self.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            UfsQueryCmd::SetFlag(cmd) => {
                let upiu = self.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::SetFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            UfsQueryCmd::ClearFlag(cmd) => {
                let upiu = self.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ClearFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            UfsQueryCmd::ToggleFlag(cmd) => {
                let upiu = self.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ToggleFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            _ => Ok(UfsDevCmd::Query(cmd)),
        }
    }
}

// UTP Request Descriptor Header
enum UtpCmdType {
    UfsStorage = 0x1,
}

enum UtpDataDirection {
    NoDataTransfer  = 0,
    HostToDevice    = 1,
    DeviceToHost    = 2,
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

// UTP Command Descriptor
#[repr(C, packed)]
#[derive(Default, Clone, Copy)]
struct PrdEntry {
    addr: u64,      // (LE)
    reserved: u32,  // (LE)
    size: u32,      // (LE)
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct Ucd {
    cmd_upiu: Upiu,
    rsp_upiu: Upiu,
    prdt: [PrdEntry; MAX_PRD_ENTRIES],
}

impl Ucd {
    fn build(&self, cmd: UfsCmd, tag: usize) -> Self {
        match cmd {
            UfsCmd::Device(cmd) => self.device(cmd, tag),
            UfsCmd::SCSI(cmd) => self.scsi(cmd, tag),
        }
    }

    fn device(&self, cmd: UfsDevCmd, tag: usize) -> Self {
        match cmd {
            UfsDevCmd::Nop => self.nop(tag),
            UfsDevCmd::Query(cmd) => self.query(cmd, tag),
            UfsDevCmd::RPMB(_) => self.nop(tag),
        }
    }

    fn nop(&self, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::nop_out(tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn query(&self, cmd: UfsQueryCmd, tag: usize) -> Self {
        match cmd {
            UfsQueryCmd::Nop => self.nop(tag),
            UfsQueryCmd::ReadDesc(cmd) => self.read_desc(cmd, tag),
            UfsQueryCmd::WriteDesc(cmd) => self.nop(tag),
            UfsQueryCmd::ReadAttr(cmd) => self.read_attr(cmd, tag),
            UfsQueryCmd::WriteAttr(cmd) => self.write_attr(cmd, tag),
            UfsQueryCmd::ReadFlag(cmd) => self.read_flag(cmd, tag),
            UfsQueryCmd::SetFlag(cmd) => self.set_flag(cmd, tag),
            UfsQueryCmd::ClearFlag(cmd) => self.clear_flag(cmd, tag),
            UfsQueryCmd::ToggleFlag(cmd) => self.toggle_flag(cmd, tag),
        }
    }

    fn read_desc(&self, cmd: UfsDescCmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::read_desc(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn read_attr(&self, cmd: UfsAttrCmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::read_attr(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn write_attr(&self, cmd: UfsAttrCmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::write_attr(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn read_flag(&self, cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::read_flag(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn set_flag(&self, cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::set_flag(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn clear_flag(&self, cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::clear_flag(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn toggle_flag(&self, cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::toggle_flag(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn scsi(&self, cmd: UfsSCSICmd, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::command(cmd, tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn fetch_dev(&self, cmd: UfsDevCmd) -> Result<UfsDevCmd> {
        match cmd {
            UfsDevCmd::Nop => {
                match self.rsp_upiu.transaction() {
                    UpiuTransaction::NopIn => Ok(cmd),
                    _ => Err(EIO),
                }
            },
            UfsDevCmd::Query(cmd) => {
                match self.rsp_upiu.transaction() {
                    UpiuTransaction::QueryRsp => {
                        match self.rsp_upiu.response() {
                            UpiuResponse::Success => self.fetch_query(cmd),
                            _ => Err(EIO),
                        }
                    },
                    _ => Err(EIO),
                }
            },
            UfsDevCmd::RPMB(cmd) => {
                match self.rsp_upiu.transaction() {
                    UpiuTransaction::Response => self.fetch_rpmb(cmd),
                    _ => Err(EIO),
                }
            },
        }
    }

    fn fetch_query(&self, cmd: UfsQueryCmd) -> Result<UfsDevCmd> {
        match cmd {
            UfsQueryCmd::ReadDesc(cmd) => {
                let upiu = self.rsp_upiu.body;
                let upiu = unsafe { upiu.query_rsp.read_desc };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadDesc(UfsDescCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    length: u16::from_be(upiu.length),
                    desc: Desc::from_buffer(upiu.idn, upiu.buffer),
                })))
            },
            UfsQueryCmd::ReadAttr(cmd) => {
                let upiu = self.rsp_upiu.body;
                let upiu = unsafe { upiu.query_rsp.attr };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadAttr(UfsAttrCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: u64::from_be(upiu.value),
                })))
            },
            UfsQueryCmd::WriteAttr(cmd) => {
                let upiu = self.rsp_upiu.body;
                let upiu = unsafe { upiu.query_rsp.attr };
                Ok(UfsDevCmd::Query(UfsQueryCmd::WriteAttr(UfsAttrCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: u64::from_be(upiu.value),
                })))
            },
            UfsQueryCmd::ReadFlag(cmd) => {
                let upiu = self.rsp_upiu.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            UfsQueryCmd::SetFlag(cmd) => {
                let upiu = self.rsp_upiu.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::SetFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            UfsQueryCmd::ClearFlag(cmd) => {
                let upiu = self.rsp_upiu.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ClearFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            UfsQueryCmd::ToggleFlag(cmd) => {
                let upiu = self.rsp_upiu.body;
                let upiu = unsafe { upiu.query_rsp.flag };
                Ok(UfsDevCmd::Query(UfsQueryCmd::ToggleFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            },
            _ => Ok(UfsDevCmd::Query(cmd)),
        }
    }

    fn fetch_rpmb(&self, cmd: UfsRPMBCmd) -> Result<UfsDevCmd> {
        Ok(UfsDevCmd::RPMB(cmd))
    }
}

enum UtpOcs {
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
    GeneralCryptoError = 0xA,
    InvalidCommandStatus = 0xF,
}

impl From<u8> for UtpOcs {
    fn from(ocs: u8) -> Self {
        match ocs {
            0x0 => UtpOcs::Success,
            0x1 => UtpOcs::InvalidCmdTableAttr,
            0x2 => UtpOcs::InvalidPrdtAttr,
            0x3 => UtpOcs::MismatchDataBufSize,
            0x4 => UtpOcs::MisMatchRespUpiuSize,
            0x5 => UtpOcs::PeerCommFailure,
            0x6 => UtpOcs::Aborted,
            0x7 => UtpOcs::FatalError,
            0x8 => UtpOcs::DeviceFatalError,
            0x9 => UtpOcs::InvalidCryptoConfig,
            0xA => UtpOcs::GeneralCryptoError,
            _ => UtpOcs::InvalidCommandStatus,
        }
    }
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy)]
struct ReqDescHeader {
    cci: u8,        // 0x00
    ehs_length: u8, // 0x01
    flags: u8,      // 0x02 (bit0: enable_crypto)
    ctrl: u8,       // 0x03 (bit0: interrupt, bit[2:1] dir, bit[7:4]: cmd_type)
    dunl: u32,      // 0x04 (LE)
    ocs: u8,        // 0x08
    cds: u8,        // 0x09
    ldbc: u16,      // 0x0A (LE)
    dunu: u32,      // 0x0C (LE)
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

    fn device(cmd: UfsDevCmd) -> Self {
        let mut header = ReqDescHeader::default();
        header.set_cmd_type(UtpCmdType::UfsStorage);
        header.set_direction(UtpDataDirection::NoDataTransfer);
        header.set_interrupt(true);
        header.ehs_length = 0;
        header.ocs = UtpOcs::InvalidCommandStatus as u8;
        header
    }

    fn scsi(cmd: UfsSCSICmd) -> Self {
        let mut header = ReqDescHeader::default();
        header.set_cmd_type(UtpCmdType::UfsStorage);
        header.set_direction(cmd.direction().into());
        header.set_interrupt(true);
        header.ocs = UtpOcs::InvalidCommandStatus as u8;
        header
    }
}

#[repr(C, packed)]
#[derive(Default)]
pub(crate) struct Utrd {
    header: ReqDescHeader,
    command_desc_base_addr: u64,
    rsp_upiu_length: u16,
    rsp_upiu_offset: u16,
    prd_table_length: u16,
    prd_table_offset: u16,
}

// UFSHCI MCQ uses the UTP Transfer Request Descriptor as each Submission Queue
// Entry. Keep this alias explicit so MCQ code can talk in SQE/CQE terms while
// sharing the descriptor layout with the SDB path.
pub(crate) type SqEntry = Utrd;

#[repr(C, packed)]
#[derive(Default, Clone, Copy)]
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
        let ptr = core::ptr::addr_of!(self.command_desc_base_addr);

        // SAFETY: `CqEntry` is packed, so integer fields may be unaligned.
        unsafe { u64::from_le(core::ptr::read_unaligned(ptr)) }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.command_desc_base_addr() == 0
    }

    pub(crate) fn task_tag(&self) -> u8 {
        self.task_tag
    }

    pub(crate) fn lun(&self) -> u8 {
        self.lun
    }

    pub(crate) fn overall_status(&self) -> u8 {
        self.overall_status
    }

    pub(crate) fn extended_error_code(&self) -> u8 {
        self.extended_error_code
    }

    pub(crate) fn iid(&self) -> u8 {
        self.iid_ext_iid & 0x0f
    }

    pub(crate) fn ext_iid(&self) -> u8 {
        self.iid_ext_iid >> 4
    }

    pub(crate) fn clear(&mut self) {
        self.command_desc_base_addr = 0;
    }
}

impl Utrd {
    fn build(&self, cmd: UfsCmd) -> Self {
        match cmd {
            UfsCmd::Device(cmd) => self.device(cmd),
            UfsCmd::SCSI(cmd) => self.scsi(cmd),
        }
    }

    fn device(&self, cmd: UfsDevCmd) -> Self {
        let header = ReqDescHeader::device(cmd);
        Self { header, ..*self }
    }

    fn scsi(&self, cmd: UfsSCSICmd) -> Self {
        let header = ReqDescHeader::scsi(cmd);
        Self { header, ..*self }
    }

    fn check_response(&self) -> Result<()> {
        match (self.header.ocs & MASK_OCS).into() {
            UtpOcs::Success => Ok(()),
            UtpOcs::InvalidCmdTableAttr => Err(EINVAL),
            UtpOcs::InvalidPrdtAttr => Err(EINVAL),
            UtpOcs::MismatchDataBufSize => Err(EINVAL),
            UtpOcs::MisMatchRespUpiuSize => Err(EINVAL),
            UtpOcs::InvalidCryptoConfig => Err(EINVAL),
            UtpOcs::GeneralCryptoError => Err(EINVAL),
            _ => Err(EIO),
        }
    }

    fn ocs(&self) -> u8 {
        self.header.ocs & MASK_OCS
    }
}

// UTP Task Management Request Descriptor
#[repr(C, packed)]
struct Utmrd {
    header: ReqDescHeader,
    upiu_req: UpiuTmReq,
    upiu_rsp: UpiuTmRsp,
}

struct UfsDmaInner {
    ucdl: dma::Coherent<[Ucd]>,
    utrdl: dma::Coherent<[Utrd]>,
    utmrdl: dma::Coherent<[Utmrd]>,
}

pub(crate) struct UfsMcqQueue {
    id: u32,
    max_entries: u32,
    sqe: dma::Coherent<[SqEntry]>,
    cqe: dma::Coherent<[CqEntry]>,
    sq_tail_slot: u32,
    cq_tail_slot: u32,
    cq_head_slot: u32,
    oprs: UfsMcqOprSet,
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
            id,
            max_entries,
            sqe: dma::Coherent::<SqEntry>::zeroed_slice(dev, entries, GFP_KERNEL)?,
            cqe: dma::Coherent::<CqEntry>::zeroed_slice(dev, entries, GFP_KERNEL)?,
            sq_tail_slot: 0,
            cq_tail_slot: 0,
            cq_head_slot: 0,
            oprs,
        })
    }

    pub(crate) fn id(&self) -> u32 {
        self.id
    }

    pub(crate) fn max_entries(&self) -> u32 {
        self.max_entries
    }

    pub(crate) fn sqe_dma_addr(&self) -> dma::DmaAddress {
        self.sqe.dma_handle()
    }

    pub(crate) fn cqe_dma_addr(&self) -> dma::DmaAddress {
        self.cqe.dma_handle()
    }

    pub(crate) fn sq_tail_slot(&self) -> u32 {
        self.sq_tail_slot
    }

    pub(crate) fn sq_tail_index(&self) -> Result<usize> {
        let index = self.sq_tail_slot as usize;
        if index >= self.max_entries as usize {
            return Err(EINVAL);
        }

        Ok(index)
    }

    fn sq_slot_offset(slot: u32) -> u32 {
        slot * core::mem::size_of::<SqEntry>() as u32
    }

    fn cq_slot_offset(slot: u32) -> u32 {
        slot * core::mem::size_of::<CqEntry>() as u32
    }

    fn offset_to_slot(offset: u32, entry_size: u32, max_entries: u32) -> Result<u32> {
        if offset % entry_size != 0 {
            return Err(EINVAL);
        }

        let slot = offset / entry_size;
        if slot >= max_entries {
            return Err(EINVAL);
        }

        Ok(slot)
    }

    fn sq_offset_to_slot(&self, offset: u32) -> Result<u32> {
        Self::offset_to_slot(
            offset,
            core::mem::size_of::<SqEntry>() as u32,
            self.max_entries,
        )
    }

    fn cq_offset_to_slot(&self, offset: u32) -> Result<u32> {
        Self::offset_to_slot(
            offset,
            core::mem::size_of::<CqEntry>() as u32,
            self.max_entries,
        )
    }

    fn next_sq_tail_slot(&self) -> u32 {
        let next = self.sq_tail_slot + 1;
        if next == self.max_entries {
            0
        } else {
            next
        }
    }

    pub(crate) fn sq_is_full(&self, reg: &UfsReg) -> Result<bool> {
        let head = self.sq_offset_to_slot(reg.read_mcq_sq_head(&self.oprs, self.id as usize)?)?;
        Ok(self.next_sq_tail_slot() == head)
    }

    pub(crate) fn cq_tail_slot(&self) -> u32 {
        self.cq_tail_slot
    }

    pub(crate) fn cq_head_slot(&self) -> u32 {
        self.cq_head_slot
    }

    pub(crate) fn update_cq_tail_slot(&mut self, reg: &UfsReg) -> Result<()> {
        self.cq_tail_slot =
            self.cq_offset_to_slot(reg.read_mcq_cq_tail(&self.oprs, self.id as usize)?)?;
        Ok(())
    }

    pub(crate) fn cq_is_empty(&self) -> bool {
        self.cq_head_slot == self.cq_tail_slot
    }

    pub(crate) fn acknowledge_cq_events(&self, reg: &UfsReg) -> Result<()> {
        let status = reg.read_mcq_cqis(&self.oprs, self.id as usize)?;
        if status != 0 {
            reg.write_mcq_cqis(&self.oprs, self.id as usize, status)?;
        }

        Ok(())
    }

    pub(crate) fn oprs(&self) -> &UfsMcqOprSet {
        &self.oprs
    }

    pub(crate) fn reset_slots(&mut self) {
        self.sq_tail_slot = 0;
        self.cq_tail_slot = 0;
        self.cq_head_slot = 0;
    }

    pub(crate) fn write_sq_entry(&mut self, entry: SqEntry) -> Result<u32> {
        let index = self.sq_tail_index()?;
        dma_write!(self.sqe, [index]?, entry);

        self.sq_tail_slot = self.next_sq_tail_slot();

        Ok(Self::sq_slot_offset(self.sq_tail_slot))
    }

    pub(crate) fn consume_cq_entry(&mut self, reg: &UfsReg) -> Result<Option<CqEntry>> {
        let index = self.cq_head_slot as usize;
        if index >= self.max_entries as usize {
            return Err(EINVAL);
        }

        let cqe = dma_read!(self.cqe, [index]?);
        dma_write!(self.cqe, [index]?, CqEntry::default());

        self.cq_head_slot += 1;
        if self.cq_head_slot == self.max_entries {
            self.cq_head_slot = 0;
        }

        reg.write_mcq_cq_head(
            &self.oprs,
            self.id as usize,
            Self::cq_slot_offset(self.cq_head_slot),
        )?;

        if cqe.is_empty() {
            Ok(None)
        } else {
            Ok(Some(cqe))
        }
    }
}

#[pin_data]
pub(crate) struct UfsDma {
    reg: Arc<UfsReg>,
    dev: ARef<device::Device>,
    transfer_slots: usize,

    #[pin]
    inner: SpinLock<UfsDmaInner>,
}

// SAFETY: UfsDma itself doesn't have any thread-affinity
unsafe impl Send for UfsDma {}

pub(crate) enum UfsPrdtMapping {
    Sg(UfsSgMapping),
    Unmap(UfsUnmapMapping),
}

pub(crate) struct UfsSgMapping {
    dev: ARef<device::Device>,
    sg: KVec<bindings::scatterlist>,
    nents: i32,
    dma_dir: bindings::dma_data_direction,
}

pub(crate) struct UfsUnmapMapping {
    dev: ARef<device::Device>,
    cpu_addr: core::ptr::NonNull<u8>,
    dma_addr: dma::DmaAddress,
}

// SAFETY: `UfsUnmapMapping` owns a DMA allocation associated with a refcounted
// device and only frees it on drop. Moving the owner between threads does not
// expose shared mutable access to the allocation.
unsafe impl Send for UfsUnmapMapping {}

struct UfsPrdt {
    mapping: Option<UfsPrdtMapping>,
    entries: KVec<PrdEntry>,
}

impl Drop for UfsSgMapping {
    fn drop(&mut self) {
        // SAFETY: `sg` was mapped by `dma_map_sg_attrs` with this device,
        // entry count, direction, and attributes.
        unsafe {
            bindings::dma_unmap_sg_attrs(
                self.dev.as_raw(),
                self.sg.as_mut_ptr(),
                self.nents,
                self.dma_dir,
                0,
            )
        };
    }
}

impl Drop for UfsUnmapMapping {
    fn drop(&mut self) {
        // SAFETY: `cpu_addr` and `dma_addr` were returned by `dma_alloc_attrs`
        // for this device and size, and this object owns that allocation.
        unsafe {
            bindings::dma_free_attrs(
                self.dev.as_raw(),
                UNMAP_PARAM_LIST_SIZE,
                self.cpu_addr.as_ptr().cast(),
                self.dma_addr,
                0,
            )
        };
    }
}

impl UfsDma {
    fn transfer_slots_for(reg: &UfsReg) -> usize {
        if reg.mcq_supported() {
            core::cmp::max(reg.nutrs(), reg.nutrs_mcq())
        } else {
            reg.nutrs()
        }
    }

    pub(crate) fn dev(&self) -> &device::Device<Bound> {
        // SAFETY: `UfsDma` is owned by the bound RUFS driver instance. MCQ queue
        // allocations only use this reference while the driver owns the device.
        unsafe { self.dev.as_bound() }
    }

    pub(crate) fn new(
        pdev: &pci::Device<Core>,
        reg: Arc<UfsReg>,
    ) -> Result<Arc<Self>> {
        let transfer_slots = Self::transfer_slots_for(&reg);
        let ucdl = dma::Coherent::<Ucd>::zeroed_slice(
            pdev.as_ref(), transfer_slots, GFP_KERNEL,
        )?;

        let utrdl = dma::Coherent::<Utrd>::zeroed_slice(
            pdev.as_ref(), transfer_slots, GFP_KERNEL,
        )?;

        for tag in 0..transfer_slots {
            let rsp_upiu_length = ((ALIGNED_UPIU_SIZE >> 2) as u16).to_le();
            let rsp_upiu_offset = ((ALIGNED_UPIU_SIZE >> 2) as u16).to_le();
            let prd_table_offset = ((ALIGNED_UPIU_SIZE >> 1) as u16).to_le();

            // The controller DMA-reads the UTP command descriptor for this tag,
            // so this must be the descriptor's DMA (bus) address, not its CPU
            // virtual address. `ucdl` is a contiguous slice, so element `tag`
            // sits at `tag * size_of::<Ucd>()` bytes from the DMA base.
            // TODO: use `io_project` when available https://lore.kernel.org/r/20260611-io_projection-v4-0-1f7224b02dcb@garyguo.net
            let command_desc_base_addr =
                ucdl.dma_handle() + (tag * core::mem::size_of::<Ucd>()) as dma::DmaAddress;

            dma_write!(utrdl, [tag]?, Utrd {
                    command_desc_base_addr: command_desc_base_addr.to_le(),
                    rsp_upiu_length,
                    rsp_upiu_offset,
                    prd_table_offset,
                    ..dma_read!(utrdl, [tag]?)
            });
        }

        let nutmrs = reg.nutmrs();
        let utmrdl = dma::Coherent::<Utmrd>::zeroed_slice(
            pdev.as_ref(), nutmrs, GFP_KERNEL,
        )?;

        Arc::pin_init(
            pin_init!(Self {
                reg,
                dev: pdev.as_ref().into(),
                transfer_slots,
                inner <- new_spinlock!(UfsDmaInner {
                    ucdl,
                    utrdl,
                    utmrdl,
                }),
            }),
            GFP_KERNEL
        )
    }

    pub(crate) fn transfer_slots(&self) -> usize {
        self.transfer_slots
    }

    pub(crate) fn make_hba_operational(&self) -> Result<()> {
        self.reg.enable_interrupts();

        self.reg.set_utrdl_base(self.inner.lock().utrdl.dma_handle() as u64);
        self.reg.set_utmrdl_base(self.inner.lock().utmrdl.dma_handle() as u64);

        self.reg.wait_for_request_ready(1000, 50)?;
        self.reg.enable_run_stop();

        Ok(())
    }

    pub(crate) fn compose_devman_upiu(
        &self,
        cmd: UfsDevCmd,
        tag: usize,
    ) -> Result<()> {
        let inner = self.inner.lock();

        dma_write!(inner.ucdl, [tag]?.cmd_upiu, Upiu::device(cmd, tag));
        dma_write!(inner.ucdl, [tag]?.rsp_upiu, Upiu::default());

        let utrd = dma_read!(inner.utrdl, [tag]?);
        dma_write!(inner.utrdl, [tag]?, utrd.build(UfsCmd::Device(cmd)));
        Ok(())
    }

    pub(crate) fn compose_scsi_upiu(
        &self,
        cmd: UfsSCSICmd,
        tag: usize,
        rq: *mut bindings::request,
    ) -> Result<Option<UfsPrdtMapping>> {
        let prdt = self.map_request_prdt(tag, cmd, rq)?;
        let inner = self.inner.lock();

        dma_write!(inner.ucdl, [tag]?.cmd_upiu, Upiu::command(cmd, tag));
        dma_write!(inner.ucdl, [tag]?.rsp_upiu, Upiu::default());

        for (i, entry) in prdt.entries.iter().enumerate() {
            // SAFETY: `tag` is checked against the UCD allocation, `i` is
            // bounded by MAX_PRD_ENTRIES in `map_request_prdt`, and PRDT lives
            // in a packed UCD so unaligned writes are required.
            unsafe {
                let ucd = inner.ucdl.as_mut_ptr().cast::<Ucd>().add(tag);
                let table = core::ptr::addr_of_mut!((*ucd).prdt).cast::<PrdEntry>();
                core::ptr::write_unaligned(table.add(i), *entry);
            }
        }

        let utrd = dma_read!(inner.utrdl, [tag]?);
        let mut utrd = utrd.build(UfsCmd::SCSI(cmd));
        utrd.prd_table_length = (prdt.entries.len() as u16).to_le();
        dma_write!(inner.utrdl, [tag]?, utrd);

        Ok(prdt.mapping)
    }

    pub(crate) fn transfer_request_desc(&self, tag: usize) -> Result<Utrd> {
        let inner = self.inner.lock();
        Ok(dma_read!(inner.utrdl, [tag]?))
    }

    pub(crate) fn tag_from_cq_entry(&self, cqe: &CqEntry) -> Result<usize> {
        const CQE_UCD_BA_MASK: u64 = !0x7f;

        let inner = self.inner.lock();
        let base = inner.ucdl.dma_handle() as u64;
        let addr = cqe.command_desc_base_addr() & CQE_UCD_BA_MASK;
        let size = core::mem::size_of::<Ucd>() as u64;

        if addr < base {
            return Err(EINVAL);
        }

        let offset = addr - base;
        if size == 0 || offset % size != 0 {
            return Err(EINVAL);
        }

        let tag = (offset / size) as usize;
        if tag >= self.transfer_slots {
            return Err(EINVAL);
        }

        Ok(tag)
    }

    fn map_request_prdt(
        &self,
        tag: usize,
        cmd: UfsSCSICmd,
        rq: *mut bindings::request,
    ) -> Result<UfsPrdt> {
        let entries = KVec::new();
        if cmd.data_len() == 0 {
            return Ok(UfsPrdt {
                mapping: None,
                entries,
            });
        }

        if tag >= self.transfer_slots {
            return Err(EINVAL);
        }

        if cmd.is_unmap() {
            return self.map_unmap_prdt(cmd);
        }

        if rq.is_null() {
            return Err(EINVAL);
        }

        // SAFETY: `rq` is a live blk-mq request owned by the queue_rq callback.
        let nr_segments = unsafe { (*rq).nr_phys_segments as usize };
        if nr_segments == 0 || nr_segments > MAX_PRD_ENTRIES {
            return Err(EINVAL);
        }

        let mut sg = KVec::with_capacity(nr_segments, GFP_KERNEL)?;
        for _ in 0..nr_segments {
            sg.push(bindings::scatterlist::default(), GFP_KERNEL)?;
        }

        // SAFETY: `sg` has `nr_segments` initialized entries.
        unsafe { bindings::sg_init_table(sg.as_mut_ptr(), nr_segments as u32) };

        let mut last_sg: *mut bindings::scatterlist = core::ptr::null_mut();

        // SAFETY: `rq` is valid and `sg` points to a scatterlist table with enough entries.
        let nents = unsafe { bindings::__blk_rq_map_sg(rq, sg.as_mut_ptr(), &mut last_sg) };
        if nents <= 0 {
            return Err(EIO);
        }

        let dma_dir = match cmd.direction() {
            UfsScsiDataDirection::Read => bindings::dma_data_direction_DMA_FROM_DEVICE,
            UfsScsiDataDirection::Write => bindings::dma_data_direction_DMA_TO_DEVICE,
            UfsScsiDataDirection::None => bindings::dma_data_direction_DMA_NONE,
        };

        // SAFETY: `self.dev` is a valid DMA device and the scatterlist was populated above.
        let mapped = unsafe {
            bindings::dma_map_sg_attrs(
                self.dev.as_raw(),
                sg.as_mut_ptr(),
                nents,
                dma_dir,
                0,
            )
        };
        if mapped <= 0 {
            return Err(ENOMEM);
        }

        let mut mapping = UfsSgMapping {
            dev: self.dev.clone(),
            sg,
            nents,
            dma_dir,
        };

        let mut entries = KVec::with_capacity(mapped as usize, GFP_KERNEL)?;
        let mut sgp = mapping.sg.as_mut_ptr();
        for _ in 0..mapped as usize {
            if sgp.is_null() {
                return Err(EIO);
            }

            // SAFETY: `sgp` is a valid mapped scatterlist entry.
            let addr = unsafe { bindings::sg_dma_address(sgp) };
            // SAFETY: `sgp` is a valid mapped scatterlist entry.
            let len = unsafe { bindings::sg_dma_len(sgp) };
            if len == 0 || len > PRDT_DATA_BYTE_COUNT_MAX {
                return Err(EINVAL);
            }

            let entry = PrdEntry {
                addr: addr.to_le(),
                reserved: 0,
                size: (len - 1).to_le(),
            };

            entries.push(entry, GFP_KERNEL)?;

            // SAFETY: `sgp` is a valid scatterlist entry.
            sgp = unsafe { bindings::sg_next(sgp) };
        }

        Ok(UfsPrdt {
            mapping: Some(UfsPrdtMapping::Sg(mapping)),
            entries,
        })
    }

    fn map_unmap_prdt(&self, cmd: UfsSCSICmd) -> Result<UfsPrdt> {
        if cmd.unmap_blocks() == 0 {
            return Err(EINVAL);
        }

        let mut data = [0u8; UNMAP_PARAM_LIST_SIZE];
        let mut dma_addr = 0;

        data[0..2].copy_from_slice(&22u16.to_be_bytes());
        data[2..4].copy_from_slice(&16u16.to_be_bytes());
        data[8..16].copy_from_slice(&cmd.unmap_lba().to_be_bytes());
        data[16..20].copy_from_slice(&cmd.unmap_blocks().to_be_bytes());

        // SAFETY: `self.dev` is a valid DMA device. The returned allocation is
        // owned by `UfsUnmapMapping` and freed in its `Drop` implementation.
        let cpu_addr = unsafe {
            bindings::dma_alloc_attrs(
                self.dev.as_raw(),
                UNMAP_PARAM_LIST_SIZE,
                &mut dma_addr,
                bindings::GFP_KERNEL,
                0,
            )
        };
        let cpu_addr = core::ptr::NonNull::new(cpu_addr.cast::<u8>()).ok_or(ENOMEM)?;

        // SAFETY: `cpu_addr` points to a DMA allocation of
        // `UNMAP_PARAM_LIST_SIZE` bytes.
        unsafe {
            core::ptr::copy_nonoverlapping(
                data.as_ptr(),
                cpu_addr.as_ptr(),
                UNMAP_PARAM_LIST_SIZE,
            )
        };

        let mapping = UfsUnmapMapping {
            dev: self.dev.clone(),
            cpu_addr,
            dma_addr,
        };

        let mut entries = KVec::with_capacity(1, GFP_KERNEL)?;
        entries.push(
            PrdEntry {
                addr: mapping.dma_addr.to_le(),
                reserved: 0,
                size: ((UNMAP_PARAM_LIST_SIZE as u32) - 1).to_le(),
            },
            GFP_KERNEL,
        )?;

        Ok(UfsPrdt {
            mapping: Some(UfsPrdtMapping::Unmap(mapping)),
            entries,
        })
    }

    pub(crate) fn fetch_devman_upiu(
        &self,
        cmd: UfsDevCmd,
        tag: usize,
    ) -> Result<UfsCmd> {
        let inner = self.inner.lock();

        let utrd = dma_read!(inner.utrdl, [tag]?);
        utrd.check_response()?;

        let rsp_upiu = dma_read!(inner.ucdl, [tag]?.rsp_upiu);
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
            UtpOcs::Success => {},
            UtpOcs::InvalidCmdTableAttr => return Err(EINVAL),
            UtpOcs::InvalidPrdtAttr => return Err(EINVAL),
            UtpOcs::MismatchDataBufSize => return Err(EINVAL),
            UtpOcs::MisMatchRespUpiuSize => return Err(EINVAL),
            UtpOcs::InvalidCryptoConfig => return Err(EINVAL),
            UtpOcs::GeneralCryptoError => return Err(EINVAL),
            _ => return Err(EIO),
        }

        let inner = self.inner.lock();
        let rsp_upiu = dma_read!(inner.ucdl, [tag]?.rsp_upiu);
        let cmd = rsp_upiu.fetch_dev(cmd)?;

        Ok(UfsCmd::Device(cmd))
    }

    pub(crate) fn fetch_scsi_completion(
        &self,
        tag: usize,
    ) -> UfsScsiResult {
        let inner = self.inner.lock();

        let utrd = match (|| -> Result<_> { Ok(dma_read!(inner.utrdl, [tag]?)) })() {
            Ok(utrd) => utrd,
            Err(_) => return UfsScsiResult::error(UtpOcs::InvalidCommandStatus as u8),
        };
        let ocs = utrd.ocs();

        if utrd.check_response().is_err() {
            return match (utrd.header.ocs & MASK_OCS).into() {
                UtpOcs::Aborted | UtpOcs::InvalidCommandStatus => UfsScsiResult::requeue(ocs),
                _ => UfsScsiResult::error(ocs),
            };
        }

        match (|| -> Result<_> { Ok(dma_read!(inner.ucdl, [tag]?.rsp_upiu)) })() {
            Ok(rsp_upiu) => rsp_upiu.scsi_result(ocs),
            Err(_) => UfsScsiResult::error(ocs),
        }
    }

    pub(crate) fn fetch_mcq_scsi_completion(
        &self,
        tag: usize,
        cqe: CqEntry,
    ) -> UfsScsiResult {
        let ocs = cqe.overall_status();

        if !matches!(ocs.into(), UtpOcs::Success) {
            return match ocs.into() {
                UtpOcs::Aborted | UtpOcs::InvalidCommandStatus => UfsScsiResult::requeue(ocs),
                _ => UfsScsiResult::error(ocs),
            };
        }

        let inner = self.inner.lock();
        match (|| -> Result<_> { Ok(dma_read!(inner.ucdl, [tag]?.rsp_upiu)) })() {
            Ok(rsp_upiu) => rsp_upiu.scsi_result(ocs),
            Err(_) => UfsScsiResult::error(UtpOcs::InvalidCommandStatus as u8),
        }
    }
}

const _: () = { assert!(size_of::<UpiuHeader>() == 12); };
const _: () = { assert!(size_of::<UpiuCmd>() == 500); };
const _: () = { assert!(size_of::<UpiuRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuTmReq>() == 32); };
const _: () = { assert!(size_of::<UpiuTmRsp>() == 32); };
const _: () = { assert!(size_of::<DescBuffer>() == QUERY_DESC_MAX_SIZE); };
const _: () = { assert!(size_of::<UpiuReadDescReq>() == 500); };
const _: () = { assert!(size_of::<UpiuWriteDescReq>() == 500); };
const _: () = { assert!(size_of::<UpiuReadAttrReq>() == 500); };
const _: () = { assert!(size_of::<UpiuWriteAttrReq>() == 500); };
const _: () = { assert!(size_of::<UpiuFlagReq>() == 500); };
const _: () = { assert!(size_of::<UpiuQueryReq>() == 500); };
const _: () = { assert!(size_of::<UpiuReadDescRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuWriteDescRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuAttrRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuFlagRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuQueryRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuNop>() == 500); };
const _: () = { assert!(size_of::<UpiuBody>() == 500); };
const _: () = { assert!(size_of::<Upiu>() == 512); };
const _: () = { assert!(size_of::<ReqDescHeader>() == 16); };
const _: () = { assert!(size_of::<PrdEntry>() == 16); };
const _: () = { assert!(size_of::<CqEntry>() == 32); };
const _: () = { assert!(size_of::<Ucd>() == 5120); };
const _: () = { assert!(size_of::<Utrd>() == 32); };
const _: () = { assert!(size_of::<Utmrd>() == 80); };

unsafe impl kernel::transmute::AsBytes for Ucd {}
unsafe impl kernel::transmute::FromBytes for Ucd {}
unsafe impl kernel::transmute::AsBytes for Upiu {}
unsafe impl kernel::transmute::FromBytes for Upiu {}
unsafe impl kernel::transmute::AsBytes for Utrd {}
unsafe impl kernel::transmute::FromBytes for Utrd {}
unsafe impl kernel::transmute::AsBytes for Utmrd {}
unsafe impl kernel::transmute::FromBytes for Utmrd {}
unsafe impl kernel::transmute::AsBytes for CqEntry {}
unsafe impl kernel::transmute::FromBytes for CqEntry {}
