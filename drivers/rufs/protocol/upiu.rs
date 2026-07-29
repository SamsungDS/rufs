// SPDX-License-Identifier: GPL-2.0

//! UFS Protocol Information Unit definitions.

#![allow(dead_code)]
#![allow(unused_variables)]

use super::{query::*, scsi::*};
use kernel::prelude::*;
use zerocopy_derive::{Immutable, KnownLayout};

const UFS_CDB_SIZE: usize = 16;
const SAM_STAT_GOOD: u8 = 0x00;
const SAM_STAT_CHECK_CONDITION: u8 = 0x02;
const SAM_STAT_BUSY: u8 = 0x08;
const SAM_STAT_RESERVATION_CONFLICT: u8 = 0x18;
const SAM_STAT_TASK_SET_FULL: u8 = 0x28;
const SAM_STAT_TASK_ABORTED: u8 = 0x40;

// UPIU
enum UpiuFlag {
    None = 0x00,
    CP = 0x04,
    Write = 0x20,
    Read = 0x40,
}

enum UpiuTransaction {
    NopOut = 0x00,
    Command = 0x01,
    DataOut = 0x02,
    TaskReq = 0x04,
    QueryReq = 0x16,
    NopIn = 0x20,
    Response = 0x21,
    DataIn = 0x22,
    TaskRsp = 0x24,
    ReadyXfer = 0x31,
    QueryRsp = 0x36,
    Reject = 0x3F,
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
    StandardRead = 0x01,
    StandardWrite = 0x81,
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
#[derive(Default, Clone, Copy, FromBytes, IntoBytes, Immutable)]
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
    pub(crate) fn nop_out(tag: usize) -> Self {
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

    pub(crate) fn command(cmd: UfsSCSICmd, tag: usize) -> Self {
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
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable, KnownLayout)]
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
#[derive(Default, Clone, Copy, FromBytes, IntoBytes, Immutable)]
pub(crate) struct UpiuTmReq {
    header: UpiuHeader,
    input_param1: u32, // (BE)
    input_param2: u32, // (BE)
    input_param3: u32, // (BE)
    reserved: [u32; 2],
    // Task Management doesn't have padding
    // because it is pre-allocated in UTMRD DMA aree directly
}

#[repr(C, packed)]
#[derive(Default, Clone, Copy, FromBytes, IntoBytes, Immutable)]
pub(crate) struct UpiuTmRsp {
    header: UpiuHeader,
    output_param1: u32, // (BE)
    output_param2: u32, // (BE)
    reserved: [u32; 3],
    // Task Management doesn't have padding
    // because it is pre-allocated in UTMRD DMA aree directly
}

enum QueryOpcode {
    Nop = 0x0,
    ReadDesc = 0x1,
    WriteDesc = 0x2,
    ReadAttr = 0x3,
    WriteAttr = 0x4,
    ReadFlag = 0x5,
    SetFlag = 0x6,
    ClearFlag = 0x7,
    ToggleFlag = 0x8,
}

#[repr(C, packed)]
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable)]
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
    pub(crate) fn read_desc(cmd: UfsDescCmd) -> Self {
        Self {
            read_desc: UpiuReadDescReq::build(cmd),
        }
    }

    pub(crate) fn read_attr(cmd: UfsAttrCmd) -> Self {
        Self {
            read_attr: UpiuReadAttrReq::build(cmd),
        }
    }

    pub(crate) fn write_attr(cmd: UfsAttrCmd) -> Self {
        Self {
            write_attr: UpiuWriteAttrReq::build(cmd),
        }
    }

    pub(crate) fn read_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            read_flag: UpiuFlagReq::build(cmd, QueryOpcode::ReadFlag),
        }
    }

    pub(crate) fn set_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            set_flag: UpiuFlagReq::build(cmd, QueryOpcode::SetFlag),
        }
    }

    pub(crate) fn clear_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            clear_flag: UpiuFlagReq::build(cmd, QueryOpcode::ClearFlag),
        }
    }

    pub(crate) fn toggle_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            toggle_flag: UpiuFlagReq::build(cmd, QueryOpcode::ToggleFlag),
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, FromBytes, IntoBytes)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes)]
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
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable)]
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
        Self {
            nop_out: UpiuNop::build(),
        }
    }

    fn read_desc(cmd: UfsDescCmd) -> Self {
        Self {
            query_req: UpiuQueryReq::read_desc(cmd),
        }
    }

    fn read_attr(cmd: UfsAttrCmd) -> Self {
        Self {
            query_req: UpiuQueryReq::read_attr(cmd),
        }
    }

    fn write_attr(cmd: UfsAttrCmd) -> Self {
        Self {
            query_req: UpiuQueryReq::write_attr(cmd),
        }
    }

    fn read_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            query_req: UpiuQueryReq::read_flag(cmd),
        }
    }

    fn set_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            query_req: UpiuQueryReq::set_flag(cmd),
        }
    }

    fn clear_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            query_req: UpiuQueryReq::clear_flag(cmd),
        }
    }

    fn toggle_flag(cmd: UfsFlagCmd) -> Self {
        Self {
            query_req: UpiuQueryReq::toggle_flag(cmd),
        }
    }

    fn command(cmd: UfsSCSICmd) -> Self {
        Self {
            cmd: UpiuCmd::command(cmd),
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, FromBytes, IntoBytes)]
pub(crate) struct Upiu {
    header: UpiuHeader,
    body: [u8; 500],
}

impl Default for Upiu {
    fn default() -> Self {
        Self {
            header: UpiuHeader::default(),
            body: <[u8; 500]>::try_from(UpiuNop::build().as_bytes()).unwrap(),
        }
    }
}

impl Upiu {
    pub(crate) fn nop_out(tag: usize) -> Self {
        Self {
            header: UpiuHeader::nop_out(tag),
            body: <[u8; 500]>::try_from(UpiuNop::build().as_bytes()).unwrap(),
        }
    }

    pub(crate) fn read_desc(cmd: UfsDescCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_read(tag),
            body: <[u8; 500]>::try_from(UpiuReadDescReq::build(cmd).as_bytes()).unwrap(),
        }
    }

    pub(crate) fn read_attr(cmd: UfsAttrCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_read(tag),
            body: <[u8; 500]>::try_from(UpiuReadAttrReq::build(cmd).as_bytes()).unwrap(),
        }
    }

    pub(crate) fn write_attr(cmd: UfsAttrCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: <[u8; 500]>::try_from(UpiuWriteAttrReq::build(cmd).as_bytes()).unwrap(),
        }
    }

    pub(crate) fn read_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_read(tag),
            body: <[u8; 500]>::try_from(UpiuFlagReq::build(cmd, QueryOpcode::ReadFlag).as_bytes())
                .unwrap(),
        }
    }

    pub(crate) fn set_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: <[u8; 500]>::try_from(UpiuFlagReq::build(cmd, QueryOpcode::SetFlag).as_bytes())
                .unwrap(),
        }
    }

    pub(crate) fn clear_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: <[u8; 500]>::try_from(UpiuFlagReq::build(cmd, QueryOpcode::ClearFlag).as_bytes())
                .unwrap(),
        }
    }

    pub(crate) fn toggle_flag(cmd: UfsFlagCmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::query_write(tag),
            body: <[u8; 500]>::try_from(
                UpiuFlagReq::build(cmd, QueryOpcode::ToggleFlag).as_bytes(),
            )
            .unwrap(),
        }
    }

    pub(crate) fn command(cmd: UfsSCSICmd, tag: usize) -> Self {
        Self {
            header: UpiuHeader::command(cmd, tag),
            body: <[u8; 500]>::try_from(UpiuCmd::command(cmd).as_bytes()).unwrap(),
        }
    }

    pub(crate) fn device(cmd: UfsDevCmd, tag: usize) -> Self {
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

    pub(crate) fn scsi_result(&self, ocs: u8) -> UfsScsiResult {
        let rsp: UpiuRsp = zerocopy::transmute!(self.body);
        let sense_data_len = usize::from(u16::from_be(rsp.sendse_data_len)).min(UFS_SENSE_SIZE);

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
            UpiuTransaction::Response => {}
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

    pub(crate) fn fetch_dev(&self, cmd: UfsDevCmd) -> Result<UfsDevCmd> {
        match cmd {
            UfsDevCmd::Nop => match self.transaction() {
                UpiuTransaction::NopIn => Ok(cmd),
                _ => Err(EIO),
            },
            UfsDevCmd::Query(cmd) => match self.transaction() {
                UpiuTransaction::QueryRsp => match self.response() {
                    UpiuResponse::Success => self.fetch_query(cmd),
                    _ => Err(EIO),
                },
                _ => Err(EIO),
            },
            UfsDevCmd::RPMB(cmd) => match self.transaction() {
                UpiuTransaction::Response => Ok(UfsDevCmd::RPMB(cmd)),
                _ => Err(EIO),
            },
        }
    }

    fn fetch_query(&self, cmd: UfsQueryCmd) -> Result<UfsDevCmd> {
        match cmd {
            UfsQueryCmd::ReadDesc(cmd) => {
                let upiu: UpiuReadDescRsp = zerocopy::transmute!(self.body);
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadDesc(UfsDescCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    length: u16::from_be(upiu.length),
                    desc: Desc::from_buffer(upiu.idn, upiu.buffer),
                })))
            }
            UfsQueryCmd::ReadAttr(cmd) => {
                let upiu: UpiuAttrRsp = zerocopy::transmute!(self.body);
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadAttr(UfsAttrCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: u64::from_be(upiu.value),
                })))
            }
            UfsQueryCmd::WriteAttr(cmd) => {
                let upiu: UpiuAttrRsp = zerocopy::transmute!(self.body);
                Ok(UfsDevCmd::Query(UfsQueryCmd::WriteAttr(UfsAttrCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: u64::from_be(upiu.value),
                })))
            }
            UfsQueryCmd::ReadFlag(cmd) => {
                let upiu: UpiuFlagRsp = zerocopy::transmute!(self.body);
                Ok(UfsDevCmd::Query(UfsQueryCmd::ReadFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            }
            UfsQueryCmd::SetFlag(cmd) => {
                let upiu: UpiuFlagRsp = zerocopy::transmute!(self.body);
                Ok(UfsDevCmd::Query(UfsQueryCmd::SetFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            }
            UfsQueryCmd::ClearFlag(cmd) => {
                let upiu: UpiuFlagRsp = zerocopy::transmute!(self.body);
                Ok(UfsDevCmd::Query(UfsQueryCmd::ClearFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            }
            UfsQueryCmd::ToggleFlag(cmd) => {
                let upiu: UpiuFlagRsp = zerocopy::transmute!(self.body);
                Ok(UfsDevCmd::Query(UfsQueryCmd::ToggleFlag(UfsFlagCmd {
                    idn: upiu.idn.into(),
                    index: upiu.index,
                    selector: upiu.selector,
                    value: upiu.value,
                })))
            }
            _ => Ok(UfsDevCmd::Query(cmd)),
        }
    }
}

const _: () = {
    assert!(size_of::<UpiuHeader>() == 12);
};
const _: () = {
    assert!(size_of::<UpiuCmd>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuRsp>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuTmReq>() == 32);
};
const _: () = {
    assert!(size_of::<UpiuTmRsp>() == 32);
};
const _: () = {
    assert!(size_of::<UpiuReadDescReq>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuWriteDescReq>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuReadAttrReq>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuWriteAttrReq>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuFlagReq>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuQueryReq>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuReadDescRsp>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuWriteDescRsp>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuAttrRsp>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuFlagRsp>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuQueryRsp>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuNop>() == 500);
};
const _: () = {
    assert!(size_of::<UpiuBody>() == 500);
};
const _: () = {
    assert!(size_of::<Upiu>() == 512);
};

unsafe impl kernel::transmute::AsBytes for Upiu {}
unsafe impl kernel::transmute::FromBytes for Upiu {}
