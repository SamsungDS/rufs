// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_variables)]

use kernel::{pci, device::Core, prelude::*, new_spinlock};
use kernel::{dma, dma_read, dma_write};
use kernel::bits::genmask_u8;
use kernel::sync::{Arc, SpinLock};
use crate::ufs_reg::*;
use crate::ufs_queue::*;
use crate::ufs_dev::*;

const PRDT_DATA_BYTE_COUNT_MAX: u32 = 0x00040000; // SZ_256K
const PRDT_DATA_BYTE_COUNT_PAD: usize = 4;
const ALIGNED_UPIU_SIZE: usize = 512;
const MAX_PRD_ENTRIES: usize = 256;
const UFS_CDB_SIZE: usize = 16;
const UFS_SENSE_SIZE: usize = 18;
const MASK_OCS: u8 = 0x0F;

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

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) struct DefaultDesc {
    _padding: [u8; 255],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) union DescBuffer {
    pub(crate) device: DefaultDesc,
    pub(crate) config: DefaultDesc,
    pub(crate) unit: DefaultDesc,
    pub(crate) rfu0: DefaultDesc,
    pub(crate) interconn: DefaultDesc,
    pub(crate) string: DefaultDesc,
    pub(crate) rfu1: DefaultDesc,
    pub(crate) geometry: DefaultDesc,
    pub(crate) power: DefaultDesc,
    pub(crate) health: DefaultDesc,
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
struct UpiuReadAttrRsp {
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
struct UpiuWriteAttrRsp {
    opcode: u8,
    idn: u8,
    index: u8,
    selector: u8,
    _reserved: [u32; 4],
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
    read_attr: UpiuReadAttrRsp,
    write_attr: UpiuWriteAttrRsp,
    read_flag: UpiuFlagRsp,
    set_flag: UpiuFlagRsp,
    clear_flag: UpiuFlagRsp,
    toggle_flag: UpiuFlagRsp,
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

    fn transaction(&self) -> UpiuTransaction {
        self.header.transaction_code.into()
    }

    fn response(&self) -> UpiuResponse {
        self.header.response.into()
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
            UfsCmd::SCSI(cmd) => self.nop(tag),
        }
    }

    fn device(&self, cmd: UfsDevCmd, tag: usize) -> Self {
        self.nop(tag)
    }

    fn nop(&self, tag: usize) -> Self {
        Self {
            cmd_upiu: Upiu::nop_out(tag),
            rsp_upiu: Upiu::default(),
            prdt: self.prdt,
        }
    }

    fn fetch_dev(&self, cmd: UfsDevCmd) -> Result<UfsDevCmd> {
        match self.rsp_upiu.transaction() {
            UpiuTransaction::NopIn => Ok(cmd),
            _ => Err(EIO),
        }
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
        ReqDescHeader::default()
    }
}

#[repr(C, packed)]
#[derive(Default)]
struct Utrd {
    header: ReqDescHeader,
    command_desc_base_addr: u64,
    rsp_upiu_length: u16,
    rsp_upiu_offset: u16,
    prd_table_length: u16,
    prd_table_offset: u16,
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

#[pin_data]
pub(crate) struct UfsDma {
    reg: Arc<UfsReg>,

    #[pin]
    inner: SpinLock<UfsDmaInner>,
}

// SAFETY: UfsDma itself doesn't have any thread-affinity
unsafe impl Send for UfsDma {}

impl UfsDma {
    pub(crate) fn new(
        pdev: &pci::Device<Core>,
        reg: Arc<UfsReg>,
    ) -> Result<Arc<Self>> {
        let nutrs = reg.nutrs();
        let ucdl = dma::Coherent::<Ucd>::zeroed_slice(
            pdev.as_ref(), nutrs, GFP_KERNEL,
        )?;

        let utrdl = dma::Coherent::<Utrd>::zeroed_slice(
            pdev.as_ref(), nutrs, GFP_KERNEL,
        )?;

        for tag in 0..nutrs {
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
                inner <- new_spinlock!(UfsDmaInner {
                    ucdl,
                    utrdl,
                    utmrdl,
                }),
            }),
            GFP_KERNEL
        )
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

        let utrd = dma_read!(inner.utrdl, [tag]?);
        dma_write!(inner.utrdl, [tag]?, utrd.build(UfsCmd::Device(cmd)));

        let ucd = dma_read!(inner.ucdl, [tag]?);
        dma_write!(inner.ucdl, [tag]?, ucd.build(UfsCmd::Device(cmd), tag));
        Ok(())
    }

    pub(crate) fn fetch_devman_upiu(
        &self,
        cmd: UfsDevCmd,
        tag: usize,
    ) -> Result<UfsCmd> {
        let inner = self.inner.lock();

        let utrd = dma_read!(inner.utrdl, [tag]?);
        utrd.check_response()?;

        let ucd = dma_read!(inner.ucdl, [tag]?);
        let cmd = ucd.fetch_dev(cmd)?;

        Ok(UfsCmd::Device(cmd))
    }
}

const _: () = { assert!(size_of::<UpiuHeader>() == 12); };
const _: () = { assert!(size_of::<UpiuCmd>() == 500); };
const _: () = { assert!(size_of::<UpiuRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuTmReq>() == 32); };
const _: () = { assert!(size_of::<UpiuTmRsp>() == 32); };
const _: () = { assert!(size_of::<UpiuReadDescReq>() == 500); };
const _: () = { assert!(size_of::<UpiuWriteDescReq>() == 500); };
const _: () = { assert!(size_of::<UpiuReadAttrReq>() == 500); };
const _: () = { assert!(size_of::<UpiuWriteAttrReq>() == 500); };
const _: () = { assert!(size_of::<UpiuFlagReq>() == 500); };
const _: () = { assert!(size_of::<UpiuQueryReq>() == 500); };
const _: () = { assert!(size_of::<UpiuReadDescRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuWriteDescRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuReadAttrRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuWriteAttrRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuFlagRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuQueryRsp>() == 500); };
const _: () = { assert!(size_of::<UpiuNop>() == 500); };
const _: () = { assert!(size_of::<UpiuBody>() == 500); };
const _: () = { assert!(size_of::<Upiu>() == 512); };
const _: () = { assert!(size_of::<ReqDescHeader>() == 16); };
const _: () = { assert!(size_of::<PrdEntry>() == 16); };
const _: () = { assert!(size_of::<Ucd>() == 5120); };
const _: () = { assert!(size_of::<Utrd>() == 32); };
const _: () = { assert!(size_of::<Utmrd>() == 80); };

unsafe impl kernel::transmute::AsBytes for Ucd {}
unsafe impl kernel::transmute::FromBytes for Ucd {}
unsafe impl kernel::transmute::AsBytes for Utrd {}
unsafe impl kernel::transmute::FromBytes for Utrd {}
unsafe impl kernel::transmute::AsBytes for Utmrd {}
unsafe impl kernel::transmute::FromBytes for Utmrd {}
