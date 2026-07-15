// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use crate::ufs_dma::{DescBuffer, MAX_PRD_ENTRIES};
use crate::ufs_lu::{QueueData, UfsLuBlockOps};
use crate::ufs_queue::*;
use kernel::alloc::mempool::MemPool;
use kernel::block::error::BlkResult;
use kernel::block::mq::dma_map_iter::DmaMapMempool;
use kernel::block::mq::{
    self, BoundRequestQueue, IdleRequest, LimitsBuilder, Request, RequestQueue,
};
use kernel::error::{from_err_ptr, to_result};
use kernel::io::poll::read_poll_timeout;
use kernel::sync::{aref::ARef, Arc, Mutex};
use kernel::time::{delay, Delta};
use kernel::types::Owned;
use kernel::uapi::NUMA_NO_NODE;
use kernel::{bindings, kvec, new_mutex, prelude::*};

const NOP_OUT_TIMEOUT_MS: i64 = 50;
const QUERY_DEFAULT_TIMEOUT_MS: i64 = 1500;
const ADVANCDE_RPMB_TIMEOUT_MS: i64 = 3000;

const FDEVICE_COMPL_TIMEOUT_MS: i64 = 1500;
const FDEVICE_COMPL_TICK_US: i64 = 500;

pub(crate) const QUERY_DESC_MAX_SIZE: usize = 255;

struct TaskManagementOps {}

#[vtable]
impl mq::Operations for TaskManagementOps {
    type RequestData = ();
    type QueueData = ();
    type HwData = ();
    type TagSetData = ();
    type GenDiskData = ();

    fn new_request_data() -> impl PinInit<Self::RequestData> {
        ()
    }

    fn queue_rq(
        hw_data: (),
        queue_data: (),
        rq: Owned<IdleRequest<Self>>,
        is_last: bool,
    ) -> BlkResult {
        todo!()
    }

    fn commit_rqs(hw_data: (), queue_data: ()) {
        todo!()
    }

    fn init_hctx(tagset_data: (), hctx_idx: u32) -> Result<Self::HwData> {
        Ok(())
    }

    fn complete(rq: ARef<Request<Self>>) {
        todo!()
    }
}

// This queue is scaffolding for future task-management/error-recovery support.
// RUFS does not issue TMF requests yet; `queue_tmf()` is only a guard callback
// so accidental dispatch is rejected instead of silently completing.
struct TmfQueue {
    tag_set: Arc<mq::TagSet<TaskManagementOps>>,
    queue: BoundRequestQueue<TaskManagementOps>,
}

// SAFETY: `TmfQueue` owns the blk-mq tag set, request queue, and request pointer
// table. Access to it is serialized by `UfsDev::tmf_queue`.
unsafe impl Send for TmfQueue {}

impl TmfQueue {
    fn new(depth: usize) -> Result<Self> {
        let tag_set = Arc::pin_init(
            mq::TagSet::new(
                1,
                (),
                depth.try_into()?,
                1,
                kernel::alloc::NumaNode::NO_NODE,
                mq::tag_set::Flags::empty(),
            ),
            GFP_KERNEL,
        )?;

        let queue = mq::RequestQueue::new(
            tag_set.clone(),
            LimitsBuilder::<TaskManagementOps>::new().build()?,
            (),
            depth as u32,
        )?;

        Ok(Self { tag_set, queue })
    }
}

#[derive(Copy, Clone)]
pub(crate) enum DescIdn {
    Device = 0x0,
    Config = 0x1,
    Unit = 0x2,
    RFU0 = 0x3,
    Interconn = 0x4,
    String = 0x5,
    RFU1 = 0x6,
    Geometry = 0x7,
    Power = 0x8,
    Health = 0x9,
    Reserved = 0xFF,
}

impl From<u8> for DescIdn {
    fn from(idn: u8) -> Self {
        match idn {
            0x0 => Self::Device,
            0x1 => Self::Config,
            0x2 => Self::Unit,
            0x3 => Self::RFU0,
            0x4 => Self::Interconn,
            0x5 => Self::String,
            0x6 => Self::RFU1,
            0x7 => Self::Geometry,
            0x8 => Self::Power,
            0x9 => Self::Health,
            _ => Self::Reserved,
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct DeviceDesc {
    length: u8,
    descriptor_idn: u8,
    device: u8,
    device_class: u8,
    device_sub_class: u8,
    protocol: u8,
    number_lu: u8,
    number_wlu: u8,
    boot_enable: u8,
    descr_access_en: u8,
    init_power_mode: u8,
    high_priority_lun: u8,
    secure_removal_type: u8,
    security_lu: u8,
    background_ops_term_lat: u8,
    init_active_icc_level: u8,
    spec_version: u16,
    manufacture_date: u16,
    manufacturer_name: u8,
    product_name: u8,
    serial_number: u8,
    oem_id: u8,
    manufacturer_id: u16,
    ud_0_base_offset: u8,
    ud_config_p_length: u8,
    device_rtt_cap: u8,
    periodic_rtc_update: u16,
    ufs_features_support: u8,
    ffu_timeout: u8,
    queue_depth: u8,
    device_version: u16,
    num_secure_wp_area: u8,
    psa_max_data_size: u32,
    psa_state_timeout: u8,
    product_revision_level: u8,
    reserved: [u8; 34],
    extended_wb_support: u16,
    extended_ufs_features_support: u32,
    write_booster_buffer_preserve_user_space_en: u8,
    write_booster_buffer_type: u8,
    num_shared_write_booster_buffer_alloc_units: u32,
    _padding: [u8; 166],
}

impl DeviceDesc {
    fn from_buffer(buffer: DescBuffer) -> Self {
        unsafe { core::ptr::read_unaligned(buffer.data.as_ptr().cast::<Self>()) }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) struct GeometryDesc {
    length: u8,
    descriptor_idn: u8,
    media_technology: u8,
    reserved: u8,
    total_raw_device_capacity: u64,
    max_number_lu: u8,
    segment_size: u32,
    allocation_unit_size: u8,
    min_addr_block_size: u8,
    optimal_read_block_size: u8,
    optimal_write_block_size: u8,
    max_in_buffer_size: u8,
    max_out_buffer_size: u8,
    rpmb_read_write_size: u8,
    dynamic_capacity_resource_policy: u8,
    data_ordering: u8,
    max_context_id_number: u8,
    sys_data_tag_unit_size: u8,
    sys_data_tag_res_size: u8,
    supported_sec_r_types: u8,
    supported_memory_types: u16,
    system_code_max_n_alloc_u: u32,
    system_code_cap_adj_fac: u16,
    non_persist_max_n_alloc_u: u32,
    non_persist_cap_adj_fac: u16,
    enhanced_1_max_n_alloc_u: u32,
    enhanced_1_cap_adj_fac: u16,
    enhanced_2_max_n_alloc_u: u32,
    enhanced_2_cap_adj_fac: u16,
    enhanced_3_max_n_alloc_u: u32,
    enhanced_3_cap_adj_fac: u16,
    enhanced_4_max_n_alloc_u: u32,
    enhanced_4_cap_adj_fac: u16,
    optimal_logical_block_size: u32,
    reserved2: [u8; 7],
    write_booster_buffer_max_n_alloc_units: u32,
    device_max_write_booster_l_us: u8,
    write_booster_buffer_cap_adj_fac: u8,
    supported_write_booster_buffer_user_space_reduction_types: u8,
    supported_write_booster_buffer_types: u8,
    reserved3: [u8; 17],
    cap_adj_fac_representation: u8,
    _padding: [u8; 150],
}

impl GeometryDesc {
    fn from_buffer(buffer: DescBuffer) -> Self {
        unsafe { core::ptr::read_unaligned(buffer.data.as_ptr().cast::<Self>()) }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) struct UnitDesc {
    length: u8,
    descriptor_idn: u8,
    unit_index: u8,
    lu_enable: u8,
    boot_lun_id: u8,
    lu_write_protect: u8,
    lu_queue_depth: u8,
    psa_sensitive: u8,
    memory_type: u8,
    data_reliability: u8,
    logical_block_size: u8,
    logical_block_count: u64,
    erase_block_size: u32,
    provisioning_type: u8,
    phy_mem_resource_count: u64,
    context_capabilities: u16,
    large_unit_granularity_m1: u8,
    reserved: [u8; 6],
    lu_num_write_booster_buffer_alloc_units: u32,
    _padding: [u8; 210],
}

impl UnitDesc {
    fn from_buffer(buffer: DescBuffer) -> Self {
        unsafe { core::ptr::read_unaligned(buffer.data.as_ptr().cast::<Self>()) }
    }

    pub(crate) fn enabled(&self) -> bool {
        self.lu_enable != 0
    }

    pub(crate) fn logical_block_shift(&self) -> u8 {
        self.logical_block_size
    }

    pub(crate) fn logical_block_count(&self) -> u64 {
        let ptr = core::ptr::addr_of!(self.logical_block_count);

        // SAFETY: `UnitDesc` is packed, so integer fields may be unaligned.
        // The descriptor stores multi-byte values in big-endian byte order.
        unsafe { u64::from_be(core::ptr::read_unaligned(ptr)) }
    }

    pub(crate) fn lu_queue_depth(&self) -> usize {
        self.lu_queue_depth as usize
    }
}

pub(crate) type DefaultDesc = DescBuffer;

#[derive(Clone, Copy)]
pub(crate) enum Desc {
    Device(DeviceDesc),
    Config(DefaultDesc),
    Unit(UnitDesc),
    RFU0(DefaultDesc),
    Interconn(DefaultDesc),
    String(DefaultDesc),
    RFU1(DefaultDesc),
    Geometry(GeometryDesc),
    Power(DefaultDesc),
    Health(DefaultDesc),
    Reserved,
}

impl Desc {
    fn get_device(&self) -> Result<DeviceDesc> {
        match *self {
            Self::Device(desc) => Ok(desc),
            _ => Err(EINVAL),
        }
    }

    fn get_geometry(&self) -> Result<GeometryDesc> {
        match *self {
            Self::Geometry(desc) => Ok(desc),
            _ => Err(EINVAL),
        }
    }

    fn get_unit(&self) -> Result<UnitDesc> {
        match *self {
            Self::Unit(desc) => Ok(desc),
            _ => Err(EINVAL),
        }
    }

    pub(crate) fn from_buffer(idn: u8, buffer: DescBuffer) -> Self {
        let idn: DescIdn = idn.into();
        match idn {
            DescIdn::Device => Self::Device(DeviceDesc::from_buffer(buffer)),
            DescIdn::Config => Self::Config(buffer),
            DescIdn::Unit => Self::Unit(UnitDesc::from_buffer(buffer)),
            DescIdn::RFU0 => Self::RFU0(buffer),
            DescIdn::Interconn => Self::Interconn(buffer),
            DescIdn::String => Self::String(buffer),
            DescIdn::RFU1 => Self::RFU1(buffer),
            DescIdn::Geometry => Self::Geometry(GeometryDesc::from_buffer(buffer)),
            DescIdn::Power => Self::Power(buffer),
            DescIdn::Health => Self::Health(buffer),
            _ => Self::Reserved,
        }
    }
}

// Query Command
#[derive(Copy, Clone)]
pub(crate) struct UfsDescCmd {
    pub(crate) idn: DescIdn,
    pub(crate) index: u8,
    pub(crate) selector: u8,
    pub(crate) length: u16,
    pub(crate) desc: Desc,
}

impl UfsDescCmd {
    fn build(idn: DescIdn, index: u8, selector: u8) -> Self {
        Self {
            idn,
            index,
            selector,
            length: QUERY_DESC_MAX_SIZE as u16,
            desc: Desc::Reserved,
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) enum AttrIdn {
    BootLuEn = 0x00,
    MaxHPBSingleCmd = 0x01,
    PowerMode = 0x02,
    ActiveICCLevel = 0x03,
    OOODataEn = 0x04,
    BkOpsStatus = 0x05,
    PurgeStatus = 0x06,
    MaxDataIn = 0x07,
    MaxDataOut = 0x08,
    DynCapNeeded = 0x09,
    RefClkFreq = 0x0A,
    ConfDescLock = 0x0B,
    MaxNumOfRTT = 0x0C,
    EEControl = 0x0D,
    EEStatus = 0x0E,
    SecondsPassed = 0x0F,
    CntxConf = 0x10,
    CorrPrgBlkNum = 0x11,
    FFUStatus = 0x14,
    PSAState = 0x15,
    PSADataSize = 0x16,
    RefClkGatingWaitTime = 0x17,
    CaseRoughTemp = 0x18,
    HighTempBound = 0x19,
    LowTempBound = 0x1A,
    WBFlushStatus = 0x1C,
    AvailWBBuffSize = 0x1D,
    WBBuffLifeTimeEst = 0x1E,
    CurrWBBuffSize = 0x1F,
    Timestamp = 0x30,
    DevLvlExceptionID = 0x34,
    HIDDefragOperation = 0x35,
    HIDAvailableSize = 0x36,
    HIDSize = 0x37,
    HIDProgressRatio = 0x38,
    HIDState = 0x39,
    WBBuffResizeHint = 0x3C,
    WBBuffResizeEn = 0x3D,
    WBBuffResizeStatus = 0x3E,
    Reserved = 0xFF,
}

impl From<u8> for AttrIdn {
    fn from(idn: u8) -> Self {
        match idn {
            0x00 => Self::BootLuEn,
            0x01 => Self::MaxHPBSingleCmd,
            0x02 => Self::PowerMode,
            0x03 => Self::ActiveICCLevel,
            0x04 => Self::OOODataEn,
            0x05 => Self::BkOpsStatus,
            0x06 => Self::PurgeStatus,
            0x07 => Self::MaxDataIn,
            0x08 => Self::MaxDataOut,
            0x09 => Self::DynCapNeeded,
            0x0A => Self::RefClkFreq,
            0x0B => Self::ConfDescLock,
            0x0C => Self::MaxNumOfRTT,
            0x0D => Self::EEControl,
            0x0E => Self::EEStatus,
            0x0F => Self::SecondsPassed,
            0x10 => Self::CntxConf,
            0x11 => Self::CorrPrgBlkNum,
            0x14 => Self::FFUStatus,
            0x15 => Self::PSAState,
            0x16 => Self::PSADataSize,
            0x17 => Self::RefClkGatingWaitTime,
            0x18 => Self::CaseRoughTemp,
            0x19 => Self::HighTempBound,
            0x1A => Self::LowTempBound,
            0x1C => Self::WBFlushStatus,
            0x1D => Self::AvailWBBuffSize,
            0x1E => Self::WBBuffLifeTimeEst,
            0x1F => Self::CurrWBBuffSize,
            0x30 => Self::Timestamp,
            0x34 => Self::DevLvlExceptionID,
            0x35 => Self::HIDDefragOperation,
            0x36 => Self::HIDAvailableSize,
            0x37 => Self::HIDSize,
            0x38 => Self::HIDProgressRatio,
            0x39 => Self::HIDState,
            0x3C => Self::WBBuffResizeHint,
            0x3D => Self::WBBuffResizeEn,
            0x3E => Self::WBBuffResizeStatus,
            _ => Self::Reserved,
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct UfsAttrCmd {
    pub(crate) idn: AttrIdn,
    pub(crate) index: u8,
    pub(crate) selector: u8,
    pub(crate) value: u64,
}

impl UfsAttrCmd {
    fn build(idn: AttrIdn, index: u8, selector: u8, value: u64) -> Self {
        Self {
            idn,
            index,
            selector,
            value,
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) enum FlagIdn {
    Reserved = 0x00,
    FDeviceInit = 0x01,
    PermanentWPE = 0x02,
    PwrOnWPE = 0x03,
    BkOpsEn = 0x04,
    LifeSpanModeEnable = 0x05,
    PurgeEnable = 0x06,
    FPhyResourceRemoval = 0x08,
    BusyRTC = 0x09,
    PermanentlyDisableFWUpdate = 0x0B,
    WBEn = 0x0E,
    WBBuffFlushEn = 0x0F,
    WBBuffFlushDuringHibern8 = 0x10,
    HPBReset = 0x11,
    HPBEn = 0x12,
    UnpinEn = 0x13,
}

impl From<u8> for FlagIdn {
    fn from(idn: u8) -> Self {
        match idn {
            0x01 => Self::FDeviceInit,
            0x02 => Self::PermanentWPE,
            0x03 => Self::PwrOnWPE,
            0x04 => Self::BkOpsEn,
            0x05 => Self::LifeSpanModeEnable,
            0x06 => Self::PurgeEnable,
            0x08 => Self::FPhyResourceRemoval,
            0x09 => Self::BusyRTC,
            0x0B => Self::PermanentlyDisableFWUpdate,
            0x0E => Self::WBEn,
            0x0F => Self::WBBuffFlushEn,
            0x10 => Self::WBBuffFlushDuringHibern8,
            0x11 => Self::HPBReset,
            0x12 => Self::HPBEn,
            0x13 => Self::UnpinEn,
            _ => Self::Reserved,
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct UfsFlagCmd {
    pub(crate) idn: FlagIdn,
    pub(crate) index: u8,
    pub(crate) selector: u8,
    pub(crate) value: u8,
}

impl UfsFlagCmd {
    fn build(idn: FlagIdn, index: u8, selector: u8) -> Self {
        Self {
            idn,
            index,
            selector,
            value: 0,
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) enum UfsQueryCmd {
    Nop,
    ReadDesc(UfsDescCmd),
    WriteDesc(UfsDescCmd),
    ReadAttr(UfsAttrCmd),
    WriteAttr(UfsAttrCmd),
    ReadFlag(UfsFlagCmd),
    SetFlag(UfsFlagCmd),
    ClearFlag(UfsFlagCmd),
    ToggleFlag(UfsFlagCmd),
}

impl Default for UfsQueryCmd {
    fn default() -> Self {
        Self::Nop
    }
}

impl UfsQueryCmd {
    fn read_desc(&self, idn: DescIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsDescCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ReadDesc(cmd)))
    }

    fn read_attr(&self, idn: AttrIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsAttrCmd::build(idn, index, selector, 0);
        UfsCmd::Device(UfsDevCmd::Query(Self::ReadAttr(cmd)))
    }

    fn write_attr(&self, idn: AttrIdn, index: u8, selector: u8, value: u64) -> UfsCmd {
        let cmd = UfsAttrCmd::build(idn, index, selector, value);
        UfsCmd::Device(UfsDevCmd::Query(Self::WriteAttr(cmd)))
    }

    fn read_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ReadFlag(cmd)))
    }

    fn set_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::SetFlag(cmd)))
    }

    fn clear_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ClearFlag(cmd)))
    }

    fn toggle_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ToggleFlag(cmd)))
    }

    fn get_read_desc(&self) -> Result<UfsDescCmd> {
        match *self {
            Self::ReadDesc(cmd) => Ok(cmd),
            _ => Err(EINVAL),
        }
    }

    fn get_attr_value(&self) -> Result<u64> {
        match *self {
            Self::ReadAttr(cmd) => Ok(cmd.value),
            Self::WriteAttr(cmd) => Ok(cmd.value),
            _ => Err(EINVAL),
        }
    }

    fn get_flag_value(&self) -> Result<u8> {
        match *self {
            Self::ReadFlag(cmd) => Ok(cmd.value),
            Self::SetFlag(cmd) => Ok(cmd.value),
            Self::ClearFlag(cmd) => Ok(cmd.value),
            Self::ToggleFlag(cmd) => Ok(cmd.value),
            _ => Err(EINVAL),
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct UfsRPMBCmd {}

#[derive(Copy, Clone)]
pub(crate) enum UfsDevCmd {
    Nop,
    Query(UfsQueryCmd),
    RPMB(UfsRPMBCmd),
}

impl UfsDevCmd {
    fn nop() -> UfsCmd {
        UfsCmd::Device(Self::Nop)
    }
    fn query() -> UfsQueryCmd {
        UfsQueryCmd::default()
    }

    pub(crate) fn timeout(&self) -> Delta {
        match *self {
            Self::Nop => Delta::from_millis(NOP_OUT_TIMEOUT_MS),
            Self::Query(_) => Delta::from_millis(QUERY_DEFAULT_TIMEOUT_MS),
            Self::RPMB(_) => Delta::from_millis(ADVANCDE_RPMB_TIMEOUT_MS),
        }
    }

    fn get_query(&self) -> Result<UfsQueryCmd> {
        match *self {
            Self::Query(cmd) => Ok(cmd),
            _ => Err(EINVAL),
        }
    }
}

#[derive(Default)]
pub(crate) struct UfsDevInfo {
    max_lu: usize,
    num_lu: usize,
    num_wlu: usize,
    manufacturer_id: u16,
    spec_version: u16,
    queue_depth: usize,
    rtt_cap: u8,
    luns_avail: usize,
}

#[pin_data]
pub(crate) struct UfsDev {
    ufs_queue: Arc<UfsQueue>,
    pub(crate) request_queue: BoundRequestQueue<UfsLuBlockOps>,

    #[pin]
    pub(crate) info: Mutex<UfsDevInfo>,

    #[pin]
    tmf_queue: Mutex<Option<TmfQueue>>,
}

impl UfsDev {
    pub(crate) fn new(ufs_queue: Arc<UfsQueue>) -> Result<Arc<Self>> {
        let limits = LimitsBuilder::<UfsLuBlockOps>::new().build()?;

        let request_queue = RequestQueue::new(
            ufs_queue.tags.clone(),
            limits,
            KBox::new(QueueData::Dev(ufs_queue.clone()), GFP_KERNEL)?,
            ufs_queue.tags.queue_depth(),
        )?;

        let this = Arc::pin_init(
            try_pin_init!(Self {
                ufs_queue,
                request_queue,
                info <- new_mutex!(UfsDevInfo::default()),
                tmf_queue <- new_mutex!(None),
            }),
            GFP_KERNEL,
        )?;

        Ok(this)
    }

    // Allocate the placeholder TMF blk-mq objects early so the ownership and
    // cleanup path are exercised, but do not treat this as functional TMF
    // support. Real TMF request composition/completion belongs with error
    // recovery.
    pub(crate) fn alloc_tmf_queue(&self, depth: usize) -> Result<()> {
        let mut tmf_queue = self.tmf_queue.lock();
        if tmf_queue.is_some() {
            return Err(EBUSY);
        }

        tmf_queue.replace(TmfQueue::new(depth)?);
        pr_info!("[RUFS] ufs_dev: allocated TMF queue depth {}", depth);
        Ok(())
    }

    fn submit(&self, cmd: UfsCmd) -> Result<UfsCmd> {
        let mut rq = self
            .request_queue
            .alloc_sync_request(mq::Command::DriverOut)?;
        rq.data_ref().inner.lock().prepare_device(cmd)?;
        rq.as_pin_mut().execute(true)?;
        let result = rq.data_ref().inner.lock().take_device_completion();
        result
    }

    fn nop(&self) -> Result<()> {
        let cmd = self.submit(UfsDevCmd::nop())?;
        Ok(())
    }

    pub(crate) fn verify_dev_init(&self) -> Result<()> {
        self.nop()?;
        pr_info!("[RUFS] ufs_dev: device verified");
        Ok(())
    }

    fn read_desc(&self, idn: DescIdn, index: u8, selector: u8) -> Result<Desc> {
        let cmd = self.submit(UfsDevCmd::query().read_desc(idn, index, selector))?;
        Ok(cmd.get_device()?.get_query()?.get_read_desc()?.desc)
    }

    fn read_attr(&self, idn: AttrIdn, index: u8, selector: u8) -> Result<u64> {
        let cmd = self.submit(UfsDevCmd::query().read_attr(idn, index, selector))?;
        Ok(cmd.get_device()?.get_query()?.get_attr_value()?)
    }

    pub(crate) fn read_unit_desc(&self, lun: u8) -> Result<UnitDesc> {
        self.read_desc(DescIdn::Unit, lun, 0)?.get_unit()
    }

    pub(crate) fn num_lu(&self) -> usize {
        self.info.lock().num_lu
    }

    fn write_attr(&self, idn: AttrIdn, index: u8, selector: u8, value: u64) -> Result<()> {
        let cmd = self.submit(UfsDevCmd::query().write_attr(idn, index, selector, value))?;
        if cmd.get_device()?.get_query()?.get_attr_value()? == value {
            Ok(())
        } else {
            Err(EIO)
        }
    }

    fn read_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<u8> {
        let cmd = self.submit(UfsDevCmd::query().read_flag(idn, index, selector))?;
        Ok(cmd.get_device()?.get_query()?.get_flag_value()?)
    }

    fn set_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<()> {
        self.submit(UfsDevCmd::query().set_flag(idn, index, selector))?;
        Ok(())
    }

    fn clear_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<()> {
        self.submit(UfsDevCmd::query().clear_flag(idn, index, selector))?;
        Ok(())
    }

    fn toggle_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<u8> {
        let cmd = self.submit(UfsDevCmd::query().toggle_flag(idn, index, selector))?;
        Ok(cmd.get_device()?.get_query()?.get_flag_value()?)
    }

    pub(crate) fn complete_dev_init(&self) -> Result<()> {
        self.set_flag(FlagIdn::FDeviceInit, 0, 0)?;

        let result = read_poll_timeout(
            || self.read_flag(FlagIdn::FDeviceInit, 0, 0),
            |flag: &u8| *flag == 0,
            Delta::from_micros(FDEVICE_COMPL_TICK_US),
            Delta::from_millis(FDEVICE_COMPL_TIMEOUT_MS),
        );
        match result {
            Ok(_) => {
                pr_info!("[RUFS] ufs_dev: device initialized\n");
                Ok(())
            }
            Err(ETIMEDOUT) => {
                pr_err!("[RUFS] ufs_dev: fDeviceInit was not cleared\n");
                Err(EBUSY)
            }
            Err(e) => {
                pr_err!(
                    "[RUFS] ufs_dev: failed to read fDeviceInit errno={}\n",
                    e.to_errno(),
                );
                Err(e)
            }
        }
    }

    pub(crate) fn device_params_init(&self) -> Result<()> {
        self.get_geometry_info()?;
        self.get_device_info()
    }

    fn get_geometry_info(&self) -> Result<()> {
        let desc = self.read_desc(DescIdn::Geometry, 0, 0)?.get_geometry()?;
        match desc.max_number_lu {
            1 => {
                self.info.lock().max_lu = 32;
            }
            _ => {
                self.info.lock().max_lu = 8;
            }
        }

        Ok(())
    }

    fn get_device_info(&self) -> Result<()> {
        let desc = self.read_desc(DescIdn::Device, 0, 0)?.get_device()?;
        self.info.lock().manufacturer_id = u16::from_be(desc.manufacturer_id);
        self.info.lock().spec_version = u16::from_be(desc.spec_version);
        self.info.lock().queue_depth = desc.queue_depth as usize;
        self.info.lock().rtt_cap = desc.device_rtt_cap;
        self.info.lock().num_lu = desc.number_lu as usize;
        self.info.lock().num_wlu = desc.number_wlu as usize;
        self.info.lock().luns_avail = (desc.number_lu + desc.number_wlu) as usize;

        Ok(())
    }
}
