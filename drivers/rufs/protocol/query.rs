// SPDX-License-Identifier: GPL-2.0

//! UFS query commands and descriptor definitions.

#![allow(dead_code)]

use super::UfsCmd;
use kernel::{prelude::*, time::Delta};
use zerocopy_derive::Immutable;

pub(crate) const QUERY_DESC_MAX_SIZE: usize = 255;
pub(crate) const UFS_DEV_WRITE_BOOSTER_SUP: u32 = 1 << 8;
pub(crate) const WB_BUF_MODE_LU_DEDICATED: u8 = 0;
pub(crate) const WB_BUF_MODE_SHARED: u8 = 1;

#[repr(C)]
#[derive(Clone, Copy, FromBytes, IntoBytes, Immutable)]
pub(crate) struct DescBuffer {
    pub(crate) data: [u8; QUERY_DESC_MAX_SIZE],
}

const NOP_OUT_TIMEOUT_MS: i64 = 50;
const QUERY_DEFAULT_TIMEOUT_MS: i64 = 1500;
const ADVANCDE_RPMB_TIMEOUT_MS: i64 = 3000;

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
#[derive(Clone, Copy, Debug, FromBytes)]
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
        Self::read_from_bytes(&buffer.data).expect("UFS device descriptor size mismatch")
    }

    pub(crate) fn number_lu(&self) -> u8 {
        self.number_lu
    }

    pub(crate) fn number_wlu(&self) -> u8 {
        self.number_wlu
    }

    pub(crate) fn spec_version(&self) -> u16 {
        u16::from_be(self.spec_version)
    }

    pub(crate) fn manufacturer_id(&self) -> u16 {
        u16::from_be(self.manufacturer_id)
    }

    pub(crate) fn device_rtt_cap(&self) -> u8 {
        self.device_rtt_cap
    }

    pub(crate) fn queue_depth(&self) -> usize {
        self.queue_depth as usize
    }

    pub(crate) fn extended_wb_support(&self) -> u16 {
        u16::from_be(self.extended_wb_support)
    }

    pub(crate) fn extended_ufs_features_support(&self) -> u32 {
        u32::from_be(self.extended_ufs_features_support)
    }

    pub(crate) fn write_booster_buffer_preserve_user_space_en(&self) -> u8 {
        self.write_booster_buffer_preserve_user_space_en
    }

    pub(crate) fn write_booster_buffer_type(&self) -> u8 {
        self.write_booster_buffer_type
    }

    pub(crate) fn num_shared_write_booster_buffer_alloc_units(&self) -> u32 {
        u32::from_be(self.num_shared_write_booster_buffer_alloc_units)
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, FromBytes)]
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
        Self::read_from_bytes(&buffer.data).expect("UFS geometry descriptor size mismatch")
    }

    pub(crate) fn max_number_lu(&self) -> u8 {
        self.max_number_lu
    }

    pub(crate) fn write_booster_buffer_max_n_alloc_units(&self) -> u32 {
        u32::from_be(self.write_booster_buffer_max_n_alloc_units)
    }

    pub(crate) fn device_max_write_booster_l_us(&self) -> u8 {
        self.device_max_write_booster_l_us
    }

    pub(crate) fn write_booster_buffer_cap_adj_fac(&self) -> u8 {
        self.write_booster_buffer_cap_adj_fac
    }

    pub(crate) fn supported_write_booster_buffer_types(&self) -> u8 {
        self.supported_write_booster_buffer_types
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, FromBytes)]
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
        Self::read_from_bytes(&buffer.data).expect("UFS unit descriptor size mismatch")
    }

    pub(crate) fn enabled(&self) -> bool {
        self.lu_enable != 0
    }

    pub(crate) fn logical_block_shift(&self) -> u8 {
        self.logical_block_size
    }

    pub(crate) fn logical_block_count(&self) -> u64 {
        u64::from_be(self.logical_block_count)
    }

    pub(crate) fn lu_queue_depth(&self) -> usize {
        self.lu_queue_depth as usize
    }

    pub(crate) fn lu_num_write_booster_buffer_alloc_units(&self) -> u32 {
        u32::from_be(self.lu_num_write_booster_buffer_alloc_units)
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
    pub(crate) fn get_device(&self) -> Result<DeviceDesc> {
        match *self {
            Self::Device(desc) => Ok(desc),
            _ => Err(EINVAL),
        }
    }

    pub(crate) fn get_geometry(&self) -> Result<GeometryDesc> {
        match *self {
            Self::Geometry(desc) => Ok(desc),
            _ => Err(EINVAL),
        }
    }

    pub(crate) fn get_unit(&self) -> Result<UnitDesc> {
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
    pub(crate) fn read_desc(&self, idn: DescIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsDescCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ReadDesc(cmd)))
    }

    pub(crate) fn read_attr(&self, idn: AttrIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsAttrCmd::build(idn, index, selector, 0);
        UfsCmd::Device(UfsDevCmd::Query(Self::ReadAttr(cmd)))
    }

    pub(crate) fn write_attr(&self, idn: AttrIdn, index: u8, selector: u8, value: u64) -> UfsCmd {
        let cmd = UfsAttrCmd::build(idn, index, selector, value);
        UfsCmd::Device(UfsDevCmd::Query(Self::WriteAttr(cmd)))
    }

    pub(crate) fn read_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ReadFlag(cmd)))
    }

    pub(crate) fn set_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::SetFlag(cmd)))
    }

    pub(crate) fn clear_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ClearFlag(cmd)))
    }

    pub(crate) fn toggle_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> UfsCmd {
        let cmd = UfsFlagCmd::build(idn, index, selector);
        UfsCmd::Device(UfsDevCmd::Query(Self::ToggleFlag(cmd)))
    }

    pub(crate) fn get_read_desc(&self) -> Result<UfsDescCmd> {
        match *self {
            Self::ReadDesc(cmd) => Ok(cmd),
            _ => Err(EINVAL),
        }
    }

    pub(crate) fn get_attr_value(&self) -> Result<u64> {
        match *self {
            Self::ReadAttr(cmd) => Ok(cmd.value),
            Self::WriteAttr(cmd) => Ok(cmd.value),
            _ => Err(EINVAL),
        }
    }

    pub(crate) fn get_flag_value(&self) -> Result<u8> {
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
    pub(crate) fn nop() -> UfsCmd {
        UfsCmd::Device(Self::Nop)
    }
    pub(crate) fn query() -> UfsQueryCmd {
        UfsQueryCmd::default()
    }

    pub(crate) fn timeout(&self) -> Delta {
        match *self {
            Self::Nop => Delta::from_millis(NOP_OUT_TIMEOUT_MS),
            Self::Query(_) => Delta::from_millis(QUERY_DEFAULT_TIMEOUT_MS),
            Self::RPMB(_) => Delta::from_millis(ADVANCDE_RPMB_TIMEOUT_MS),
        }
    }

    pub(crate) fn get_query(&self) -> Result<UfsQueryCmd> {
        match *self {
            Self::Query(cmd) => Ok(cmd),
            _ => Err(EINVAL),
        }
    }
}

const _: () = {
    assert!(size_of::<DescBuffer>() == QUERY_DESC_MAX_SIZE);
};
