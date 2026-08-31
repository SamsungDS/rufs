// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::io::{
    poll::read_poll_timeout,
    register,
    register::Register,
    Io,
};
use kernel::time::Delta;
use kernel::{prelude::*, sync::Arc};

use crate::resource::HostResources;

register! {
    CONTROLLER_CAPABILITIES(u32) @ 0x00 {
        30:30 mcq_supported => bool;
        23:23 auto_hibern8_supported => bool;
        18:16 task_management_request_slots;
        15:8 number_outstanding_rtt;
        4:0 transfer_request_slots_sdb;
    }
    CONTROLLER_CAPABILITIES_MCQ(u32) => CONTROLLER_CAPABILITIES {
        7:0 transfer_request_slots;
    }
    MCQCAP(u32) @ 0x04 {
        23:16 queue_config_pointer;
        7:0 max_queue_supported;
    }
    CONTROLLER_CAPABILITIES_H(u32) @ 0x04 {
        31:0 value;
    }
    UFS_VERSION(u32) @ 0x08 {
        31:0 value;
    }
    INTERRUPT_STATUS(u32) @ 0x20 {
        31:0 value;
    }
    INTERRUPT_ENABLE(u32) @ 0x24 {
        31:0 value;
    }
    CONTROLLER_STATUS(u32) @ 0x30 {
        10:8 power_mode_change_request_status;
        5:5 device_error_indicator => bool;
        4:4 host_error_indicator => bool;
        3:3 uic_command_ready => bool;
        2:2 task_request_list_ready => bool;
        1:1 transfer_request_list_ready => bool;
        0:0 device_present => bool;
    }
    CONTROLLER_ENABLE_REG(u32) @ 0x34 {
        1:1 crypto_general_enable => bool;
        0:0 controller_enable => bool;
    }
    UIC_ERROR_CODE_PHY_ADAPTER_LAYER(u32) @ 0x38 {
        31:0 value;
    }
    UIC_ERROR_CODE_DATA_LINK_LAYER(u32) @ 0x3C {
        31:0 value;
    }
    UIC_ERROR_CODE_NETWORK_LAYER(u32) @ 0x40 {
        31:0 value;
    }
    UIC_ERROR_CODE_TRANSPORT_LAYER(u32) @ 0x44 {
        31:0 value;
    }
    UIC_ERROR_CODE_DME(u32) @ 0x48 {
        31:0 value;
    }
    UTP_TRANSFER_REQ_INT_AGG_CONTROL(u32) @ 0x4C {
        31:0 value;
    }
    UTP_TRANSFER_REQ_LIST_BASE_L(u32) @ 0x50 {
        31:0 value;
    }
    UTP_TRANSFER_REQ_LIST_BASE_H(u32) @ 0x54 {
        31:0 value;
    }
    UTP_TRANSFER_REQ_DOOR_BELL(u32) @ 0x58 {
        31:0 value;
    }
    UTP_TRANSFER_REQ_LIST_CLEAR(u32) @ 0x5C {
        31:0 value;
    }
    UTP_TRANSFER_REQ_LIST_RUN_STOP(u32) @ 0x60 {
        31:0 value;
    }
    UTP_TRANSFER_REQ_LIST_COMPLETION_NOTIFICATION(u32) @ 0x64 {
        31:0 value;
    }
    UTP_TASK_REQ_LIST_BASE_L(u32) @ 0x70 {
        31:0 value;
    }
    UTP_TASK_REQ_LIST_BASE_H(u32) @ 0x74 {
        31:0 value;
    }
    UTP_TASK_REQ_DOOR_BELL(u32) @ 0x78 {
        31:0 value;
    }
    UTP_TASK_REQ_LIST_CLEAR(u32) @ 0x7C {
        31:0 value;
    }
    UTP_TASK_REQ_LIST_RUN_STOP(u32) @ 0x80 {
        31:0 value;
    }
    UIC_COMMAND(u32) @ 0x90 {
        31:0 value;
    }
    UIC_ARG1(u32) @ 0x94 {
        31:0 value;
    }
    UIC_ARG2(u32) @ 0x98 {
        7:0 command_result;
    }
    UIC_ARG3(u32) @ 0x9C {
        31:0 value;
    }
    UFS_MEM_CFG(u32) @ 0x300 {
        1:1 esi_enable => bool;
        0:0 mcq_mode_select => bool;
    }
    UFS_MCQ_CFG(u32) @ 0x380 {
        16:8 max_active_cmds;
    }
    UFS_ESILBA(u32) @ 0x384 {
        31:0 value;
    }
    UFS_ESIUBA(u32) @ 0x388 {
        31:0 value;
    }
}

struct McqQueueCfgBase;

register! {
    MCQ_SQATTR(u32) @ McqQueueCfgBase + 0x00 {
        31:31 enable => bool;
        23:16 cq_id;
        15:0 size;
    }
    MCQ_SQLBA(u32) @ McqQueueCfgBase + 0x04 { 31:0 value; }
    MCQ_SQUBA(u32) @ McqQueueCfgBase + 0x08 { 31:0 value; }
    MCQ_SQDAO(u32) @ McqQueueCfgBase + 0x0c { 31:0 value; }
    MCQ_SQISAO(u32) @ McqQueueCfgBase + 0x10 { 31:0 value; }
    MCQ_CQATTR(u32) @ McqQueueCfgBase + 0x20 {
        31:31 enable => bool;
        15:0 size;
    }
    MCQ_CQLBA(u32) @ McqQueueCfgBase + 0x24 { 31:0 value; }
    MCQ_CQUBA(u32) @ McqQueueCfgBase + 0x28 { 31:0 value; }
    MCQ_CQDAO(u32) @ McqQueueCfgBase + 0x2c { 31:0 value; }
    MCQ_CQISAO(u32) @ McqQueueCfgBase + 0x30 { 31:0 value; }
}

struct McqOprBase;

register! {
    MCQ_SQHP(u32) @ McqOprBase + 0x00 { 31:0 value; }
    MCQ_SQTP(u32) @ McqOprBase + 0x04 { 31:0 value; }
    MCQ_SQRTC(u32) @ McqOprBase + 0x08 {
        2:2 initiate_cleanup => bool;
        1:0 operation;
    }
    MCQ_SQCTI(u32) @ McqOprBase + 0x0c {
        15:8 lun;
        7:0 task_tag;
    }
    MCQ_SQRTS(u32) @ McqOprBase + 0x10 {
        7:4 cleanup_error_code;
        1:1 cleanup_in_progress => bool;
        0:0 stopped => bool;
    }
    MCQ_CQHP(u32) @ McqOprBase + 0x00 { 31:0 value; }
    MCQ_CQTP(u32) @ McqOprBase + 0x04 { 31:0 value; }
    MCQ_CQIS(u32) @ McqOprBase + 0x00 {
        0:0 tail_entry_pushed => bool;
    }
    MCQ_CQIE(u32) @ McqOprBase + 0x04 {
        0:0 tail_entry_push_enable => bool;
    }
}

const MCQ_QCFG_STRIDE: usize = 0x40;
const MCQ_QCFGPTR_UNIT: usize = 0x200;
const MCQ_ENTRY_SIZE_IN_DWORD: u32 = 8;
const MCQ_DEFAULT_OPR_STRIDE: usize = 48;
const MCQ_POLL_INTERVAL_US: i64 = 20;
const MCQ_POLL_TIMEOUT_US: i64 = 500000;

const MCQ_SQ_START: u32 = 0x0;
const MCQ_SQ_STOP: u32 = 0x1;

// IS - Interrupt Status
const UTP_TRANSFER_REQ_COMPL: u32 = 0x00000001;
const UIC_DME_END_PT_RESET: u32 = 0x00000002;
const UIC_ERROR: u32 = 0x00000004;
const UIC_TEST_MODE: u32 = 0x00000008;
const UIC_POWER_MODE: u32 = 0x00000010;
const UIC_HIBERNATE_EXIT: u32 = 0x00000020;
const UIC_HIBERNATE_ENTER: u32 = 0x00000040;
const UIC_LINK_LOST: u32 = 0x00000080;
const UIC_LINK_STARTUP: u32 = 0x00000100;
const UTP_TASK_REQ_COMPL: u32 = 0x00000200;
const UIC_COMMAND_COMPL: u32 = 0x00000400;
const DEVICE_FATAL_ERROR: u32 = 0x00000800;
const UTP_ERROR: u32 = 0x00001000;
const CONTROLLER_FATAL_ERROR: u32 = 0x00010000;
const SYSTEM_BUS_FATAL_ERROR: u32 = 0x00020000;
const CRYPTO_ENGINE_FATAL_ERROR: u32 = 0x00040000;
const MCQ_CQ_EVENT_STATUS: u32 = 0x00100000;

const UIC_INTR_HIBERNATE_MASK: u32 = UIC_HIBERNATE_EXIT | UIC_HIBERNATE_ENTER;
const UIC_INTR_POWER_MASK: u32 = UIC_POWER_MODE | UIC_INTR_HIBERNATE_MASK;
const UIC_INTR_MASK: u32 = UIC_INTR_POWER_MASK | UIC_COMMAND_COMPL;

const UTP_REQ_COMPL_MASK: u32 = UTP_TRANSFER_REQ_COMPL;
const ERROR_MASK: u32 = UIC_ERROR
    | UIC_LINK_LOST
    | DEVICE_FATAL_ERROR
    | CONTROLLER_FATAL_ERROR
    | SYSTEM_BUS_FATAL_ERROR
    | CRYPTO_ENGINE_FATAL_ERROR
    | UTP_ERROR;

const INT_AGGR_STATUS_BIT: u32 = 1 << 20;
const INT_AGGR_ENABLE: u32 = 1 << 31;
const UIC_ERROR_FLAG: u32 = 1 << 31;
const UIC_DL_PA_INIT_ERROR: u32 = 1 << 13;
const UIC_NL_ERROR_CODE_MASK: u32 = 0x7;
const UIC_TL_ERROR_CODE_MASK: u32 = 0x7f;
const UIC_DME_ERROR_CODE_MASK: u32 = 0x1;

#[derive(Copy, Clone, Debug)]
pub(crate) struct UicErrorStatus {
    pub(crate) phy: u32,
    pub(crate) data_link: u32,
    pub(crate) network: u32,
    pub(crate) transport: u32,
    pub(crate) dme: u32,
}

impl UicErrorStatus {
    pub(crate) fn requires_recovery(&self) -> bool {
        self.data_link & (UIC_ERROR_FLAG | UIC_DL_PA_INIT_ERROR)
            == UIC_ERROR_FLAG | UIC_DL_PA_INIT_ERROR
            || self.network & (UIC_ERROR_FLAG | UIC_NL_ERROR_CODE_MASK) > UIC_ERROR_FLAG
            || self.transport & (UIC_ERROR_FLAG | UIC_TL_ERROR_CODE_MASK) > UIC_ERROR_FLAG
            || self.dme & (UIC_ERROR_FLAG | UIC_DME_ERROR_CODE_MASK) > UIC_ERROR_FLAG
    }
}

pub(crate) enum PowerMode {
    OK = 0x00,
    Local = 0x01,
    Remote = 0x02,
    Busy = 0x03,
    ErrorCap = 0x04,
    FatalError = 0x05,
}

#[derive(Clone, Copy)]
pub(crate) enum McqRegisterRegion {
    Hci,
    Mcq,
}

#[derive(Clone, Copy)]
pub(crate) enum UfsMcqOprRegion {
    Sqd,
    Sqis,
    Cqd,
    Cqis,
}

#[derive(Clone, Copy)]
pub(crate) struct UfsMcqOprInfo {
    region: McqRegisterRegion,
    register_offset: usize,
    config_offset: usize,
    stride: usize,
}

impl UfsMcqOprInfo {
    pub(crate) fn new(
        region: McqRegisterRegion,
        register_offset: usize,
        config_offset: usize,
        stride: usize,
    ) -> Self {
        Self {
            region,
            register_offset,
            config_offset,
            stride,
        }
    }

    fn register_offset(&self, queue: usize) -> usize {
        self.register_offset + self.stride * queue
    }

    fn config_offset(&self, queue: usize) -> usize {
        self.config_offset + self.stride * queue
    }
}

#[derive(Clone, Copy)]
pub(crate) struct UfsMcqOprSet {
    sqd: UfsMcqOprInfo,
    sqis: UfsMcqOprInfo,
    cqd: UfsMcqOprInfo,
    cqis: UfsMcqOprInfo,
}

impl UfsMcqOprSet {
    pub(crate) fn new(
        sqd: UfsMcqOprInfo,
        sqis: UfsMcqOprInfo,
        cqd: UfsMcqOprInfo,
        cqis: UfsMcqOprInfo,
    ) -> Self {
        Self {
            sqd,
            sqis,
            cqd,
            cqis,
        }
    }

    fn get(&self, region: UfsMcqOprRegion) -> UfsMcqOprInfo {
        match region {
            UfsMcqOprRegion::Sqd => self.sqd,
            UfsMcqOprRegion::Sqis => self.sqis,
            UfsMcqOprRegion::Cqd => self.cqd,
            UfsMcqOprRegion::Cqis => self.cqis,
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct McqQueueConfigLayout {
    region: McqRegisterRegion,
    offset: usize,
    stride: usize,
}

impl McqQueueConfigLayout {
    pub(crate) fn new(
        region: McqRegisterRegion,
        offset: usize,
        stride: usize,
    ) -> Self {
        Self {
            region,
            offset,
            stride,
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct McqRegisterLayout {
    queue_config: McqQueueConfigLayout,
    oprs: UfsMcqOprSet,
}

impl McqRegisterLayout {
    pub(crate) fn new(
        queue_config: McqQueueConfigLayout,
        oprs: UfsMcqOprSet,
    ) -> Self {
        Self { queue_config, oprs }
    }

    pub(crate) fn oprs(&self) -> UfsMcqOprSet {
        self.oprs
    }
}

pub(crate) struct UfsReg {
    resources: Arc<HostResources>,
}

impl UfsReg {
    pub(crate) fn new(resources: Arc<HostResources>) -> Result<Arc<Self>> {
        Ok(Arc::new(Self { resources }, GFP_KERNEL)?)
    }

    pub(crate) fn hci_access(&self) -> Result<crate::resource::HciMmioAccess<'_>> {
        self.resources.hci_access()
    }

    #[inline(always)]
    fn dma_addr_lo(dma_addr: u64) -> u32 {
        dma_addr as u32
    }

    #[inline(always)]
    fn dma_addr_hi(dma_addr: u64) -> u32 {
        (dma_addr >> 32) as u32
    }

    // Basic Controller/Version/Interrupt
    #[inline]
    pub(crate) fn read_cap_lo(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(CONTROLLER_CAPABILITIES).into_raw()
    }

    #[inline]
    pub(crate) fn read_cap_hi(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(CONTROLLER_CAPABILITIES_H).value().get()
    }

    #[inline]
    pub(crate) fn read_version(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UFS_VERSION).value().get()
    }

    #[inline]
    pub(crate) fn read_is(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(INTERRUPT_STATUS).value().get()
    }

    #[inline]
    pub(crate) fn write_is(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(INTERRUPT_STATUS::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_ie(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(INTERRUPT_ENABLE).value().get()
    }

    #[inline]
    pub(crate) fn write_ie(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(INTERRUPT_ENABLE::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_hcs(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(CONTROLLER_STATUS).into_raw()
    }

    #[inline]
    pub(crate) fn read_hce(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(CONTROLLER_ENABLE_REG).into_raw()
    }

    #[inline]
    pub(crate) fn write_hce(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(CONTROLLER_ENABLE_REG::from_raw(value))
    }

    #[inline]
    pub(crate) fn read_uic_error_phy(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UIC_ERROR_CODE_PHY_ADAPTER_LAYER).value().get()
    }

    #[inline]
    pub(crate) fn confirm_uic_error(&self) {
        self.write_is(UIC_ERROR);
    }

    pub(crate) fn read_uic_errors(&self) -> UicErrorStatus {
        let access = self.resources.hci_access().unwrap();
        UicErrorStatus {
            phy: access
                .read(UIC_ERROR_CODE_PHY_ADAPTER_LAYER)
                .value()
                .get(),
            data_link: access
                .read(UIC_ERROR_CODE_DATA_LINK_LAYER)
                .value()
                .get(),
            network: access.read(UIC_ERROR_CODE_NETWORK_LAYER).value().get(),
            transport: access
                .read(UIC_ERROR_CODE_TRANSPORT_LAYER)
                .value()
                .get(),
            dme: access.read(UIC_ERROR_CODE_DME).value().get(),
        }
    }

    #[inline]
    pub(crate) fn write_uic_error_phy(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UIC_ERROR_CODE_PHY_ADAPTER_LAYER::zeroed().with_value(value))
    }

    // UTRL(Transfer)
    #[inline]
    pub(crate) fn write_utrlba(&self, low: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_BASE_L::zeroed().with_value(low))
    }

    #[inline]
    pub(crate) fn write_utrlbau(&self, high: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_BASE_H::zeroed().with_value(high))
    }

    #[inline]
    pub(crate) fn read_utrl_doorbell(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UTP_TRANSFER_REQ_DOOR_BELL).value().get()
    }

    #[inline]
    pub(crate) fn ring_utrl_doorbell(&self, tag: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_DOOR_BELL::zeroed().with_value(1u32 << tag))
    }

    #[inline]
    pub(crate) fn write_utrl_runstop(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_RUN_STOP::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn clear_utrl_slots(&self, mask: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_CLEAR::zeroed().with_value(mask))
    }

    // UTMRL(Task Management)
    #[inline]
    pub(crate) fn write_utmrlba(&self, low: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_BASE_L::zeroed().with_value(low))
    }

    #[inline]
    pub(crate) fn write_utmrlbau(&self, high: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_BASE_H::zeroed().with_value(high))
    }

    #[inline]
    pub(crate) fn read_utmrl_doorbell(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UTP_TASK_REQ_DOOR_BELL).value().get()
    }

    #[inline]
    pub(crate) fn ring_utmrl_doorbell(&self, mask: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TASK_REQ_DOOR_BELL::zeroed().with_value(mask))
    }

    #[inline]
    pub(crate) fn write_utmrl_runstop(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_RUN_STOP::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn clear_utmrl_slots(&self, mask: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_CLEAR::zeroed().with_value(mask))
    }

    // UIC command
    #[inline]
    pub(crate) fn read_uic_cmd(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UIC_COMMAND).value().get()
    }

    #[inline]
    pub(crate) fn write_uic_cmd(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UIC_COMMAND::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_uic_arg1(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UIC_ARG1).value().get()
    }

    #[inline]
    pub(crate) fn write_uic_arg1(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UIC_ARG1::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_uic_arg2(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UIC_ARG2).into_raw()
    }

    #[inline]
    pub(crate) fn write_uic_arg2(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UIC_ARG2::from_raw(value))
    }

    #[inline]
    pub(crate) fn read_uic_arg3(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UIC_ARG3).value().get()
    }

    #[inline]
    pub(crate) fn write_uic_arg3(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UIC_ARG3::zeroed().with_value(value))
    }

    // MCQ global configuration
    #[inline]
    pub(crate) fn read_mcq_cap(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(MCQCAP).into_raw()
    }

    #[inline]
    pub(crate) fn mcq_max_queues(&self) -> usize {
        let access = self.resources.hci_access().unwrap();
        access.read(MCQCAP).max_queue_supported().get() as usize + 1
    }

    #[inline]
    pub(crate) fn mcq_queue_cfg_base(&self) -> usize {
        let access = self.resources.hci_access().unwrap();
        access.read(MCQCAP).queue_config_pointer().get() as usize * MCQ_QCFGPTR_UNIT
    }

    pub(crate) fn mcq_register_layout(&self) -> Result<McqRegisterLayout> {
        self.resources.variant().mcq_register_layout(self)
    }

    pub(crate) fn standard_mcq_register_layout(&self) -> Result<McqRegisterLayout> {
        let queue_config = McqQueueConfigLayout::new(
            McqRegisterRegion::Hci,
            self.mcq_queue_cfg_base(),
            MCQ_QCFG_STRIDE,
        );
        let sqd = self.read_mcq_queue_cfg_at::<MCQ_SQDAO>(&queue_config, 0)?
            .value()
            .get() as usize;
        let sqis = self.read_mcq_queue_cfg_at::<MCQ_SQISAO>(&queue_config, 0)?
            .value()
            .get() as usize;
        let cqd = self.read_mcq_queue_cfg_at::<MCQ_CQDAO>(&queue_config, 0)?
            .value()
            .get() as usize;
        let cqis = self.read_mcq_queue_cfg_at::<MCQ_CQISAO>(&queue_config, 0)?
            .value()
            .get() as usize;

        Ok(McqRegisterLayout::new(
            queue_config,
            UfsMcqOprSet::new(
                UfsMcqOprInfo::new(
                    McqRegisterRegion::Hci,
                    sqd,
                    sqd,
                    MCQ_DEFAULT_OPR_STRIDE,
                ),
                UfsMcqOprInfo::new(
                    McqRegisterRegion::Hci,
                    sqis,
                    sqis,
                    MCQ_DEFAULT_OPR_STRIDE,
                ),
                UfsMcqOprInfo::new(
                    McqRegisterRegion::Hci,
                    cqd,
                    cqd,
                    MCQ_DEFAULT_OPR_STRIDE,
                ),
                UfsMcqOprInfo::new(
                    McqRegisterRegion::Hci,
                    cqis,
                    cqis,
                    MCQ_DEFAULT_OPR_STRIDE,
                ),
            ),
        ))
    }

    #[inline]
    pub(crate) fn read_ufs_mem_cfg(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UFS_MEM_CFG).into_raw()
    }

    #[inline]
    pub(crate) fn write_ufs_mem_cfg(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UFS_MEM_CFG::from_raw(value))
    }

    #[inline]
    pub(crate) fn enable_mcq_mode(&self) {
        let access = self.resources.hci_access().unwrap();
        access.update(UFS_MEM_CFG, |reg| reg.with_mcq_mode_select(true));
    }

    #[inline]
    pub(crate) fn disable_mcq_mode(&self) {
        let access = self.resources.hci_access().unwrap();
        access.update(UFS_MEM_CFG, |reg| reg.with_mcq_mode_select(false));
    }

    #[inline]
    pub(crate) fn enable_mcq_esi(&self) {
        let access = self.resources.hci_access().unwrap();
        access.update(UFS_MEM_CFG, |reg| reg.with_esi_enable(true));
    }

    #[inline]
    pub(crate) fn config_mcq_esi(&self, addr: u64) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UFS_ESILBA::zeroed().with_value(Self::dma_addr_lo(addr)));
        access.write_reg(UFS_ESIUBA::zeroed().with_value(Self::dma_addr_hi(addr)));
    }

    #[inline]
    pub(crate) fn read_mcq_cfg(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UFS_MCQ_CFG).into_raw()
    }

    #[inline]
    pub(crate) fn write_mcq_cfg(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UFS_MCQ_CFG::from_raw(value))
    }

    pub(crate) fn config_mcq_max_active_cmds(&self, max_active_cmds: u32) -> Result<()> {
        if max_active_cmds == 0 {
            return Err(EINVAL);
        }

        let access = self.resources.hci_access()?;
        let value = access
            .read(UFS_MCQ_CFG)
            .try_with_max_active_cmds(max_active_cmds - 1)?;
        access.write_reg(value);
        Ok(())
    }

    // MCQ queue configuration registers
    #[inline]
    fn read_mcq_region(&self, region: McqRegisterRegion, offset: usize) -> Result<u32> {
        match region {
            McqRegisterRegion::Hci => self.resources.hci_access()?.try_read32(offset),
            McqRegisterRegion::Mcq => self.resources.mcq_access()?.try_read32(offset),
        }
    }

    #[inline]
    fn write_mcq_region(
        &self,
        region: McqRegisterRegion,
        offset: usize,
        value: u32,
    ) -> Result<()> {
        match region {
            McqRegisterRegion::Hci => {
                self.resources.hci_access()?.try_write32(value, offset)
            }
            McqRegisterRegion::Mcq => {
                self.resources.mcq_access()?.try_write32(value, offset)
            }
        }
    }

    #[inline]
    fn mcq_queue_cfg_offset<T>(
        layout: &McqQueueConfigLayout,
        queue: usize,
    ) -> usize
    where
        T: Register<Storage = u32> + From<u32>,
        u32: From<T>,
    {
        layout.offset + layout.stride * queue + T::OFFSET
    }

    #[inline]
    fn read_mcq_queue_cfg_at<T>(
        &self,
        layout: &McqQueueConfigLayout,
        queue: usize,
    ) -> Result<T>
    where
        T: Register<Storage = u32> + From<u32>,
        u32: From<T>,
    {
        Ok(T::from(self.read_mcq_region(
            layout.region,
            Self::mcq_queue_cfg_offset::<T>(layout, queue),
        )?))
    }

    #[inline]
    fn write_mcq_queue_cfg<T>(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        value: T,
    ) -> Result<()>
    where
        T: Register<Storage = u32> + From<u32>,
        u32: From<T>,
    {
        self.write_mcq_region(
            layout.queue_config.region,
            Self::mcq_queue_cfg_offset::<T>(&layout.queue_config, queue),
            value.into(),
        )
    }

    fn mcq_queue_attr(max_entries: usize) -> Result<u32> {
        let dwords = u32::try_from(max_entries)
            .map_err(|_| EINVAL)?
            .checked_mul(MCQ_ENTRY_SIZE_IN_DWORD)
            .ok_or(EOVERFLOW)?;

        dwords.checked_sub(1).ok_or(EINVAL)
    }

    #[inline]
    pub(crate) fn write_mcq_sqlba(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        dma_addr: u64,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_SQLBA::zeroed().with_value(Self::dma_addr_lo(dma_addr)),
        )
    }

    #[inline]
    pub(crate) fn write_mcq_squba(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        dma_addr: u64,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_SQUBA::zeroed().with_value(Self::dma_addr_hi(dma_addr)),
        )
    }

    #[inline]
    pub(crate) fn set_mcq_sq_base_addr(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        dma_addr: u64,
    ) -> Result<()> {
        self.write_mcq_sqlba(layout, queue, dma_addr)?;
        self.write_mcq_squba(layout, queue, dma_addr)
    }

    #[inline]
    pub(crate) fn write_mcq_sqdao(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        offset: usize,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_SQDAO::zeroed().with_value(offset as u32),
        )
    }

    #[inline]
    pub(crate) fn write_mcq_sqisao(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        offset: usize,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_SQISAO::zeroed().with_value(offset as u32),
        )
    }

    #[inline]
    pub(crate) fn write_mcq_cqlba(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        dma_addr: u64,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_CQLBA::zeroed().with_value(Self::dma_addr_lo(dma_addr)),
        )
    }

    #[inline]
    pub(crate) fn write_mcq_cquba(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        dma_addr: u64,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_CQUBA::zeroed().with_value(Self::dma_addr_hi(dma_addr)),
        )
    }

    #[inline]
    pub(crate) fn set_mcq_cq_base_addr(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        dma_addr: u64,
    ) -> Result<()> {
        self.write_mcq_cqlba(layout, queue, dma_addr)?;
        self.write_mcq_cquba(layout, queue, dma_addr)
    }

    #[inline]
    pub(crate) fn write_mcq_cqdao(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        offset: usize,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_CQDAO::zeroed().with_value(offset as u32),
        )
    }

    #[inline]
    pub(crate) fn write_mcq_cqisao(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        offset: usize,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(
            layout,
            queue,
            MCQ_CQISAO::zeroed().with_value(offset as u32),
        )
    }

    pub(crate) fn enable_mcq_sq(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        max_entries: usize,
        cq_id: usize,
    ) -> Result<()> {
        let attr = MCQ_SQATTR::zeroed()
            .with_enable(true)
            .try_with_cq_id(u32::try_from(cq_id).map_err(|_| EINVAL)?)?
            .try_with_size(Self::mcq_queue_attr(max_entries)?)?;
        self.write_mcq_queue_cfg(layout, queue, attr)
    }

    pub(crate) fn enable_mcq_cq(
        &self,
        layout: &McqRegisterLayout,
        queue: usize,
        max_entries: usize,
    ) -> Result<()> {
        let attr = MCQ_CQATTR::zeroed()
            .with_enable(true)
            .try_with_size(Self::mcq_queue_attr(max_entries)?)?;
        self.write_mcq_queue_cfg(layout, queue, attr)
    }

    // MCQ operation and runtime registers
    #[inline]
    pub(crate) fn mcq_opr_region_offset(
        &self,
        oprs: &UfsMcqOprSet,
        region: UfsMcqOprRegion,
        queue: usize,
    ) -> usize {
        oprs.get(region).config_offset(queue)
    }

    #[inline]
    fn read_mcq_opr<T>(
        &self,
        oprs: &UfsMcqOprSet,
        region: UfsMcqOprRegion,
        queue: usize,
    ) -> Result<T>
    where
        T: Register<Storage = u32> + From<u32>,
        u32: From<T>,
    {
        let info = oprs.get(region);
        Ok(T::from(self.read_mcq_region(
            info.region,
            info.register_offset(queue) + T::OFFSET,
        )?))
    }

    #[inline]
    fn write_mcq_opr<T>(
        &self,
        oprs: &UfsMcqOprSet,
        region: UfsMcqOprRegion,
        queue: usize,
        value: T,
    ) -> Result<()>
    where
        T: Register<Storage = u32> + From<u32>,
        u32: From<T>,
    {
        let info = oprs.get(region);
        self.write_mcq_region(
            info.region,
            info.register_offset(queue) + T::OFFSET,
            value.into(),
        )
    }

    #[inline]
    pub(crate) fn read_mcq_sq_head(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        Ok(self
            .read_mcq_opr::<MCQ_SQHP>(oprs, UfsMcqOprRegion::Sqd, queue)?
            .value()
            .get())
    }

    #[inline]
    pub(crate) fn read_mcq_sq_tail(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        Ok(self
            .read_mcq_opr::<MCQ_SQTP>(oprs, UfsMcqOprRegion::Sqd, queue)?
            .value()
            .get())
    }

    #[inline]
    pub(crate) fn write_mcq_sq_tail(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        tail: u32,
    ) -> Result<()> {
        self.write_mcq_opr(
            oprs,
            UfsMcqOprRegion::Sqd,
            queue,
            MCQ_SQTP::zeroed().with_value(tail),
        )
    }

    #[inline]
    pub(crate) fn write_mcq_sq_runtime_control(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        value: u32,
    ) -> Result<()> {
        let value = MCQ_SQRTC::zeroed().try_with_operation(value)?;
        self.write_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, value)
    }

    #[inline]
    pub(crate) fn read_mcq_sq_runtime_status(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
    ) -> Result<u32> {
        Ok(self
            .read_mcq_opr::<MCQ_SQRTS>(oprs, UfsMcqOprRegion::Sqd, queue)?
            .into_raw())
    }

    fn wait_mcq_sq_stopped(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        stopped: bool,
    ) -> Result<()> {
        read_poll_timeout(
            || self.read_mcq_opr::<MCQ_SQRTS>(oprs, UfsMcqOprRegion::Sqd, queue),
            |v: &MCQ_SQRTS| v.stopped() == stopped,
            Delta::from_micros(MCQ_POLL_INTERVAL_US),
            Delta::from_micros(MCQ_POLL_TIMEOUT_US),
        )
        .map(|_| ())
    }

    pub(crate) fn stop_mcq_sq(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<()> {
        self.write_mcq_sq_runtime_control(oprs, queue, MCQ_SQ_STOP)?;
        self.wait_mcq_sq_stopped(oprs, queue, true)
    }

    pub(crate) fn start_mcq_sq(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<()> {
        self.write_mcq_sq_runtime_control(oprs, queue, MCQ_SQ_START)?;
        self.wait_mcq_sq_stopped(oprs, queue, false)
    }

    #[inline]
    pub(crate) fn write_mcq_sq_cleanup_target(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        lun: u8,
        tag: u8,
    ) -> Result<()> {
        let target = MCQ_SQCTI::zeroed()
            .with_lun(lun)
            .with_task_tag(tag);
        self.write_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, target)
    }

    pub(crate) fn initiate_mcq_sq_cleanup(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<()> {
        let rtc = self
            .read_mcq_opr::<MCQ_SQRTC>(oprs, UfsMcqOprRegion::Sqd, queue)?
            .with_initiate_cleanup(true);
        self.write_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, rtc)?;
        read_poll_timeout(
            || self.read_mcq_opr::<MCQ_SQRTS>(oprs, UfsMcqOprRegion::Sqd, queue),
            |v: &MCQ_SQRTS| v.cleanup_in_progress(),
            Delta::from_micros(MCQ_POLL_INTERVAL_US),
            Delta::from_micros(MCQ_POLL_TIMEOUT_US),
        )
        .map(|_| ())
    }

    #[inline]
    pub(crate) fn mcq_sq_cleanup_error_code(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
    ) -> Result<u32> {
        Ok(self
            .read_mcq_opr::<MCQ_SQRTS>(oprs, UfsMcqOprRegion::Sqd, queue)?
            .cleanup_error_code()
            .get())
    }

    #[inline]
    pub(crate) fn read_mcq_cq_head(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        Ok(self
            .read_mcq_opr::<MCQ_CQHP>(oprs, UfsMcqOprRegion::Cqd, queue)?
            .value()
            .get())
    }

    #[inline]
    pub(crate) fn write_mcq_cq_head(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        head: u32,
    ) -> Result<()> {
        self.write_mcq_opr(
            oprs,
            UfsMcqOprRegion::Cqd,
            queue,
            MCQ_CQHP::zeroed().with_value(head),
        )
    }

    #[inline]
    pub(crate) fn read_mcq_cq_tail(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        Ok(self
            .read_mcq_opr::<MCQ_CQTP>(oprs, UfsMcqOprRegion::Cqd, queue)?
            .value()
            .get())
    }

    #[inline]
    pub(crate) fn read_mcq_cqis(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        Ok(self
            .read_mcq_opr::<MCQ_CQIS>(oprs, UfsMcqOprRegion::Cqis, queue)?
            .into_raw())
    }

    #[inline]
    pub(crate) fn write_mcq_cqis(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        value: u32,
    ) -> Result<()> {
        self.write_mcq_opr(
            oprs,
            UfsMcqOprRegion::Cqis,
            queue,
            MCQ_CQIS::from_raw(value),
        )
    }

    #[inline]
    pub(crate) fn enable_mcq_cq_tail_push_intr(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
    ) -> Result<()> {
        self.write_mcq_opr(
            oprs,
            UfsMcqOprRegion::Cqis,
            queue,
            MCQ_CQIE::zeroed().with_tail_entry_push_enable(true),
        )
    }

    // Helpers
    #[inline]
    pub(crate) fn nutrs(&self) -> usize {
        let access = self.resources.hci_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES)
            .transfer_request_slots_sdb()
            .get() as usize
            + 1
    }

    #[inline]
    pub(crate) fn mcq_hardware_supported(&self) -> bool {
        let access = self.resources.hci_access().unwrap();
        access.read(CONTROLLER_CAPABILITIES).mcq_supported()
    }

    #[inline]
    pub(crate) fn mcq_variant_enabled(&self) -> bool {
        self.resources.variant().mcq_enabled()
    }

    #[inline]
    pub(crate) fn constrain_mcq_active_commands(&self, reported: usize) -> usize {
        self.resources
            .variant()
            .constrain_mcq_active_commands(reported)
    }

    #[inline]
    pub(crate) fn nutrs_mcq(&self) -> usize {
        let access = self.resources.hci_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES_MCQ)
            .transfer_request_slots()
            .get() as usize
            + 1
    }

    #[inline]
    pub(crate) fn nutmrs(&self) -> usize {
        let access = self.resources.hci_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES)
            .task_management_request_slots()
            .get() as usize
            + 1
    }

    #[inline]
    pub(crate) fn autoh8(&self) -> bool {
        let access = self.resources.hci_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES)
            .auto_hibern8_supported()
    }

    #[inline]
    pub(crate) fn ctrl_enable(&self) {
        let access = self.resources.hci_access().unwrap();
        access.update(CONTROLLER_ENABLE_REG, |reg| {
            reg.with_controller_enable(true)
        });
    }

    #[inline]
    pub(crate) fn ctrl_disable(&self) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(CONTROLLER_ENABLE_REG::zeroed());
    }

    #[inline]
    pub(crate) fn ctrl_enabled(&self) -> bool {
        let access = self.resources.hci_access().unwrap();
        access.read(CONTROLLER_ENABLE_REG).controller_enable()
    }

    #[inline]
    pub(crate) fn clear_all_interrupts(&self) {
        let isb = self.read_is();
        if isb != 0 {
            self.write_is(isb)
        }
    }

    #[inline]
    pub(crate) fn disable_interrupts(&self) {
        self.write_ie(0);
    }

    #[inline]
    pub(crate) fn set_utrdl_base(&self, dma_addr: u64) {
        self.write_utrlba(dma_addr as u32);
        self.write_utrlbau((dma_addr >> 32) as u32);
    }

    #[inline]
    pub(crate) fn set_utmrdl_base(&self, dma_addr: u64) {
        self.write_utmrlba(dma_addr as u32);
        self.write_utmrlbau((dma_addr >> 32) as u32);
    }

    #[inline]
    pub(crate) fn wait_for_ctrl_enable(&self, interval_us: i64, timeout_ms: i64) -> Result<()> {
        pr_info!("[RUFS] drivers/rufs/ufs_reg: wait_for_ctrl_enable");
        match read_poll_timeout(
            || {
                let access = self.resources.hci_access()?;
                Ok(access.read(CONTROLLER_ENABLE_REG))
            },
            |v: &CONTROLLER_ENABLE_REG| v.controller_enable(),
            Delta::from_micros(interval_us),
            Delta::from_millis(timeout_ms),
        ) {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }

    #[inline]
    pub(crate) fn wait_for_ctrl_disable(&self, interval_us: i64, timeout_ms: i64) -> Result<()> {
        match read_poll_timeout(
            || {
                let access = self.resources.hci_access()?;
                Ok(access.read(CONTROLLER_ENABLE_REG))
            },
            |v: &CONTROLLER_ENABLE_REG| !v.controller_enable(),
            Delta::from_micros(interval_us),
            Delta::from_millis(timeout_ms),
        ) {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }

    pub(crate) fn enable_uic_interrupts(&self) {
        self.write_ie(self.read_ie() | UIC_INTR_MASK);
    }

    pub(crate) fn read_uic_interrupts(&self) -> u32 {
        self.read_is() & self.read_ie() & UIC_INTR_MASK
    }

    pub(crate) fn uic_interrupts(value: u32) -> u32 {
        value & UIC_INTR_MASK
    }

    pub(crate) fn confirm_uic_interrupts(&self, value: u32) {
        self.write_is(value & UIC_INTR_MASK);
    }

    pub(crate) fn get_uic_cmd_result(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access.read(UIC_ARG2).command_result().get()
    }

    pub(crate) fn get_dme_attr_val(&self) -> u32 {
        self.read_uic_arg3()
    }

    pub(crate) fn get_power_mode_change_status(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access
            .read(CONTROLLER_STATUS)
            .power_mode_change_request_status()
            .get()
    }

    pub(crate) fn wait_for_uic_cmd_ready(&self, interval_us: i64, timeout_ms: i64) -> Result<()> {
        match read_poll_timeout(
            || {
                let access = self.resources.hci_access()?;
                Ok(access.read(CONTROLLER_STATUS))
            },
            |v: &CONTROLLER_STATUS| v.uic_command_ready(),
            Delta::from_micros(interval_us),
            Delta::from_millis(timeout_ms),
        ) {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }

    pub(crate) fn read_transfer_interrupts(&self) -> u32 {
        self.read_is()
            & self.read_ie()
            & (UTP_TRANSFER_REQ_COMPL | MCQ_CQ_EVENT_STATUS | ERROR_MASK)
    }

    pub(crate) fn transfer_interrupts(value: u32) -> u32 {
        value & (UTP_TRANSFER_REQ_COMPL | MCQ_CQ_EVENT_STATUS | ERROR_MASK)
    }

    pub(crate) fn confirm_transfer_interrupts(&self, value: u32) {
        self.write_is(value & (UTP_TRANSFER_REQ_COMPL | MCQ_CQ_EVENT_STATUS | ERROR_MASK));
    }

    pub(crate) fn enable_transfer_interrupts(&self) {
        self.write_ie(self.read_ie() | UTP_REQ_COMPL_MASK | ERROR_MASK);
    }

    pub(crate) fn disable_transfer_req_int_aggr(&self) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_INT_AGG_CONTROL::zeroed().with_value(0 as u32));
        let value = access.read(UTP_TRANSFER_REQ_INT_AGG_CONTROL).value().get();
        let int_enable = (value & INT_AGGR_ENABLE) != 0;
        let int_status = (value & INT_AGGR_STATUS_BIT) != 0;

        pr_info!(
            "[RUFS] ufs_reg: transfer request interrupt aggregation raw=0x{:08x} enabled={} status={}\n",
            value,
            int_enable,
            int_status,
        );
    }

    pub(crate) fn enable_mcq_interrupts(&self) {
        self.write_ie(self.read_ie() | MCQ_CQ_EVENT_STATUS);
    }

    pub(crate) fn wait_for_request_ready(&self, interval_us: i64, timeout_ms: i64) -> Result<()> {
        match read_poll_timeout(
            || {
                let access = self.resources.hci_access()?;
                Ok(access.read(CONTROLLER_STATUS))
            },
            |v: &CONTROLLER_STATUS| {
                v.transfer_request_list_ready()
                    && v.task_request_list_ready()
                    && v.uic_command_ready()
            },
            Delta::from_micros(interval_us),
            Delta::from_millis(timeout_ms),
        ) {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }

    pub(crate) fn enable_run_stop(&self) {
        self.write_utmrl_runstop(1);
        self.write_utrl_runstop(1);
    }

    pub(crate) fn disable_run_stop(&self) {
        self.write_utrl_runstop(0);
        self.write_utmrl_runstop(0);
    }

    pub(crate) fn utrlcnr(&self) -> u32 {
        let access = self.resources.hci_access().unwrap();
        access
            .read(UTP_TRANSFER_REQ_LIST_COMPLETION_NOTIFICATION)
            .value()
            .get()
    }

    pub(crate) fn write_utrlcnr(&self, value: u32) {
        let access = self.resources.hci_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_COMPLETION_NOTIFICATION::zeroed().with_value(value))
    }
}

#[inline]
pub(crate) fn is_uic_command_completion(interrupt_status: u32) -> bool {
    (interrupt_status & UIC_COMMAND_COMPL) != 0
}

#[inline]
pub(crate) fn is_uic_power_mode(interrupt_status: u32) -> bool {
    (interrupt_status & UIC_POWER_MODE) != 0
}

#[inline]
pub(crate) fn is_error_interrupt(interrupt_status: u32) -> bool {
    (interrupt_status & ERROR_MASK) != 0
}

#[inline]
pub(crate) fn is_uic_error_interrupt(interrupt_status: u32) -> bool {
    (interrupt_status & UIC_ERROR) != 0
}

#[inline]
pub(crate) fn is_transfer_recovery_interrupt(interrupt_status: u32) -> bool {
    (interrupt_status & (ERROR_MASK & !UIC_ERROR)) != 0
}
