// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::io::{poll::read_poll_timeout, register, Io};
use kernel::time::Delta;
use kernel::{c_str, device::Core, devres::Devres, pci, prelude::*, sync::Arc};

const UFS_BAR0_LEN: usize = 0x1000;
type Bar0 = pci::Bar<'static, UFS_BAR0_LEN>;

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

// MCQ queue configuration registers. These offsets are relative to each
// queue's 0x40-byte config block.
const REG_MCQ_SQATTR: usize = 0x00;
const REG_MCQ_SQLBA: usize = 0x04;
const REG_MCQ_SQUBA: usize = 0x08;
const REG_MCQ_SQDAO: usize = 0x0C;
const REG_MCQ_SQISAO: usize = 0x10;
const REG_MCQ_CQATTR: usize = 0x20;
const REG_MCQ_CQLBA: usize = 0x24;
const REG_MCQ_CQUBA: usize = 0x28;
const REG_MCQ_CQDAO: usize = 0x2C;
const REG_MCQ_CQISAO: usize = 0x30;

// MCQ operation/runtime registers. These offsets are relative to each SQD,
// SQIS, CQD, or CQIS operation region.
const REG_MCQ_SQHP: usize = 0x00;
const REG_MCQ_SQTP: usize = 0x04;
const REG_MCQ_SQRTC: usize = 0x08;
const REG_MCQ_SQCTI: usize = 0x0C;
const REG_MCQ_SQRTS: usize = 0x10;
const REG_MCQ_CQHP: usize = 0x00;
const REG_MCQ_CQTP: usize = 0x04;
const REG_MCQ_CQIS: usize = 0x00;
const REG_MCQ_CQIE: usize = 0x04;

const MCQ_QCFG_STRIDE: usize = 0x40;
const MCQ_QCFGPTR_UNIT: usize = 0x200;
const MCQ_ENTRY_SIZE_IN_DWORD: u32 = 8;
const MCQ_QUEUE_EN: u32 = 1 << 31;
const MCQ_QUEUE_ID_SHIFT: u32 = 16;
const MCQ_DEFAULT_OPR_STRIDE: usize = 48;
const MCQ_POLL_INTERVAL_US: i64 = 20;
const MCQ_POLL_TIMEOUT_US: i64 = 500000;

const MCQ_CQIS_TAIL_ENT_PUSH_STS: u32 = 0x1;

const MCQ_SQ_START: u32 = 0x0;
const MCQ_SQ_STOP: u32 = 0x1;
const MCQ_SQ_ICU: u32 = 0x2;
const MCQ_SQ_STS: u32 = 0x1;
const MCQ_SQ_CUS: u32 = 0x2;
const MASK_MCQ_SQ_ICU_ERR_CODE: u32 = 0xF0;
const SHIFT_MCQ_SQ_ICU_ERR_CODE: u32 = 4;

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

#[derive(Copy, Clone)]
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
pub(crate) enum UfsMcqOprRegion {
    Sqd,
    Sqis,
    Cqd,
    Cqis,
}

#[derive(Clone, Copy, Default)]
pub(crate) struct UfsMcqOprInfo {
    offset: usize,
    stride: usize,
}

impl UfsMcqOprInfo {
    pub(crate) fn new(offset: usize, stride: usize) -> Self {
        Self { offset, stride }
    }

    pub(crate) fn offset(&self) -> usize {
        self.offset
    }

    pub(crate) fn stride(&self) -> usize {
        self.stride
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

pub(crate) struct UfsReg {
    bar: Devres<Bar0>,
}

impl UfsReg {
    pub(crate) fn new(pdev: &pci::Device<Core<'_>>) -> Result<Arc<Self>> {
        Ok(Arc::new(
            Self {
                bar: pdev
                    .iomap_region_sized::<UFS_BAR0_LEN>(0, c_str!("rufs_pci"))?
                    .into_devres()?,
            },
            GFP_KERNEL,
        )?)
    }

    #[inline(always)]
    fn try_read(&self, offset: usize) -> Result<u32> {
        let access = self.bar.try_access().ok_or(ENODEV)?;
        access.try_read32(offset)
    }

    #[inline(always)]
    fn try_write(&self, offset: usize, value: u32) -> Result<()> {
        let access = self.bar.try_access().ok_or(ENODEV)?;
        access.try_write32(value, offset)
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
        let access = self.bar.try_access().unwrap();
        access.read(CONTROLLER_CAPABILITIES).into_raw()
    }

    #[inline]
    pub(crate) fn read_cap_hi(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(CONTROLLER_CAPABILITIES_H).value().get()
    }

    #[inline]
    pub(crate) fn read_version(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UFS_VERSION).value().get()
    }

    #[inline]
    pub(crate) fn read_is(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(INTERRUPT_STATUS).value().get()
    }

    #[inline]
    pub(crate) fn write_is(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(INTERRUPT_STATUS::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_ie(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(INTERRUPT_ENABLE).value().get()
    }

    #[inline]
    pub(crate) fn write_ie(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(INTERRUPT_ENABLE::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_hcs(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(CONTROLLER_STATUS).into_raw()
    }

    #[inline]
    pub(crate) fn read_hce(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(CONTROLLER_ENABLE_REG).into_raw()
    }

    #[inline]
    pub(crate) fn write_hce(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(CONTROLLER_ENABLE_REG::from_raw(value))
    }

    #[inline]
    pub(crate) fn read_uic_error_phy(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UIC_ERROR_CODE_PHY_ADAPTER_LAYER).value().get()
    }

    #[inline]
    pub(crate) fn confirm_uic_error(&self) {
        self.write_is(UIC_ERROR);
    }

    pub(crate) fn read_uic_errors(&self) -> UicErrorStatus {
        let access = self.bar.try_access().unwrap();
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
        let access = self.bar.try_access().unwrap();
        access.write_reg(UIC_ERROR_CODE_PHY_ADAPTER_LAYER::zeroed().with_value(value))
    }

    // UTRL(Transfer)
    #[inline]
    pub(crate) fn write_utrlba(&self, low: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_BASE_L::zeroed().with_value(low))
    }

    #[inline]
    pub(crate) fn write_utrlbau(&self, high: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_BASE_H::zeroed().with_value(high))
    }

    #[inline]
    pub(crate) fn read_utrl_doorbell(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UTP_TRANSFER_REQ_DOOR_BELL).value().get()
    }

    #[inline]
    pub(crate) fn ring_utrl_doorbell(&self, tag: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_DOOR_BELL::zeroed().with_value(1u32 << tag))
    }

    #[inline]
    pub(crate) fn write_utrl_runstop(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_RUN_STOP::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn clear_utrl_slots(&self, mask: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TRANSFER_REQ_LIST_CLEAR::zeroed().with_value(mask))
    }

    // UTMRL(Task Management)
    #[inline]
    pub(crate) fn write_utmrlba(&self, low: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_BASE_L::zeroed().with_value(low))
    }

    #[inline]
    pub(crate) fn write_utmrlbau(&self, high: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_BASE_H::zeroed().with_value(high))
    }

    #[inline]
    pub(crate) fn read_utmrl_doorbell(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UTP_TASK_REQ_DOOR_BELL).value().get()
    }

    #[inline]
    pub(crate) fn ring_utmrl_doorbell(&self, mask: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TASK_REQ_DOOR_BELL::zeroed().with_value(mask))
    }

    #[inline]
    pub(crate) fn write_utmrl_runstop(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_RUN_STOP::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn clear_utmrl_slots(&self, mask: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UTP_TASK_REQ_LIST_CLEAR::zeroed().with_value(mask))
    }

    // UIC command
    #[inline]
    pub(crate) fn read_uic_cmd(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UIC_COMMAND).value().get()
    }

    #[inline]
    pub(crate) fn write_uic_cmd(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UIC_COMMAND::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_uic_arg1(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UIC_ARG1).value().get()
    }

    #[inline]
    pub(crate) fn write_uic_arg1(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UIC_ARG1::zeroed().with_value(value))
    }

    #[inline]
    pub(crate) fn read_uic_arg2(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UIC_ARG2).into_raw()
    }

    #[inline]
    pub(crate) fn write_uic_arg2(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UIC_ARG2::from_raw(value))
    }

    #[inline]
    pub(crate) fn read_uic_arg3(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UIC_ARG3).value().get()
    }

    #[inline]
    pub(crate) fn write_uic_arg3(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UIC_ARG3::zeroed().with_value(value))
    }

    // MCQ global configuration
    #[inline]
    pub(crate) fn read_mcq_cap(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(MCQCAP).into_raw()
    }

    #[inline]
    pub(crate) fn mcq_max_queues(&self) -> usize {
        let access = self.bar.try_access().unwrap();
        access.read(MCQCAP).max_queue_supported().get() as usize + 1
    }

    #[inline]
    pub(crate) fn mcq_queue_cfg_base(&self) -> usize {
        let access = self.bar.try_access().unwrap();
        access.read(MCQCAP).queue_config_pointer().get() as usize * MCQ_QCFGPTR_UNIT
    }

    #[inline]
    pub(crate) fn read_ufs_mem_cfg(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UFS_MEM_CFG).into_raw()
    }

    #[inline]
    pub(crate) fn write_ufs_mem_cfg(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UFS_MEM_CFG::from_raw(value))
    }

    #[inline]
    pub(crate) fn enable_mcq_mode(&self) {
        let access = self.bar.try_access().unwrap();
        access.update(UFS_MEM_CFG, |reg| reg.with_mcq_mode_select(true));
    }

    #[inline]
    pub(crate) fn disable_mcq_mode(&self) {
        let access = self.bar.try_access().unwrap();
        access.update(UFS_MEM_CFG, |reg| reg.with_mcq_mode_select(false));
    }

    #[inline]
    pub(crate) fn enable_mcq_esi(&self) {
        let access = self.bar.try_access().unwrap();
        access.update(UFS_MEM_CFG, |reg| reg.with_esi_enable(true));
    }

    #[inline]
    pub(crate) fn config_mcq_esi(&self, addr: u64) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UFS_ESILBA::zeroed().with_value(Self::dma_addr_lo(addr)));
        access.write_reg(UFS_ESIUBA::zeroed().with_value(Self::dma_addr_hi(addr)));
    }

    #[inline]
    pub(crate) fn read_mcq_cfg(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UFS_MCQ_CFG).into_raw()
    }

    #[inline]
    pub(crate) fn write_mcq_cfg(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(UFS_MCQ_CFG::from_raw(value))
    }

    pub(crate) fn config_mcq_max_active_cmds(&self, max_active_cmds: u32) -> Result<()> {
        if max_active_cmds == 0 {
            return Err(EINVAL);
        }

        let access = self.bar.try_access().ok_or(ENODEV)?;
        let value = access
            .read(UFS_MCQ_CFG)
            .try_with_max_active_cmds(max_active_cmds - 1)?;
        access.write_reg(value);
        Ok(())
    }

    // MCQ queue configuration registers
    #[inline]
    pub(crate) fn mcq_queue_cfg_offset(&self, queue: usize, reg: usize) -> usize {
        self.mcq_queue_cfg_base() + MCQ_QCFG_STRIDE * queue + reg
    }

    #[inline]
    pub(crate) fn read_mcq_queue_cfg(&self, queue: usize, reg: usize) -> Result<u32> {
        self.try_read(self.mcq_queue_cfg_offset(queue, reg))
    }

    #[inline]
    pub(crate) fn write_mcq_queue_cfg(&self, queue: usize, reg: usize, value: u32) -> Result<()> {
        self.try_write(self.mcq_queue_cfg_offset(queue, reg), value)
    }

    #[inline]
    fn write_mcq_queue_dma_addr(
        &self,
        queue: usize,
        low_reg: usize,
        high_reg: usize,
        dma_addr: u64,
    ) -> Result<()> {
        self.write_mcq_queue_cfg(queue, low_reg, Self::dma_addr_lo(dma_addr))?;
        self.write_mcq_queue_cfg(queue, high_reg, Self::dma_addr_hi(dma_addr))
    }

    fn mcq_queue_attr(max_entries: usize) -> Result<u32> {
        let dwords = u32::try_from(max_entries)
            .map_err(|_| EINVAL)?
            .checked_mul(MCQ_ENTRY_SIZE_IN_DWORD)
            .ok_or(EOVERFLOW)?;

        Ok(MCQ_QUEUE_EN | dwords.checked_sub(1).ok_or(EINVAL)?)
    }

    #[inline]
    pub(crate) fn write_mcq_sqlba(&self, queue: usize, dma_addr: u64) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_SQLBA, Self::dma_addr_lo(dma_addr))
    }

    #[inline]
    pub(crate) fn write_mcq_squba(&self, queue: usize, dma_addr: u64) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_SQUBA, Self::dma_addr_hi(dma_addr))
    }

    #[inline]
    pub(crate) fn set_mcq_sq_base_addr(&self, queue: usize, dma_addr: u64) -> Result<()> {
        self.write_mcq_queue_dma_addr(queue, REG_MCQ_SQLBA, REG_MCQ_SQUBA, dma_addr)
    }

    #[inline]
    pub(crate) fn write_mcq_sqdao(&self, queue: usize, offset: usize) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_SQDAO, offset as u32)
    }

    #[inline]
    pub(crate) fn write_mcq_sqisao(&self, queue: usize, offset: usize) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_SQISAO, offset as u32)
    }

    #[inline]
    pub(crate) fn write_mcq_cqlba(&self, queue: usize, dma_addr: u64) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_CQLBA, Self::dma_addr_lo(dma_addr))
    }

    #[inline]
    pub(crate) fn write_mcq_cquba(&self, queue: usize, dma_addr: u64) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_CQUBA, Self::dma_addr_hi(dma_addr))
    }

    #[inline]
    pub(crate) fn set_mcq_cq_base_addr(&self, queue: usize, dma_addr: u64) -> Result<()> {
        self.write_mcq_queue_dma_addr(queue, REG_MCQ_CQLBA, REG_MCQ_CQUBA, dma_addr)
    }

    #[inline]
    pub(crate) fn write_mcq_cqdao(&self, queue: usize, offset: usize) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_CQDAO, offset as u32)
    }

    #[inline]
    pub(crate) fn write_mcq_cqisao(&self, queue: usize, offset: usize) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_CQISAO, offset as u32)
    }

    pub(crate) fn enable_mcq_sq(
        &self,
        queue: usize,
        max_entries: usize,
        cq_id: usize,
    ) -> Result<()> {
        let attr = Self::mcq_queue_attr(max_entries)?
            | ((u32::try_from(cq_id).map_err(|_| EINVAL)?) << MCQ_QUEUE_ID_SHIFT);
        self.write_mcq_queue_cfg(queue, REG_MCQ_SQATTR, attr)
    }

    pub(crate) fn enable_mcq_cq(&self, queue: usize, max_entries: usize) -> Result<()> {
        self.write_mcq_queue_cfg(queue, REG_MCQ_CQATTR, Self::mcq_queue_attr(max_entries)?)
    }

    pub(crate) fn mcq_default_opr_set(&self) -> Result<UfsMcqOprSet> {
        Ok(UfsMcqOprSet::new(
            UfsMcqOprInfo::new(
                self.read_mcq_queue_cfg(0, REG_MCQ_SQDAO)? as usize,
                MCQ_DEFAULT_OPR_STRIDE,
            ),
            UfsMcqOprInfo::new(
                self.read_mcq_queue_cfg(0, REG_MCQ_SQISAO)? as usize,
                MCQ_DEFAULT_OPR_STRIDE,
            ),
            UfsMcqOprInfo::new(
                self.read_mcq_queue_cfg(0, REG_MCQ_CQDAO)? as usize,
                MCQ_DEFAULT_OPR_STRIDE,
            ),
            UfsMcqOprInfo::new(
                self.read_mcq_queue_cfg(0, REG_MCQ_CQISAO)? as usize,
                MCQ_DEFAULT_OPR_STRIDE,
            ),
        ))
    }

    // MCQ operation and runtime registers
    #[inline]
    pub(crate) fn mcq_opr_offset(
        &self,
        oprs: &UfsMcqOprSet,
        region: UfsMcqOprRegion,
        queue: usize,
        reg: usize,
    ) -> usize {
        let info = oprs.get(region);
        info.offset + info.stride * queue + reg
    }

    #[inline]
    pub(crate) fn read_mcq_opr(
        &self,
        oprs: &UfsMcqOprSet,
        region: UfsMcqOprRegion,
        queue: usize,
        reg: usize,
    ) -> Result<u32> {
        self.try_read(self.mcq_opr_offset(oprs, region, queue, reg))
    }

    #[inline]
    pub(crate) fn write_mcq_opr(
        &self,
        oprs: &UfsMcqOprSet,
        region: UfsMcqOprRegion,
        queue: usize,
        reg: usize,
        value: u32,
    ) -> Result<()> {
        self.try_write(self.mcq_opr_offset(oprs, region, queue, reg), value)
    }

    #[inline]
    pub(crate) fn read_mcq_sq_head(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        self.read_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, REG_MCQ_SQHP)
    }

    #[inline]
    pub(crate) fn read_mcq_sq_tail(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        self.read_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, REG_MCQ_SQTP)
    }

    #[inline]
    pub(crate) fn write_mcq_sq_tail(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        tail: u32,
    ) -> Result<()> {
        self.write_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, REG_MCQ_SQTP, tail)
    }

    #[inline]
    pub(crate) fn write_mcq_sq_runtime_control(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        value: u32,
    ) -> Result<()> {
        self.write_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, REG_MCQ_SQRTC, value)
    }

    #[inline]
    pub(crate) fn read_mcq_sq_runtime_status(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
    ) -> Result<u32> {
        self.read_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, REG_MCQ_SQRTS)
    }

    fn wait_mcq_sq_status(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        mask: u32,
        set: bool,
    ) -> Result<()> {
        read_poll_timeout(
            || self.read_mcq_sq_runtime_status(oprs, queue),
            |v: &u32| ((*v & mask) != 0) == set,
            Delta::from_micros(MCQ_POLL_INTERVAL_US),
            Delta::from_micros(MCQ_POLL_TIMEOUT_US),
        )
        .map(|_| ())
    }

    pub(crate) fn stop_mcq_sq(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<()> {
        self.write_mcq_sq_runtime_control(oprs, queue, MCQ_SQ_STOP)?;
        self.wait_mcq_sq_status(oprs, queue, MCQ_SQ_STS, true)
    }

    pub(crate) fn start_mcq_sq(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<()> {
        self.write_mcq_sq_runtime_control(oprs, queue, MCQ_SQ_START)?;
        self.wait_mcq_sq_status(oprs, queue, MCQ_SQ_STS, false)
    }

    #[inline]
    pub(crate) fn write_mcq_sq_cleanup_target(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        lun: u8,
        tag: u8,
    ) -> Result<()> {
        self.write_mcq_opr(
            oprs,
            UfsMcqOprRegion::Sqd,
            queue,
            REG_MCQ_SQCTI,
            (u32::from(lun) << 8) | u32::from(tag),
        )
    }

    pub(crate) fn initiate_mcq_sq_cleanup(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<()> {
        let rtc = self.read_mcq_opr(oprs, UfsMcqOprRegion::Sqd, queue, REG_MCQ_SQRTC)?;
        self.write_mcq_sq_runtime_control(oprs, queue, rtc | MCQ_SQ_ICU)?;
        self.wait_mcq_sq_status(oprs, queue, MCQ_SQ_CUS, true)
    }

    #[inline]
    pub(crate) fn mcq_sq_cleanup_error_code(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
    ) -> Result<u32> {
        Ok(
            (self.read_mcq_sq_runtime_status(oprs, queue)? & MASK_MCQ_SQ_ICU_ERR_CODE)
                >> SHIFT_MCQ_SQ_ICU_ERR_CODE,
        )
    }

    #[inline]
    pub(crate) fn read_mcq_cq_head(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        self.read_mcq_opr(oprs, UfsMcqOprRegion::Cqd, queue, REG_MCQ_CQHP)
    }

    #[inline]
    pub(crate) fn write_mcq_cq_head(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        head: u32,
    ) -> Result<()> {
        self.write_mcq_opr(oprs, UfsMcqOprRegion::Cqd, queue, REG_MCQ_CQHP, head)
    }

    #[inline]
    pub(crate) fn read_mcq_cq_tail(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        self.read_mcq_opr(oprs, UfsMcqOprRegion::Cqd, queue, REG_MCQ_CQTP)
    }

    #[inline]
    pub(crate) fn read_mcq_cqis(&self, oprs: &UfsMcqOprSet, queue: usize) -> Result<u32> {
        self.read_mcq_opr(oprs, UfsMcqOprRegion::Cqis, queue, REG_MCQ_CQIS)
    }

    #[inline]
    pub(crate) fn write_mcq_cqis(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
        value: u32,
    ) -> Result<()> {
        self.write_mcq_opr(oprs, UfsMcqOprRegion::Cqis, queue, REG_MCQ_CQIS, value)
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
            REG_MCQ_CQIE,
            MCQ_CQIS_TAIL_ENT_PUSH_STS,
        )
    }

    // Helpers
    #[inline]
    pub(crate) fn nutrs(&self) -> usize {
        let access = self.bar.try_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES)
            .transfer_request_slots_sdb()
            .get() as usize
            + 1
    }

    #[inline]
    pub(crate) fn mcq_supported(&self) -> bool {
        let access = self.bar.try_access().unwrap();
        access.read(CONTROLLER_CAPABILITIES).mcq_supported()
    }

    #[inline]
    pub(crate) fn nutrs_mcq(&self) -> usize {
        let access = self.bar.try_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES_MCQ)
            .transfer_request_slots()
            .get() as usize
            + 1
    }

    #[inline]
    pub(crate) fn nutmrs(&self) -> usize {
        let access = self.bar.try_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES)
            .task_management_request_slots()
            .get() as usize
            + 1
    }

    #[inline]
    pub(crate) fn autoh8(&self) -> bool {
        let access = self.bar.try_access().unwrap();
        access
            .read(CONTROLLER_CAPABILITIES)
            .auto_hibern8_supported()
    }

    #[inline]
    pub(crate) fn ctrl_enable(&self) {
        let access = self.bar.try_access().unwrap();
        access.update(CONTROLLER_ENABLE_REG, |reg| {
            reg.with_controller_enable(true)
        });
    }

    #[inline]
    pub(crate) fn ctrl_disable(&self) {
        let access = self.bar.try_access().unwrap();
        access.write_reg(CONTROLLER_ENABLE_REG::zeroed());
    }

    #[inline]
    pub(crate) fn ctrl_enabled(&self) -> bool {
        let access = self.bar.try_access().unwrap();
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
                let access = self.bar.try_access().ok_or(ENODEV)?;
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
                let access = self.bar.try_access().ok_or(ENODEV)?;
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

    pub(crate) fn confirm_uic_interrupts(&self, value: u32) {
        self.write_is(value & UIC_INTR_MASK);
    }

    pub(crate) fn get_uic_cmd_result(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read(UIC_ARG2).command_result().get()
    }

    pub(crate) fn get_dme_attr_val(&self) -> u32 {
        self.read_uic_arg3()
    }

    pub(crate) fn get_power_mode_change_status(&self) -> u32 {
        let access = self.bar.try_access().unwrap();
        access
            .read(CONTROLLER_STATUS)
            .power_mode_change_request_status()
            .get()
    }

    pub(crate) fn wait_for_uic_cmd_ready(&self, interval_us: i64, timeout_ms: i64) -> Result<()> {
        match read_poll_timeout(
            || {
                let access = self.bar.try_access().ok_or(ENODEV)?;
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

    pub(crate) fn confirm_transfer_interrupts(&self, value: u32) {
        self.write_is(value & (UTP_TRANSFER_REQ_COMPL | MCQ_CQ_EVENT_STATUS | ERROR_MASK));
    }

    pub(crate) fn enable_interrupts(&self) {
        self.write_ie(self.read_ie() | UTP_REQ_COMPL_MASK | ERROR_MASK);
    }

    pub(crate) fn disable_transfer_req_int_aggr(&self) {
        let access = self.bar.try_access().unwrap();
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
                let access = self.bar.try_access().ok_or(ENODEV)?;
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
        let access = self.bar.try_access().unwrap();
        access
            .read(UTP_TRANSFER_REQ_LIST_COMPLETION_NOTIFICATION)
            .value()
            .get()
    }

    pub(crate) fn write_utrlcnr(&self, value: u32) {
        let access = self.bar.try_access().unwrap();
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
