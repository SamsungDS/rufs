// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::{pci, device::Core, devres::Devres, prelude::*, c_str, sync::Arc};
use kernel::io::{Io, poll::read_poll_timeout};
use kernel::time::Delta;

const UFS_BAR0_LEN: usize = 0x1000;
type Bar0 = pci::Bar<UFS_BAR0_LEN>;

const REG_CONTROLLER_CAPABILITIES:           usize = 0x00; // CAP[31:0]
const REG_MCQCAP:                            usize = 0x04; // MCQCAP
const REG_CONTROLLER_CAPABILITIES_H:         usize = 0x04; // CAP[63:32]
const REG_UFS_VERSION:                       usize = 0x08; // VER
const REG_INTERRUPT_STATUS:                  usize = 0x20; // IS
const REG_INTERRUPT_ENABLE:                  usize = 0x24; // IE
const REG_CONTROLLER_STATUS:                 usize = 0x30; // HCS
const REG_CONTROLLER_ENABLE:                 usize = 0x34; // HCE

// UIC Error
const REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER: usize = 0x38;
const REG_UIC_ERROR_CODE_DATA_LINK_LAYER:   usize = 0x3C;

// Transfer Request List (UTRL)
const REG_UTP_TRANSFER_REQ_LIST_BASE_L:      usize = 0x50; // UTRLBA
const REG_UTP_TRANSFER_REQ_LIST_BASE_H:      usize = 0x54; // UTRLBAU
const REG_UTP_TRANSFER_REQ_DOOR_BELL:        usize = 0x58; // UTRLDBR
const REG_UTP_TRANSFER_REQ_LIST_CLEAR:       usize = 0x5C; // UTRLCLR
const REG_UTP_TRANSFER_REQ_LIST_RUN_STOP:    usize = 0x60; // UTRLRSR

// Task Management Request List (UTMRL)
const REG_UTP_TASK_REQ_LIST_BASE_L:          usize = 0x70; // UTMRLBA
const REG_UTP_TASK_REQ_LIST_BASE_H:          usize = 0x74; // UTMRLBAU
const REG_UTP_TASK_REQ_DOOR_BELL:            usize = 0x78; // UTMRLDBR
const REG_UTP_TASK_REQ_LIST_CLEAR:           usize = 0x7C; // UTMRLCLR
const REG_UTP_TASK_REQ_LIST_RUN_STOP:        usize = 0x80; // UTMRLRSR
                                                           //
const UTP_TRANSFER_REQ_LIST_RUN_STOP_BIT:   u32 = 0x1;
const UTP_TASK_REQ_LIST_RUN_STOP_BIT:       u32 = 0x1;

// UIC command
const REG_UIC_COMMAND:                       usize = 0x90; // UICCMD
const REG_UIC_ARG1:                          usize = 0x94; // UICCMDARG1
const REG_UIC_ARG2:                          usize = 0x98; // UICCMDARG2
const REG_UIC_ARG3:                          usize = 0x9C; // UICCMDARG3

// MCQ global registers
const REG_UFS_MEM_CFG:                       usize = 0x300;
const REG_UFS_MCQ_CFG:                       usize = 0x380;
const REG_UFS_ESILBA:                        usize = 0x384;
const REG_UFS_ESIUBA:                        usize = 0x388;

// MCQ queue configuration registers. These offsets are relative to each
// queue's 0x40-byte config block.
const REG_MCQ_SQATTR:                        usize = 0x00;
const REG_MCQ_SQLBA:                         usize = 0x04;
const REG_MCQ_SQUBA:                         usize = 0x08;
const REG_MCQ_SQDAO:                         usize = 0x0C;
const REG_MCQ_SQISAO:                        usize = 0x10;
const REG_MCQ_CQATTR:                        usize = 0x20;
const REG_MCQ_CQLBA:                         usize = 0x24;
const REG_MCQ_CQUBA:                         usize = 0x28;
const REG_MCQ_CQDAO:                         usize = 0x2C;
const REG_MCQ_CQISAO:                        usize = 0x30;

// MCQ operation/runtime registers. These offsets are relative to each SQD,
// SQIS, CQD, or CQIS operation region.
const REG_MCQ_SQHP:                          usize = 0x00;
const REG_MCQ_SQTP:                          usize = 0x04;
const REG_MCQ_SQRTC:                         usize = 0x08;
const REG_MCQ_SQCTI:                         usize = 0x0C;
const REG_MCQ_SQRTS:                         usize = 0x10;
const REG_MCQ_CQHP:                          usize = 0x00;
const REG_MCQ_CQTP:                          usize = 0x04;
const REG_MCQ_CQIS:                          usize = 0x00;
const REG_MCQ_CQIE:                          usize = 0x04;

const MASK_UIC_COMMAND_RESULT:                  u32 = 0xFF;

// MCQ capability/configuration masks
const MASK_MCQ_MAX_QUEUE_SUP:                   u32 = 0x000000FF;
const MASK_MCQ_QCFGPTR:                         u32 = 0x00FF0000;
const SHIFT_MCQ_QCFGPTR:                        u32 = 16;
const MASK_MCQ_CFG_MAC:                         u32 = 0x0001FF00;
const SHIFT_MCQ_CFG_MAC:                        u32 = 8;

const MCQ_QCFG_STRIDE:                          usize = 0x40;
const MCQ_QCFGPTR_UNIT:                         usize = 0x200;
const MCQ_ENTRY_SIZE_IN_DWORD:                  u32 = 8;
const MCQ_QUEUE_EN:                             u32 = 1 << 31;
const MCQ_QUEUE_ID_SHIFT:                       u32 = 16;
const MCQ_DEFAULT_OPR_STRIDE:                   usize = 48;
const MCQ_POLL_INTERVAL_US:                     i64 = 20;
const MCQ_POLL_TIMEOUT_US:                      i64 = 500000;

const MCQ_MODE_SELECT:                          u32 = 1 << 0;
const ESI_ENABLE:                               u32 = 1 << 1;
const MCQ_CQIS_TAIL_ENT_PUSH_STS:               u32 = 0x1;

const MCQ_SQ_START:                             u32 = 0x0;
const MCQ_SQ_STOP:                              u32 = 0x1;
const MCQ_SQ_ICU:                               u32 = 0x2;
const MCQ_SQ_STS:                               u32 = 0x1;
const MCQ_SQ_CUS:                               u32 = 0x2;
const MASK_MCQ_SQ_ICU_ERR_CODE:                 u32 = 0xF0;
const SHIFT_MCQ_SQ_ICU_ERR_CODE:                u32 = 4;

// Controller capability masks
const MASK_TRANSFER_REQUESTS_SLOTS_SDB:          u32 = 0x0000001F;
const MASK_TRANSFER_REQUESTS_SLOTS_MCQ:          u32 = 0x000000FF;
const MASK_NUMBER_OUTSTANDING_RTT:               u32 = 0x0000FF00;
const MASK_TASK_MANAGEMENT_REQUEST_SLOTS:        u32 = 0x00070000;
const MASK_EHSLUTRD_SUPPORTED:                   u32 = 0x00400000;
const MASK_AUTO_HIBERN8_SUPPORT:                 u32 = 0x00800000;
const MASK_64_ADDRESSING_SUPPORT:                u32 = 0x01000000;
const MASK_OUT_OF_ORDER_DATA_DELIVERY_SUPPORT:   u32 = 0x02000000;
const MASK_UIC_DME_TEST_MODE_SUPPORT:            u32 = 0x04000000;
const MASK_CRYPTO_SUPPORT:                       u32 = 0x10000000;
const MASK_LSDB_SUPPORT:                         u32 = 0x20000000;
const MASK_MCQ_SUPPORT:                          u32 = 0x40000000;

// IS - Interrupt Status
const UTP_TRANSFER_REQ_COMPL:                    u32 = 0x00000001;
const UIC_DME_END_PT_RESET:                      u32 = 0x00000002;
const UIC_ERROR:                                 u32 = 0x00000004;
const UIC_TEST_MODE:                             u32 = 0x00000008;
const UIC_POWER_MODE:                            u32 = 0x00000010;
const UIC_HIBERNATE_EXIT:                        u32 = 0x00000020;
const UIC_HIBERNATE_ENTER:                       u32 = 0x00000040;
const UIC_LINK_LOST:                             u32 = 0x00000080;
const UIC_LINK_STARTUP:                          u32 = 0x00000100;
const UTP_TASK_REQ_COMPL:                        u32 = 0x00000200;
const UIC_COMMAND_COMPL:                         u32 = 0x00000400;
const DEVICE_FATAL_ERROR:                        u32 = 0x00000800;
const UTP_ERROR:                                 u32 = 0x00001000;
const CONTROLLER_FATAL_ERROR:                    u32 = 0x00010000;
const SYSTEM_BUS_FATAL_ERROR:                    u32 = 0x00020000;
const CRYPTO_ENGINE_FATAL_ERROR:                 u32 = 0x00040000;
const MCQ_CQ_EVENT_STATUS:                       u32 = 0x00100000;

const UIC_INTR_HIBERNATE_MASK: u32 = UIC_HIBERNATE_EXIT | UIC_HIBERNATE_ENTER;
const UIC_INTR_POWER_MASK: u32 = UIC_POWER_MODE | UIC_INTR_HIBERNATE_MASK;
const UIC_INTR_MASK: u32 = UIC_INTR_POWER_MASK | UIC_COMMAND_COMPL;

const UTP_REQ_COMPL_MASK: u32 = UTP_TRANSFER_REQ_COMPL | UTP_TASK_REQ_COMPL;
const ERROR_MASK: u32 = UIC_ERROR | UIC_LINK_LOST | DEVICE_FATAL_ERROR |
                        CONTROLLER_FATAL_ERROR | SYSTEM_BUS_FATAL_ERROR |
                        CRYPTO_ENGINE_FATAL_ERROR | UTP_ERROR;

// HCS - Host Controller Status
const DEVICE_PRESENT:                            u32 = 0x00000001;
const UTP_TRANSFER_REQ_LIST_READY:               u32 = 0x00000002;
const UTP_TASK_REQ_LIST_READY:                   u32 = 0x00000004;
const UIC_COMMAND_READY:                         u32 = 0x00000008;
const HOST_ERROR_INDICATOR:                      u32 = 0x00000010;
const DEVICE_ERROR_INDICATOR:                    u32 = 0x00000020;
const UIC_POWER_MODE_CHANGE_REQ_STATUS_MASK:     u32 = 0x00000700;

const STATUS_READY: u32 = UTP_TRANSFER_REQ_LIST_READY |
                          UTP_TASK_REQ_LIST_READY |
                          UIC_COMMAND_READY;

// HCE - Host Controller Enable
const CONTROLLER_DISABLE:                        u32 = 0x00000000;
const CONTROLLER_ENABLE:                         u32 = 0x00000001;
const CRYPTO_GENERAL_ENABLE:                     u32 = 0x00000002;

pub(crate) enum PowerMode {
    OK          = 0x00,
    Local       = 0x01,
    Remote      = 0x02,
    Busy        = 0x03,
    ErrorCap    = 0x04,
    FatalError  = 0x05,
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
        Self { sqd, sqis, cqd, cqis }
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

#[pin_data]
pub(crate) struct UfsReg {
    #[pin]
    bar: Devres<Bar0>,
}

impl UfsReg {
    pub(crate) fn new(pdev: &pci::Device<Core>) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                bar <- pdev.iomap_region_sized::<UFS_BAR0_LEN>(0, c_str!("rufs_pci")),
            }), GFP_KERNEL,
        )
    }

    #[inline(always)]
    fn read(&self, offset: usize) -> u32 {
        let access = self.bar.try_access().unwrap();
        access.read32(offset)
    }

    #[inline(always)]
    fn write(&self, offset: usize, value: u32) {
        let access = self.bar.try_access().unwrap();
        access.write32(value, offset);
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
    fn update(&self, offset: usize, mask: u32, value: u32) {
        self.write(offset, (self.read(offset) & !mask) | (value & mask));
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
        self.read(REG_CONTROLLER_CAPABILITIES)
    }

    #[inline]
    pub(crate) fn read_cap_hi(&self) -> u32 {
        self.read(REG_CONTROLLER_CAPABILITIES_H)
    }

    #[inline]
    pub(crate) fn read_version(&self) -> u32 {
        self.read(REG_UFS_VERSION)
    }

    #[inline]
    pub(crate) fn read_is(&self) -> u32 {
        self.read(REG_INTERRUPT_STATUS)
    }

    #[inline]
    pub(crate) fn write_is(&self, value: u32) {
        self.write(REG_INTERRUPT_STATUS, value)
    }

    #[inline]
    pub(crate) fn read_ie(&self) -> u32 {
        self.read(REG_INTERRUPT_ENABLE)
    }

    #[inline]
    pub(crate) fn write_ie(&self, value: u32) {
        self.write(REG_INTERRUPT_ENABLE, value)
    }

    #[inline]
    pub(crate) fn read_hcs(&self) -> u32 {
        self.read(REG_CONTROLLER_STATUS)
    }

    #[inline]
    pub(crate) fn read_hce(&self) -> u32 {
        self.read(REG_CONTROLLER_ENABLE)
    }

    #[inline]
    pub(crate) fn write_hce(&self, value: u32) {
        self.write(REG_CONTROLLER_ENABLE, value)
    }

    #[inline]
    pub(crate) fn read_uic_error_phy(&self) -> u32 {
        self.read(REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER)
    }

    #[inline]
    pub(crate) fn write_uic_error_phy(&self, value: u32) {
        self.write(REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER, value)
    }

    // UTRL(Transfer)
    #[inline]
    pub(crate) fn write_utrlba(&self, low: u32) {
        self.write(REG_UTP_TRANSFER_REQ_LIST_BASE_L, low)
    }

    #[inline]
    pub(crate) fn write_utrlbau(&self, high: u32) {
        self.write(REG_UTP_TRANSFER_REQ_LIST_BASE_H, high)
    }

    #[inline]
    pub(crate) fn read_utrl_doorbell(&self) -> u32 {
        self.read(REG_UTP_TRANSFER_REQ_DOOR_BELL)
    }

    #[inline]
    pub(crate) fn ring_utrl_doorbell(&self, tag: usize) {
        self.write(REG_UTP_TRANSFER_REQ_DOOR_BELL, 1 << tag)
    }

    #[inline]
    pub(crate) fn write_utrl_runstop(&self, value: u32) {
        self.write(REG_UTP_TRANSFER_REQ_LIST_RUN_STOP, value)
    }

    #[inline]
    pub(crate) fn clear_utrl_slots(&self, mask: u32) {
        self.write(REG_UTP_TRANSFER_REQ_LIST_CLEAR, mask)
    }

    // UTMRL(Task Management)
    #[inline]
    pub(crate) fn write_utmrlba(&self, low: u32) {
        self.write(REG_UTP_TASK_REQ_LIST_BASE_L, low)
    }

    #[inline]
    pub(crate) fn write_utmrlbau(&self, high: u32) {
        self.write(REG_UTP_TASK_REQ_LIST_BASE_H, high)
    }

    #[inline]
    pub(crate) fn read_utmrl_doorbell(&self) -> u32 {
        self.read(REG_UTP_TASK_REQ_DOOR_BELL)
    }

    #[inline]
    pub(crate) fn ring_utmrl_doorbell(&self, mask: u32) {
        self.write(REG_UTP_TASK_REQ_DOOR_BELL, mask)
    }

    #[inline]
    pub(crate) fn write_utmrl_runstop(&self, value: u32) {
        self.write(REG_UTP_TASK_REQ_LIST_RUN_STOP, value)
    }

    #[inline]
    pub(crate) fn clear_utmrl_slots(&self, mask: u32) {
        self.write(REG_UTP_TASK_REQ_LIST_CLEAR, mask)
    }

    // UIC command
    #[inline]
    pub(crate) fn read_uic_cmd(&self) -> u32 {
        self.read(REG_UIC_COMMAND)
    }

    #[inline]
    pub(crate) fn write_uic_cmd(&self, value: u32) {
        self.write(REG_UIC_COMMAND, value)
    }

    #[inline]
    pub(crate) fn read_uic_arg1(&self) -> u32 {
        self.read(REG_UIC_ARG1)
    }

    #[inline]
    pub(crate) fn write_uic_arg1(&self, value: u32) {
        self.write(REG_UIC_ARG1, value)
    }

    #[inline]
    pub(crate) fn read_uic_arg2(&self) -> u32 {
        self.read(REG_UIC_ARG2)
    }

    #[inline]
    pub(crate) fn write_uic_arg2(&self, value: u32) {
        self.write(REG_UIC_ARG2, value)
    }

    #[inline]
    pub(crate) fn read_uic_arg3(&self) -> u32 {
        self.read(REG_UIC_ARG3)
    }

    #[inline]
    pub(crate) fn write_uic_arg3(&self, value: u32) {
        self.write(REG_UIC_ARG3, value)
    }

    // MCQ global configuration
    #[inline]
    pub(crate) fn read_mcq_cap(&self) -> u32 {
        self.read(REG_MCQCAP)
    }

    #[inline]
    pub(crate) fn mcq_max_queues(&self) -> usize {
        (self.read_mcq_cap() & MASK_MCQ_MAX_QUEUE_SUP) as usize + 1
    }

    #[inline]
    pub(crate) fn mcq_queue_cfg_base(&self) -> usize {
        (((self.read_mcq_cap() & MASK_MCQ_QCFGPTR) >> SHIFT_MCQ_QCFGPTR) as usize)
            * MCQ_QCFGPTR_UNIT
    }

    #[inline]
    pub(crate) fn read_ufs_mem_cfg(&self) -> u32 {
        self.read(REG_UFS_MEM_CFG)
    }

    #[inline]
    pub(crate) fn write_ufs_mem_cfg(&self, value: u32) {
        self.write(REG_UFS_MEM_CFG, value)
    }

    #[inline]
    pub(crate) fn enable_mcq_mode(&self) {
        self.update(REG_UFS_MEM_CFG, MCQ_MODE_SELECT, MCQ_MODE_SELECT);
    }

    #[inline]
    pub(crate) fn disable_mcq_mode(&self) {
        self.update(REG_UFS_MEM_CFG, MCQ_MODE_SELECT, 0);
    }

    #[inline]
    pub(crate) fn enable_mcq_esi(&self) {
        self.update(REG_UFS_MEM_CFG, ESI_ENABLE, ESI_ENABLE);
    }

    #[inline]
    pub(crate) fn config_mcq_esi(&self, addr: u64) {
        self.write(REG_UFS_ESILBA, Self::dma_addr_lo(addr));
        self.write(REG_UFS_ESIUBA, Self::dma_addr_hi(addr));
    }

    #[inline]
    pub(crate) fn read_mcq_cfg(&self) -> u32 {
        self.read(REG_UFS_MCQ_CFG)
    }

    #[inline]
    pub(crate) fn write_mcq_cfg(&self, value: u32) {
        self.write(REG_UFS_MCQ_CFG, value)
    }

    pub(crate) fn config_mcq_max_active_cmds(&self, max_active_cmds: u32) -> Result<()> {
        if max_active_cmds == 0 {
            return Err(EINVAL);
        }

        self.update(
            REG_UFS_MCQ_CFG,
            MASK_MCQ_CFG_MAC,
            (max_active_cmds - 1) << SHIFT_MCQ_CFG_MAC,
        );
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

    pub(crate) fn initiate_mcq_sq_cleanup(
        &self,
        oprs: &UfsMcqOprSet,
        queue: usize,
    ) -> Result<()> {
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
        Ok((self.read_mcq_sq_runtime_status(oprs, queue)? & MASK_MCQ_SQ_ICU_ERR_CODE)
            >> SHIFT_MCQ_SQ_ICU_ERR_CODE)
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
        (self.read_cap_lo() & MASK_TRANSFER_REQUESTS_SLOTS_SDB) as usize + 1
    }

    #[inline]
    pub(crate) fn mcq_supported(&self) -> bool {
        (self.read_cap_lo() & MASK_MCQ_SUPPORT) != 0
    }

    #[inline]
    pub(crate) fn nutrs_mcq(&self) -> usize {
        (self.read_cap_lo() & MASK_TRANSFER_REQUESTS_SLOTS_MCQ) as usize + 1
    }

    #[inline]
    pub(crate) fn nutmrs(&self) -> usize {
        ((self.read_cap_lo() &
          MASK_TASK_MANAGEMENT_REQUEST_SLOTS) >> 16) as usize + 1
    }

    #[inline]
    pub(crate) fn autoh8(&self) -> bool {
        (self.read_cap_lo() & MASK_AUTO_HIBERN8_SUPPORT) != 0
    }

    #[inline]
    pub(crate) fn ctrl_enable(&self) {
        self.write_hce(self.read_hce() | CONTROLLER_ENABLE);
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
        self.write_utrlbau((dma_addr >>32) as u32);
    }

    #[inline]
    pub(crate) fn set_utmrdl_base(&self, dma_addr: u64) {
        self.write_utmrlba(dma_addr as u32);
        self.write_utmrlbau((dma_addr >>32) as u32);
    }

    #[inline]
    pub(crate) fn wait_for_ctrl_enable(
        &self,
        interval_us: i64,
        timeout_ms: i64,
    ) -> Result<()> {

        pr_info!("[RUFS] drivers/rufs/ufs_reg: wait_for_ctrl_enable");
        match read_poll_timeout(
            || Ok(self.read_hce()),
            |v: &u32| (*v & CONTROLLER_ENABLE) == CONTROLLER_ENABLE,
            Delta::from_micros(interval_us),
            Delta::from_millis(timeout_ms),
        ) {
            Ok(_) => { Ok(()) },
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
        self.read_uic_arg2() & MASK_UIC_COMMAND_RESULT
    }

    pub(crate) fn get_dme_attr_val(&self) -> u32 {
        self.read_uic_arg3()
    }

    pub(crate) fn wait_for_uic_cmd_ready(
        &self,
        interval_us: i64,
        timeout_ms: i64,
    ) -> Result<()> {
        match read_poll_timeout(
            || Ok(self.read_hcs()),
            |v: &u32| (*v & UIC_COMMAND_READY) == UIC_COMMAND_READY,
            Delta::from_micros(interval_us),
            Delta::from_millis(timeout_ms),
        ) {
            Ok(_) => { Ok(()) },
            Err(e) => Err(e),
        }
    }

    pub(crate) fn read_transfer_interrupts(&self) -> u32 {
        self.read_is() & self.read_ie() & (UTP_TRANSFER_REQ_COMPL | MCQ_CQ_EVENT_STATUS)
    }

    pub(crate) fn confirm_transfer_interrupts(&self, value: u32) {
        self.write_is(value & (UTP_TRANSFER_REQ_COMPL | MCQ_CQ_EVENT_STATUS));
    }

    pub(crate) fn enable_interrupts(&self) {
        self.write_ie(self.read_ie() | UTP_REQ_COMPL_MASK | ERROR_MASK);
    }

    pub(crate) fn enable_mcq_interrupts(&self) {
        self.write_ie(self.read_ie() | MCQ_CQ_EVENT_STATUS);
    }

    pub(crate) fn wait_for_request_ready(
        &self,
        interval_us: i64,
        timeout_ms: i64,
    ) -> Result<()> {
        match read_poll_timeout(
            || Ok(self.read_hcs()),
            |v: &u32| (*v & STATUS_READY) == STATUS_READY,
            Delta::from_micros(interval_us),
            Delta::from_millis(timeout_ms),
        ) {
            Ok(_) => { Ok(()) },
            Err(e) => Err(e),
        }
    }

    pub(crate) fn enable_run_stop(&self) {
        self.write_utmrl_runstop(UTP_TASK_REQ_LIST_RUN_STOP_BIT);
        self.write_utrl_runstop(UTP_TRANSFER_REQ_LIST_RUN_STOP_BIT);
    }
}

#[inline]
pub(crate) fn is_uic_command_completion(interrupt_status: u32) -> bool {
    (interrupt_status & UIC_COMMAND_COMPL) != 0
}

#[inline]
pub(crate) fn is_uic_power_mode(interrupt_status: u32) -> bool {
    (interrupt_status & UIC_INTR_POWER_MASK) != 0
}
