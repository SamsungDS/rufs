// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::{pci, device::Core, devres::Devres, prelude::*, c_str, sync::Arc};
use kernel::io::{Io, poll::read_poll_timeout};
use kernel::time::Delta;

const UFS_BAR0_LEN: usize = 0x1000;
type Bar0 = pci::Bar<UFS_BAR0_LEN>;

const REG_CONTROLLER_CAPABILITIES:           usize = 0x00; // CAP[31:0]
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

const MASK_UIC_COMMAND_RESULT:                  u32 = 0xFF;

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

    // Helpers
    #[inline]
    pub(crate) fn nutrs(&self) -> usize {
        (self.read_cap_lo() & MASK_TRANSFER_REQUESTS_SLOTS_SDB) as usize + 1
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
        self.read_is() & self.read_ie() & UTP_TRANSFER_REQ_COMPL
    }

    pub(crate) fn confirm_transfer_interrupts(&self, value: u32) {
        self.write_is(value & UTP_TRANSFER_REQ_COMPL);
    }

    pub(crate) fn enable_interrupts(&self) {
        self.write_ie(self.read_ie() | UTP_REQ_COMPL_MASK | ERROR_MASK);
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
