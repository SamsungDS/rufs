// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::{pci, device::Core, prelude::*, new_spinlock};
use kernel::{dma, dma_read, dma_write};
use kernel::sync::{Arc, SpinLock};
use crate::ufs_reg::*;

const PRDT_DATA_BYTE_COUNT_MAX: u32 = 0x00040000; // SZ_256K
const PRDT_DATA_BYTE_COUNT_PAD: usize = 4;
const ALIGNED_UPIU_SIZE: usize = 512;
const MAX_PRD_ENTRIES: usize = 256;

// UTP Request Descriptor Header
#[repr(C)]
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


// UTP Command Descriptor
#[repr(C)]
#[derive(Default, Clone, Copy)]
struct PrdEntry {
    addr: u64,      // (LE)
    reserved: u32,  // (LE)
    size: u32,      // (LE)
}

#[repr(C)]
struct Ucd {
    cmd_upiu: [u8; ALIGNED_UPIU_SIZE],
    rsp_upiu: [u8; ALIGNED_UPIU_SIZE],
    prdt: [PrdEntry; MAX_PRD_ENTRIES],
}

// UTP Transfer Request Descriptor
#[repr(C)]
struct Utrd {
    header: ReqDescHeader,
    command_desc_base_addr: u64,
    rsp_upiu_length: u16,
    rsp_upiu_offset: u16,
    prd_table_length: u16,
    prd_table_offset: u16,
}


// UTP Task Management Request Descriptor
#[repr(C)]
#[derive(Default, Clone, Copy)]
struct UpiuHeader {
    transaction_code: u8,
    flags: u8,
    lun: u8,
    task_tag: u8,
    cmd_set: u8,
    func: u8,
    response: u8,
    status: u8,
    ehs_length: u8,
    dev_info: u8,
    data_seg_len: u16, // (BE)
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct UpiuTmReq {
    header: UpiuHeader,
    input_param1: u32,  // (BE)
    input_param2: u32,  // (BE)
    input_param3: u32,  // (BE)
    reserved: [u32; 2],
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct UpiuTmRsp {
    header: UpiuHeader,
    output_param1: u32, // (BE)
    output_param2: u32, // (BE)
    reserved: [u32; 3],
}

#[repr(C)]
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

            // CAST: TODO
            let command_desc_base_addr = kernel::ptr::project!(ucdl.as_ptr(), [tag]?).addr() as u64;

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
}

const _: () = { assert!(size_of::<ReqDescHeader>() == 16); };
const _: () = { assert!(size_of::<PrdEntry>() == 16); };
const _: () = { assert!(size_of::<Ucd>() == 5120); };
const _: () = { assert!(size_of::<Utrd>() == 32); };
const _: () = { assert!(size_of::<UpiuHeader>() == 12); };
const _: () = { assert!(size_of::<UpiuTmReq>() == 32); };
const _: () = { assert!(size_of::<UpiuTmRsp>() == 32); };
const _: () = { assert!(size_of::<Utmrd>() == 80); };

unsafe impl kernel::transmute::AsBytes for Ucd {}
unsafe impl kernel::transmute::FromBytes for Ucd {}
unsafe impl kernel::transmute::AsBytes for Utrd {}
unsafe impl kernel::transmute::FromBytes for Utrd {}
unsafe impl kernel::transmute::AsBytes for Utmrd {}
unsafe impl kernel::transmute::FromBytes for Utmrd {}
