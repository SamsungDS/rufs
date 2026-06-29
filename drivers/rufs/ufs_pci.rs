// SPDX-License-Identifier: GPL-2.0

//! Driver for UFS devices.
//!
//! Based on the C driver written by Santosh Yaraganavi <santosh.sy@samsung.com>.

use kernel::{device::Core, pci, prelude::*};

mod ufs_dev;
mod ufs_dma;
mod ufs_host;
mod ufs_irq;
mod ufs_lu;
mod ufs_queue;
mod ufs_reg;
mod ufs_uic;

use ufs_host::UfsHost;

#[derive(Clone, Copy)]
enum UfsPciPlatform {
    Qemu,
    Samsung,
    IntelCnl,
    IntelEhl,
    IntelLkf,
    IntelAdl,
    IntelMtl,
}

impl UfsPciPlatform {
    const fn name(self) -> &'static str {
        match self {
            Self::Qemu => "qemu",
            Self::Samsung => "samsung",
            Self::IntelCnl => "intel-cnl",
            Self::IntelEhl => "intel-ehl",
            Self::IntelLkf => "intel-lkf",
            Self::IntelAdl => "intel-adl",
            Self::IntelMtl => "intel-mtl",
        }
    }
}

kernel::pci_device_table!(
    PCI_TABLE,
    MODULE_PCI_TABLE,
    <UfsPci as pci::Driver>::IdInfo,
    [
        // Match the PCI IDs handled by drivers/ufs/host/ufshcd-pci.c.
        (
            pci::DeviceId::from_id(pci::Vendor::REDHAT, 0x0013),
            UfsPciPlatform::Qemu,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::SAMSUNG, 0xc00c),
            UfsPciPlatform::Samsung,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x9dfa),
            UfsPciPlatform::IntelCnl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x4b41),
            UfsPciPlatform::IntelEhl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x4b43),
            UfsPciPlatform::IntelEhl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x98fa),
            UfsPciPlatform::IntelLkf,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x51ff),
            UfsPciPlatform::IntelAdl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x54ff),
            UfsPciPlatform::IntelAdl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x7e47),
            UfsPciPlatform::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0xa847),
            UfsPciPlatform::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x7747),
            UfsPciPlatform::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0xe447),
            UfsPciPlatform::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x4d47),
            UfsPciPlatform::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0xd335),
            UfsPciPlatform::IntelMtl,
        ),
    ]
);

struct UfsPci;

#[pin_data]
struct UfsPciData<'a> {
    pdev: &'a pci::Device,
    #[pin]
    host: UfsHost,
}

impl pci::Driver for UfsPci {
    type IdInfo = UfsPciPlatform;
    type Data<'a> = UfsPciData<'a>;
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe<'a>(
        pdev: &'a pci::Device<Core<'_>>,
        platform: &'a Self::IdInfo,
    ) -> impl PinInit<Self::Data<'a>, Error> + 'a {
        pin_init::pin_init_scope(move || {
            pr_info!(
                "rufs: probe: platform={} vendor={} device=0x{:04x} subvendor=0x{:04x} subdevice=0x{:04x} class={} revision=0x{:02x}",
                platform.name(),
                pdev.vendor_id(),
                pdev.device_id(),
                pdev.subsystem_vendor_id(),
                pdev.subsystem_device_id(),
                pdev.pci_class(),
                pdev.revision_id(),
            );


            pdev.enable_device_mem()?;
            pdev.set_master();

            let host = UfsHost::new(pdev);
            //host.bring_up_controller()?;

            pr_info!("rufs: probe done");

            Ok(try_pin_init!(UfsPciData { pdev, host <- host}))
        })
    }

    fn unbind(pdev: &pci::Device<Core<'_>>, _this: Pin<&Self::Data<'_>>) {
        dev_dbg!(pdev.as_ref(), "Remove Rust UFS driver.\n");
    }
}

kernel::module_pci_driver! {
    type: UfsPci,
    name: "rufs_pci",
    authors: ["Jaemyung Lee"],
    description: "Rust UFS (RUFS) PCI driver",
    license: "GPL v2",
}
