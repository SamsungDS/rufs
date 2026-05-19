// SPDX-License-Identifier: GPL-2.0

//! Driver for UFS devices.
//!
//! Based on the C driver written by Santosh Yaraganavi <santosh.sy@samsung.com>.

use kernel::{c_str, device::Core, devres::Devres, pci, prelude::*, sync::aref::ARef};

kernel::pci_device_table!(
    PCI_TABLE,
    MODULE_PCI_TABLE,
    <UfsPci as pci::Driver>::IdInfo,
    [(
        pci::DeviceId::from_id(pci::Vendor::REDHAT, 0x0013),
        (),
    )]
);

const UFS_BAR0_LEN: usize = 0x1000;
type Bar0 = pci::Bar<UFS_BAR0_LEN>;

#[pin_data(PinnedDrop)]
struct UfsPci {
    pdev: ARef<pci::Device>,
    #[pin]
    bar: Devres<Bar0>
}

impl pci::Driver for UfsPci {
    type IdInfo = ();
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe(pdev: &pci::Device<Core>, _info: &Self::IdInfo) -> impl PinInit<Self, Error> {
        pin_init::pin_init_scope(move || {
            pr_info!(
                "rufs: probe: vendor={} device=0x{:04x}, BAR0=0x{:x}",
                pdev.vendor_id(), pdev.device_id(), UFS_BAR0_LEN,
            );

            pdev.enable_device_mem()?;
            pdev.set_master();

            Ok(try_pin_init!(Self {
                bar <- pdev.iomap_region_sized::<{ UFS_BAR0_LEN }>(0, c_str!("rufs_pci")),
                pdev: pdev.into(),
                _: {
                    pr_info!("rufs: probe done");
                },
            }))
        })
    }


    fn unbind(_pdev: &pci::Device<Core>, _this: Pin<&Self>) {
    }
}

#[pinned_drop]
impl PinnedDrop for UfsPci {
    fn drop(self: Pin<&mut Self>) {
        dev_dbg!(self.pdev.as_ref(), "Remove Rust UFS driver.\n");
    }
}

kernel::module_pci_driver! {
    type: UfsPci,
    name: "rufs_pci",
    authors: ["Jaemyung Lee"],
    description: "Rust UFS (RUFS) PCI driver",
    license: "GPL v2",
}
