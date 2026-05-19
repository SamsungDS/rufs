// SPDX-License-Identifier: GPL-2.0

//! Driver for UFS devices.
//!
//! Based on the C driver written by Santosh Yaraganavi <santosh.sy@samsung.com>.

use kernel::{device::Core, pci, prelude::*, sync::aref::ARef, sync::Arc};

mod ufs_host;
mod ufs_reg;
mod ufs_dma;
mod ufs_irq;
mod ufs_uic;
mod ufs_queue;

use ufs_host::UfsHost;

kernel::pci_device_table!(
    PCI_TABLE,
    MODULE_PCI_TABLE,
    <UfsPci as pci::Driver>::IdInfo,
    [(
        pci::DeviceId::from_id(pci::Vendor::REDHAT, 0x0013),
        (),
    )]
);

#[pin_data(PinnedDrop)]
struct UfsPci {
    pdev: ARef<pci::Device>,
    host: Arc<UfsHost>,
}

impl pci::Driver for UfsPci {
    type IdInfo = ();
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe(pdev: &pci::Device<Core>, _info: &Self::IdInfo) -> impl PinInit<Self, Error> {
        pin_init::pin_init_scope(move || {
            pr_info!(
                "rufs: probe: vendor={} device=0x{:04x}",
                pdev.vendor_id(), pdev.device_id(),
            );

            pdev.enable_device_mem()?;
            pdev.set_master();

            let host = UfsHost::new(pdev)?;
            //host.bring_up_controller()?;

            pr_info!("rufs: probe done");

            Ok(try_pin_init!(Self {
                pdev: pdev.into(),
                host: host,
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
