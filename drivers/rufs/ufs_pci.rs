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

kernel::pci_device_table!(
    PCI_TABLE,
    MODULE_PCI_TABLE,
    <UfsPci as pci::Driver>::IdInfo,
    [(pci::DeviceId::from_id(pci::Vendor::REDHAT, 0x0013), (),)]
);

struct UfsPci;

#[pin_data]
struct UfsPciData<'a> {
    pdev: &'a pci::Device,
    #[pin]
    host: UfsHost,
}

impl pci::Driver for UfsPci {
    type IdInfo = ();
    type Data<'a> = UfsPciData<'a>;
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe<'a>(
        pdev: &'a pci::Device<Core<'_>>,
        _info: &'a Self::IdInfo,
    ) -> impl PinInit<Self::Data<'a>, Error> + 'a {
        pin_init::pin_init_scope(move || {
            pr_info!(
                "rufs: probe: vendor={} device=0x{:04x}",
                pdev.vendor_id(),
                pdev.device_id(),
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
