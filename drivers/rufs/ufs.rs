// SPDX-License-Identifier: GPL-2.0

//! Driver for UFS host controllers.
//!
//! Based on the C driver written by Santosh Yaraganavi <santosh.sy@samsung.com>.

use kernel::{c_str, driver, prelude::*, InPlaceModule};
#[cfg(CONFIG_RUFS_PCI)]
use kernel::pci;
#[cfg(CONFIG_RUFS_QCOM)]
use kernel::platform;

mod command;
mod device;
mod dma;
mod frontend;
mod hci;
mod host;
mod irq;
mod lu;
mod protocol;
mod queue;
mod reg;
mod resource;
mod transport;
mod uic;
mod variant;

#[cfg(CONFIG_RUFS_PCI)]
use frontend::pci::UfsPci;
#[cfg(CONFIG_RUFS_QCOM)]
use frontend::qcom::UfsQcom;

#[pin_data]
struct UfsModule {
    #[cfg(CONFIG_RUFS_PCI)]
    #[pin]
    _pci_driver: driver::Registration<pci::Adapter<UfsPci>>,
    #[cfg(CONFIG_RUFS_QCOM)]
    #[pin]
    _qcom_driver: driver::Registration<platform::Adapter<UfsQcom>>,
}

impl InPlaceModule for UfsModule {
    fn init(module: &'static kernel::ThisModule) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            #[cfg(CONFIG_RUFS_PCI)]
            _pci_driver <- driver::Registration::new(c_str!("rufs"), module),
            #[cfg(CONFIG_RUFS_QCOM)]
            _qcom_driver <- driver::Registration::new(c_str!("rufs-qcom"), module),
        })
    }
}

module! {
    type: UfsModule,
    name: "rufs",
    authors: ["Jaemyung Lee"],
    description: "Rust UFS host controller driver",
    license: "GPL v2",
}
