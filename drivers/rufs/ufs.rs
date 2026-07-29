// SPDX-License-Identifier: GPL-2.0

//! Driver for UFS host controllers.
//!
//! Based on the C driver written by Santosh Yaraganavi <santosh.sy@samsung.com>.

use kernel::{c_str, driver, pci, prelude::*, InPlaceModule};

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

use frontend::pci::UfsPci;

#[pin_data]
struct UfsModule {
    #[pin]
    _pci_driver: driver::Registration<pci::Adapter<UfsPci>>,
}

impl InPlaceModule for UfsModule {
    fn init(module: &'static kernel::ThisModule) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            _pci_driver <- driver::Registration::new(c_str!("rufs"), module),
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
