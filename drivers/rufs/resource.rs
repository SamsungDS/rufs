// SPDX-License-Identifier: GPL-2.0

//! Resources owned by a UFS host-controller frontend.

use kernel::device::{self, Bound};
use kernel::devres::Devres;
use kernel::io::mem::IoMem;
use kernel::io::{IoBase, Mmio, MmioBackend, Region};
use kernel::sync::{aref::ARef, Arc};
use kernel::{c_str, pci, prelude::*};

pub(crate) const HCI_MMIO_SIZE: usize = 0x1000;

type PciHciMmio = pci::Bar<'static, HCI_MMIO_SIZE>;

pub(crate) enum HciMmio {
    Pci(Devres<PciHciMmio>),
    #[allow(dead_code)]
    Platform(Devres<IoMem<'static, HCI_MMIO_SIZE>>),
}

impl HciMmio {
    pub(crate) fn from_pci(pdev: &pci::Device<Bound>) -> Result<Self> {
        Ok(Self::Pci(
            pdev.iomap_region_sized::<HCI_MMIO_SIZE>(0, c_str!("rufs_pci"))?
                .into_devres()?,
        ))
    }

    fn access<'a>(
        &'a self,
        dev: &'a device::Device<Bound>,
    ) -> Result<HciMmioAccess<'a>> {
        match self {
            Self::Pci(mmio) => Ok(HciMmioAccess::Pci(mmio.access(dev)?)),
            Self::Platform(mmio) => Ok(HciMmioAccess::Platform(mmio.access(dev)?)),
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) enum HciMmioAccess<'a> {
    Pci(&'a PciHciMmio),
    Platform(&'a IoMem<'static, HCI_MMIO_SIZE>),
}

impl<'a> IoBase<'a> for HciMmioAccess<'a> {
    type Backend = MmioBackend;
    type Target = Region<HCI_MMIO_SIZE>;

    fn as_view(self) -> Mmio<'a, Self::Target> {
        match self {
            Self::Pci(mmio) => mmio.as_view(),
            Self::Platform(mmio) => mmio.as_view(),
        }
    }
}

pub(crate) struct HostResources {
    device: ARef<device::Device>,
    hci: Arc<HciMmio>,
}

impl HostResources {
    pub(crate) fn new(device: ARef<device::Device>, hci: HciMmio) -> Result<Arc<Self>> {
        Ok(Arc::new(
            Self {
                device,
                hci: Arc::new(hci, GFP_KERNEL)?,
            },
            GFP_KERNEL,
        )?)
    }

    pub(crate) fn device(&self) -> &device::Device<Bound> {
        // SAFETY: `HostResources` is owned by the bound RUFS driver instance
        // and is dropped before the frontend finishes unbinding the device.
        unsafe { self.device.as_bound() }
    }

    pub(crate) fn hci_access(&self) -> Result<HciMmioAccess<'_>> {
        self.hci.access(self.device())
    }
}
