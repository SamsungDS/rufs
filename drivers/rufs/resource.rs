// SPDX-License-Identifier: GPL-2.0

//! Resources owned by a UFS host-controller frontend.

use kernel::alloc::KBox;
use kernel::device::{self, Bound};
use kernel::devres::Devres;
use kernel::io::mem::IoMem;
use kernel::io::{IoBase, Mmio, MmioBackend, Region};
use kernel::sync::{aref::ARef, Arc};
#[cfg(CONFIG_RUFS_PCI)]
use kernel::{c_str, pci};
#[cfg(CONFIG_RUFS_QCOM)]
use kernel::platform;
use kernel::prelude::*;

use crate::variant::UfsVariantOps;

pub(crate) const HCI_MMIO_SIZE: usize = 0x1000;

#[cfg(CONFIG_RUFS_PCI)]
type PciHciMmio = pci::Bar<'static, HCI_MMIO_SIZE>;

pub(crate) enum HciMmio {
    #[cfg(CONFIG_RUFS_PCI)]
    Pci(Devres<PciHciMmio>),
    #[cfg(CONFIG_RUFS_QCOM)]
    Platform(Devres<IoMem<'static, HCI_MMIO_SIZE>>),
}

/// An optional, separately mapped MCQ register region.
///
/// Some platform controllers expose MCQ queue configuration and operation
/// registers through a named resource rather than the standard HCI region.
pub(crate) struct McqMmio(Devres<IoMem<'static>>);

impl McqMmio {
    #[cfg(CONFIG_RUFS_QCOM)]
    pub(crate) fn from_platform(mmio: Devres<IoMem<'static>>) -> Self {
        Self(mmio)
    }

    fn access<'a>(
        &'a self,
        dev: &'a device::Device<Bound>,
    ) -> Result<&'a IoMem<'static>> {
        self.0.access(dev)
    }
}

impl HciMmio {
    #[cfg(CONFIG_RUFS_PCI)]
    pub(crate) fn from_pci(pdev: &pci::Device<Bound>) -> Result<Self> {
        Ok(Self::Pci(
            pdev.iomap_region_sized::<HCI_MMIO_SIZE>(0, c_str!("rufs_pci"))?
                .into_devres()?,
        ))
    }

    #[cfg(CONFIG_RUFS_QCOM)]
    pub(crate) fn from_platform(pdev: &platform::Device<Bound>) -> Result<Self> {
        let request = pdev.io_request_by_index(0).ok_or(ENODEV)?;

        Ok(Self::Platform(
            request.iomap_sized::<HCI_MMIO_SIZE>()?.into_devres()?,
        ))
    }

    fn access<'a>(
        &'a self,
        dev: &'a device::Device<Bound>,
    ) -> Result<HciMmioAccess<'a>> {
        match self {
            #[cfg(CONFIG_RUFS_PCI)]
            Self::Pci(mmio) => Ok(HciMmioAccess::Pci(mmio.access(dev)?)),
            #[cfg(CONFIG_RUFS_QCOM)]
            Self::Platform(mmio) => Ok(HciMmioAccess::Platform(mmio.access(dev)?)),
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) enum HciMmioAccess<'a> {
    #[cfg(CONFIG_RUFS_PCI)]
    Pci(&'a PciHciMmio),
    #[cfg(CONFIG_RUFS_QCOM)]
    Platform(&'a IoMem<'static, HCI_MMIO_SIZE>),
}

impl<'a> IoBase<'a> for HciMmioAccess<'a> {
    type Backend = MmioBackend;
    type Target = Region<HCI_MMIO_SIZE>;

    fn as_view(self) -> Mmio<'a, Self::Target> {
        match self {
            #[cfg(CONFIG_RUFS_PCI)]
            Self::Pci(mmio) => mmio.as_view(),
            #[cfg(CONFIG_RUFS_QCOM)]
            Self::Platform(mmio) => mmio.as_view(),
        }
    }
}

pub(crate) struct HostResources {
    device: ARef<device::Device>,
    hci: Arc<HciMmio>,
    mcq: Option<Arc<McqMmio>>,
    variant: KBox<dyn UfsVariantOps>,
}

impl HostResources {
    pub(crate) fn new(
        device: ARef<device::Device>,
        hci: HciMmio,
        mcq: Option<McqMmio>,
        variant: KBox<dyn UfsVariantOps>,
    ) -> Result<Arc<Self>> {
        Ok(Arc::new(
            Self {
                device,
                hci: Arc::new(hci, GFP_KERNEL)?,
                mcq: mcq
                    .map(|mcq| Arc::new(mcq, GFP_KERNEL))
                    .transpose()?,
                variant,
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

    pub(crate) fn mcq_access(&self) -> Result<&IoMem<'static>> {
        self.mcq.as_ref().ok_or(ENODEV)?.access(self.device())
    }

    pub(crate) fn variant(&self) -> &dyn UfsVariantOps {
        &*self.variant
    }
}
