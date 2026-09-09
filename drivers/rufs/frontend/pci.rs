// SPDX-License-Identifier: GPL-2.0

//! PCI frontend for the UFS driver.

use kernel::{device::Core, pci, prelude::*};

use crate::host::UfsHost;
use crate::resource::{HciMmio, HostResources};
use crate::variant::UfsVariantOps;

#[derive(Clone, Copy)]
pub(crate) enum UfsPciVariant {
    Qemu,
    Samsung,
    IntelCnl,
    IntelEhl,
    IntelLkf,
    IntelAdl,
    IntelMtl,
}

impl UfsPciVariant {
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

impl UfsVariantOps for UfsPciVariant {}

kernel::pci_device_table!(
    PCI_TABLE,
    <UfsPci as pci::Driver>::IdInfo,
    [
        // Match the PCI IDs handled by drivers/ufs/host/ufshcd-pci.c.
        (
            pci::DeviceId::from_id(pci::Vendor::REDHAT, 0x0013),
            UfsPciVariant::Qemu,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::SAMSUNG, 0xc00c),
            UfsPciVariant::Samsung,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x9dfa),
            UfsPciVariant::IntelCnl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x4b41),
            UfsPciVariant::IntelEhl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x4b43),
            UfsPciVariant::IntelEhl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x98fa),
            UfsPciVariant::IntelLkf,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x51ff),
            UfsPciVariant::IntelAdl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x54ff),
            UfsPciVariant::IntelAdl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x7e47),
            UfsPciVariant::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0xa847),
            UfsPciVariant::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x7747),
            UfsPciVariant::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0xe447),
            UfsPciVariant::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0x4d47),
            UfsPciVariant::IntelMtl,
        ),
        (
            pci::DeviceId::from_id(pci::Vendor::INTEL, 0xd335),
            UfsPciVariant::IntelMtl,
        ),
    ]
);

pub(crate) struct UfsPci;

#[pin_data]
pub(crate) struct UfsPciData<'a> {
    pdev: &'a pci::Device,
    #[pin]
    host: UfsHost<'a>,
    irq_vectors: pci::IrqVectorRegistration<'a>,
}

impl pci::Driver for UfsPci {
    type IdInfo = UfsPciVariant;
    type Data<'a> = UfsPciData<'a>;
    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe<'a>(
        pdev: &'a pci::Device<Core<'_>>,
        platform: Option<&'a Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'a>, Error> + 'a {
        pin_init::pin_init_scope(move || {
            let platform = platform.ok_or(ENODEV)?;
            pr_info!(
                "rufs: probe: platform={} vendor={} device=0x{:04x}\n",
                platform.name(),
                pdev.vendor_id(),
                pdev.device_id(),
            );

            pdev.enable_device_mem()?;
            pdev.set_master();

            let variant = KBox::new(*platform, GFP_KERNEL)? as KBox<dyn UfsVariantOps>;
            let resources = HostResources::new(
                pdev.as_ref().into(),
                HciMmio::from_pci(pdev)?,
                None,
                variant,
            )?;
            // Until MCQ ESI routing is implemented, all independently
            // registered handlers share the controller IRQ. Per-CQ actions
            // still provide queue-local completion dispatch for MCQ.
            let irq_vectors = pdev.alloc_irq_vectors(1, 1, pci::IrqTypes::all())?;
            // SAFETY: `irq_vectors` is stored after `host`, so the host drops
            // all IRQ registrations before the vector allocation is freed.
            let uic_irq = unsafe { irq_vectors.request(0)? };
            // SAFETY: Same lifetime and field-order guarantee as above.
            let queue_irq = unsafe { irq_vectors.request(0)? };
            // SAFETY: Same lifetime and field-order guarantee as above.
            let mcq_irq = unsafe { irq_vectors.request(0)? };
            let host = UfsHost::new(resources, uic_irq, queue_irq).pin_chain(move |host| {
                host.request_mcq_queue_irqs(|| {
                    // SAFETY: `irq_vectors` remains stored after `host`, so
                    // this IRQ stays allocated for every per-CQ registration.
                    Ok(unsafe { mcq_irq.duplicate() })
                })
            });

            Ok(try_pin_init!(UfsPciData {
                pdev,
                host <- host,
                irq_vectors,
            }))
        })
    }

    fn unbind(_pdev: &pci::Device<Core<'_>>, this: Pin<&Self::Data<'_>>) {
        this.host.shutdown();
    }
}
