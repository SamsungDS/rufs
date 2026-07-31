// SPDX-License-Identifier: GPL-2.0

//! Qualcomm platform frontend for the UFS driver.

use kernel::{device::Core, of, platform, prelude::*};

pub(crate) struct UfsQcom;

#[derive(Clone, Copy)]
pub(crate) enum UfsQcomVariant {
    Generic,
    Sm8550,
    Sa8255p,
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <UfsQcom as platform::Driver>::IdInfo,
    [
        (
            of::DeviceId::new(c"qcom,ufshc"),
            UfsQcomVariant::Generic,
        ),
        (
            of::DeviceId::new(c"qcom,sm8550-ufshc"),
            UfsQcomVariant::Sm8550,
        ),
        (
            of::DeviceId::new(c"qcom,sm8650-ufshc"),
            UfsQcomVariant::Sm8550,
        ),
        (
            of::DeviceId::new(c"qcom,sa8255p-ufshc"),
            UfsQcomVariant::Sa8255p,
        ),
    ]
);

impl platform::Driver for UfsQcom {
    type IdInfo = UfsQcomVariant;
    type Data<'bound> = Self;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        _pdev: &'bound platform::Device<Core<'_>>,
        _variant: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        // Keep the match and registration boundary buildable without touching
        // hardware until the Qualcomm resource and lifecycle sequence exists.
        Err(ENODEV)
    }
}
