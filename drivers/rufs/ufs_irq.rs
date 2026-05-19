// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use kernel::prelude::*;
use kernel::sync::Arc;

#[pin_data]
pub(crate) struct UfsIrq {
    #[pin]
    placeholder: u32,
}

impl UfsIrq {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                placeholder: 0,
            }), GFP_KERNEL,
        )
    }
}
