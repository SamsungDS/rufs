// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use kernel::time::{Delta, delay};
use kernel::{prelude::*, new_mutex};
use kernel::sync::{Arc, Mutex};
use crate::ufs_dma::DescBuffer;
use crate::ufs_queue::*;

const NOP_OUT_TIMEOUT_MS: i64 = 50;
const QUERY_DEFAULT_TIMEOUT_MS: i64 = 1500;
const ADVANCDE_RPMB_TIMEOUT_MS: i64 = 3000;

#[derive(Copy, Clone)]
pub(crate) struct UfsQueryCmd {}

#[derive(Copy, Clone)]
pub(crate) struct UfsRPMBCmd {}

#[derive(Copy, Clone)]
pub(crate) enum UfsDevCmd {
    Nop,
    Query(UfsQueryCmd),
    RPMB(UfsRPMBCmd),
}

impl UfsDevCmd {
    fn nop() -> UfsCmd { UfsCmd::Device(Self::Nop) }

    pub(crate) fn timeout(&self) -> Delta {
        match *self {
            Self::Nop => Delta::from_millis(NOP_OUT_TIMEOUT_MS),
            Self::Query(_) => Delta::from_millis(QUERY_DEFAULT_TIMEOUT_MS),
            Self::RPMB(_) => Delta::from_millis(ADVANCDE_RPMB_TIMEOUT_MS),
        }
    }
}

#[pin_data]
pub(crate) struct UfsDev {
    #[pin]
    request: Mutex<Arc<UfsRequest>>,
}

impl UfsDev{
    pub(crate) fn new(queue: Arc<UfsQueue>) -> Result<Arc<Self>> {
        let request = queue.reserve()?;
        Arc::pin_init(
            try_pin_init!(Self {
                request <- new_mutex!(request),
            }),
            GFP_KERNEL,
        )
    }

    fn nop(&self) -> Result<()> {
        let request = self.request.lock();
        let cmd = request.issue(UfsDevCmd::nop())?;
        Ok(())
    }

    pub(crate) fn verify_dev_init(&self) -> Result<()> {
        self.nop()?;
        pr_info!("[RUFS] ufs_dev: device verified");
        Ok(())
    }
}
