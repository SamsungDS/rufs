// SPDX-License-Identifier: GPL-2.0

//! UFS device protocol definitions.

use kernel::prelude::*;

use self::query::UfsDevCmd;
use self::scsi::UfsSCSICmd;

pub(crate) mod query;
pub(crate) mod scsi;
pub(crate) mod upiu;

#[derive(Copy, Clone)]
pub(crate) enum UfsCmd {
    Device(UfsDevCmd),
    SCSI(UfsSCSICmd),
}

impl UfsCmd {
    pub(crate) fn get_device(&self) -> Result<UfsDevCmd> {
        match *self {
            Self::Device(cmd) => Ok(cmd),
            _ => Err(EINVAL),
        }
    }
}
