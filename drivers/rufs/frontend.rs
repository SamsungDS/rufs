// SPDX-License-Identifier: GPL-2.0

//! UFS host-controller bus frontends.

#[cfg(CONFIG_RUFS_PCI)]
pub(crate) mod pci;
#[cfg(CONFIG_RUFS_QCOM)]
pub(crate) mod qcom;
