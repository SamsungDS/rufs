// SPDX-License-Identifier: GPL-2.0

//! UFS command tag representation.

use kernel::prelude::*;

pub(crate) const TASK_TAG_COUNT: usize = 1usize << u8::BITS;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct TaskTag(u8);

impl TaskTag {
    pub(crate) const fn from_value(tag: u8) -> Self {
        Self(tag)
    }

    pub(crate) fn new(tag: u32) -> Result<Self> {
        Ok(Self(u8::try_from(tag).map_err(|_| EINVAL)?))
    }

    pub(crate) fn value(self) -> u8 {
        self.0
    }

    pub(crate) fn index(self) -> usize {
        usize::from(self.0)
    }
}
