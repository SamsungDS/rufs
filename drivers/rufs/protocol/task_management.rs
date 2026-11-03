// SPDX-License-Identifier: GPL-2.0

//! UFS task management protocol definitions.

#![allow(dead_code)]

use crate::command::TaskTag;
use kernel::prelude::*;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum TaskManagementFunction {
    AbortTask = 0x01,
    AbortTaskSet = 0x02,
    ClearTaskSet = 0x04,
    LogicalReset = 0x08,
    QueryTask = 0x80,
    QueryTaskSet = 0x81,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct TaskManagementRequest {
    function: TaskManagementFunction,
    lun: u8,
    task_tag: TaskTag,
}

impl TaskManagementRequest {
    pub(crate) const fn new(
        function: TaskManagementFunction,
        lun: u8,
        task_tag: TaskTag,
    ) -> Self {
        Self {
            function,
            lun,
            task_tag,
        }
    }

    pub(crate) const fn function(self) -> TaskManagementFunction {
        self.function
    }

    pub(crate) const fn lun(self) -> u8 {
        self.lun
    }

    pub(crate) const fn task_tag(self) -> TaskTag {
        self.task_tag
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum TaskManagementServiceResponse {
    FunctionComplete,
    FunctionNotSupported,
    FunctionFailed,
    FunctionSucceeded,
    IncorrectLogicalUnitNumber,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub(crate) enum TaskManagementCommandResult {
    Success = 0x00,
    Failure = 0x01,
}

impl TryFrom<u8> for TaskManagementServiceResponse {
    type Error = Error;

    fn try_from(response: u8) -> Result<Self> {
        match response {
            0x00 => Ok(Self::FunctionComplete),
            0x04 => Ok(Self::FunctionNotSupported),
            0x05 => Ok(Self::FunctionFailed),
            0x08 => Ok(Self::FunctionSucceeded),
            0x09 => Ok(Self::IncorrectLogicalUnitNumber),
            _ => Err(EIO),
        }
    }
}
