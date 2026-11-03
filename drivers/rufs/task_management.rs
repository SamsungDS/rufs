// SPDX-License-Identifier: GPL-2.0

//! UFS task management request execution.

#![allow(dead_code)]

use crate::command::TaskTag;
use crate::dma::UfsDma;
use crate::protocol::task_management::{
    TaskManagementFunction, TaskManagementRequest, TaskManagementServiceResponse,
};
use crate::reg::UfsReg;
use kernel::sync::{barrier, Arc, Completion, Mutex, SpinLock};
use kernel::time::Delta;
use kernel::{new_mutex, new_spinlock, prelude::*};

const TASK_MANAGEMENT_SLOT: u32 = 0;
const TASK_MANAGEMENT_TIMEOUT_MS: i64 = 100;
const TASK_MANAGEMENT_CLEAR_TIMEOUT_MS: i64 = 1000;

#[derive(Clone, Copy, Eq, PartialEq)]
enum TaskManagementState {
    Ready,
    Active,
    Unusable,
}

#[pin_data]
pub(crate) struct TaskManagement {
    reg: Arc<UfsReg>,
    dma: Arc<UfsDma>,

    #[pin]
    transaction: Mutex<()>,
    #[pin]
    state: SpinLock<TaskManagementState>,
    #[pin]
    completion: Completion,
}

impl TaskManagement {
    pub(crate) fn new(reg: Arc<UfsReg>, dma: Arc<UfsDma>) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                reg,
                dma,
                transaction <- new_mutex!(()),
                state <- new_spinlock!(TaskManagementState::Ready),
                completion <- Completion::new(),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn execute(
        &self,
        request: TaskManagementRequest,
    ) -> Result<TaskManagementServiceResponse> {
        let _transaction = self.transaction.lock();
        let slot = TASK_MANAGEMENT_SLOT as usize;
        let mask = 1u32 << TASK_MANAGEMENT_SLOT;

        match *self.state.lock() {
            TaskManagementState::Ready => {}
            TaskManagementState::Active => return Err(EBUSY),
            TaskManagementState::Unusable => return Err(EIO),
        }
        if self.reg.read_utmrl_doorbell() & mask != 0 {
            *self.state.lock() = TaskManagementState::Unusable;
            return Err(EIO);
        }

        self.completion.reinit();
        self.dma.compose_task_management_request(slot, request)?;
        *self.state.lock() = TaskManagementState::Active;

        barrier::dma_mb(barrier::Write);
        self.reg.ring_utmrl_doorbell(mask);

        let timeout = Delta::from_millis(TASK_MANAGEMENT_TIMEOUT_MS);
        if self.completion.wait_for_completion_timeout(timeout) == 0 {
            *self.state.lock() = TaskManagementState::Unusable;
            if self
                .reg
                .clear_utmrl_slot(TASK_MANAGEMENT_SLOT)
                .and_then(|_| {
                    self.reg.wait_for_utmrl_slot_clear(
                        TASK_MANAGEMENT_SLOT,
                        1000,
                        TASK_MANAGEMENT_CLEAR_TIMEOUT_MS,
                    )
                })
                .is_err()
            {
                pr_err!(
                    "rufs: failed to clear timed-out task management slot {}\n",
                    TASK_MANAGEMENT_SLOT,
                );
            }
            return Err(ETIMEDOUT);
        }

        barrier::dma_mb(barrier::Read);
        if self.reg.read_utmrl_doorbell() & mask != 0 {
            *self.state.lock() = TaskManagementState::Unusable;
            return Err(EIO);
        }
        let response = self
            .dma
            .fetch_task_management_response(slot, request);
        *self.state.lock() = TaskManagementState::Ready;
        response
    }

    pub(crate) fn query_task(
        &self,
        lun: u8,
        task_tag: TaskTag,
    ) -> Result<TaskManagementServiceResponse> {
        self.execute(TaskManagementRequest::new(
            TaskManagementFunction::QueryTask,
            lun,
            task_tag,
        ))
    }

    pub(crate) fn abort_task(
        &self,
        lun: u8,
        task_tag: TaskTag,
    ) -> Result<TaskManagementServiceResponse> {
        self.execute(TaskManagementRequest::new(
            TaskManagementFunction::AbortTask,
            lun,
            task_tag,
        ))
    }

    pub(crate) fn logical_reset(
        &self,
        lun: u8,
    ) -> Result<TaskManagementServiceResponse> {
        self.execute(TaskManagementRequest::logical_reset(lun))
    }

    pub(crate) fn complete(&self) {
        if *self.state.lock() == TaskManagementState::Active {
            self.completion.complete();
        }
    }

    pub(crate) fn reset_after_controller_reset(&self) -> Result<()> {
        let _transaction = self.transaction.lock();
        if self.reg.read_utmrl_doorbell() != 0 {
            return Err(EBUSY);
        }
        self.completion.reinit();
        *self.state.lock() = TaskManagementState::Ready;
        Ok(())
    }
}
