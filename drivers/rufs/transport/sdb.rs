// SPDX-License-Identifier: GPL-2.0

//! UFSHCI single doorbell transport.

use crate::command::TaskTag;
use crate::reg::UfsReg;
use crate::transport::{
    CompletionBatch, RequestDmaAccessEnded, SubmissionOutcome, UfsHwQueue,
    UfsTransferOps,
};
use kernel::sync::{barrier, Arc, SpinLockIrq};
use kernel::{new_spinlock_irq, prelude::*};

const SDB_TASK_ABORT_CLEAR_TIMEOUT_MS: i64 = 1000;

pub(crate) struct SdbRequestsDmaAccessEnded {
    task_mask: u32,
}

impl SdbRequestsDmaAccessEnded {
    pub(crate) fn take(&mut self, task_tag: TaskTag) -> Result<RequestDmaAccessEnded> {
        let mask = 1u32
            .checked_shl(u32::from(task_tag.value()))
            .ok_or(EINVAL)?;
        if self.task_mask & mask == 0 {
            return Err(ENOENT);
        }
        self.task_mask &= !mask;
        Ok(RequestDmaAccessEnded::new(task_tag))
    }

    pub(crate) fn contains(&self, task_tag: TaskTag) -> bool {
        1u32
            .checked_shl(u32::from(task_tag.value()))
            .is_some_and(|mask| self.task_mask & mask != 0)
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.task_mask == 0
    }

    pub(crate) fn len(&self) -> usize {
        self.task_mask.count_ones() as usize
    }
}

pub(crate) struct SdbTransferBackend {
    reg: Arc<UfsReg>,
    state: Arc<SdbTransferState>,
}

#[derive(Default)]
struct SdbCompletionState {
    outstanding: u32,
    polled: u32,
}

#[derive(Copy, Clone)]
enum SdbCompletionSource {
    Interrupt,
    Poll,
    Recovery,
}

#[pin_data]
struct SdbTransferState {
    #[pin]
    completion: SpinLockIrq<SdbCompletionState>,
}

#[derive(Clone)]
pub(crate) struct SdbHwQueue {
    reg: Arc<UfsReg>,
    state: Arc<SdbTransferState>,
}

impl SdbHwQueue {
    pub(crate) fn id(&self) -> u32 {
        0
    }

    pub(crate) fn submit<F>(&self, tag: u32, polled: bool, publish: F) -> SubmissionOutcome
    where
        F: FnOnce() -> Result<()>,
    {
        let Some(mask) = SdbTransferBackend::tag_mask(tag) else {
            return SubmissionOutcome::NotSubmitted(EINVAL);
        };
        let mut state = self.state.completion.lock();

        if state.outstanding & mask != 0 {
            return SubmissionOutcome::NotSubmitted(EBUSY);
        }
        if let Err(e) = publish() {
            return SubmissionOutcome::NotSubmitted(e);
        }
        state.outstanding |= mask;
        if polled {
            state.polled |= mask;
        } else {
            state.polled &= !mask;
        }
        barrier::dma_mb(barrier::Write);
        self.reg.ring_utrl_doorbell(tag);
        SubmissionOutcome::Submitted
    }

    pub(crate) fn poll(&self, completed: &mut CompletionBatch) -> Result<()> {
        SdbTransferBackend::collect_state_completions(
            &self.reg,
            &self.state,
            SdbCompletionSource::Poll,
            completed,
        )
    }
}

impl SdbTransferBackend {
    pub(crate) fn new(reg: Arc<UfsReg>) -> Result<Self> {
        let state = Arc::pin_init(
            pin_init!(SdbTransferState {
                completion <- new_spinlock_irq!(SdbCompletionState::default()),
            }),
            GFP_KERNEL,
        )?;

        Ok(Self { reg, state })
    }

    fn tag_mask(tag: u32) -> Option<u32> {
        1u32.checked_shl(tag)
    }

    fn collect_state_completions(
        reg: &UfsReg,
        state: &SdbTransferState,
        source: SdbCompletionSource,
        requests: &mut CompletionBatch,
    ) -> Result<()> {
        let mut state = state.completion.lock();
        let doorbell = reg.read_utrl_doorbell();
        let completed = !doorbell & state.outstanding;
        let eligible = match source {
            SdbCompletionSource::Interrupt => completed & !state.polled,
            SdbCompletionSource::Poll => completed & state.polled,
            SdbCompletionSource::Recovery => completed,
        };
        if eligible != 0 {
            barrier::dma_mb(barrier::Read);
        }
        let collected = requests.insert_sdb_mask(eligible)?;

        state.outstanding &= !collected;
        state.polled &= !collected;
        Ok(())
    }
}

impl UfsTransferOps for SdbTransferBackend {
    fn hw_queues(&self) -> Result<KVec<UfsHwQueue>> {
        let mut queues = KVec::new();
        queues.push(
            UfsHwQueue::sdb(SdbHwQueue {
                reg: self.reg.clone(),
                state: self.state.clone(),
            }),
            GFP_KERNEL,
        )?;
        Ok(queues)
    }

    fn collect_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        Self::collect_state_completions(
            &self.reg,
            &self.state,
            SdbCompletionSource::Interrupt,
            completed,
        )
    }

    fn collect_recovery_completions(&self, completed: &mut CompletionBatch) -> Result<()> {
        Self::collect_state_completions(
            &self.reg,
            &self.state,
            SdbCompletionSource::Recovery,
            completed,
        )
    }

    fn enable_interrupts(&self) {
        self.reg.enable_transfer_interrupts()
    }

    fn reset(&self) -> Result<()> {
        *self.state.completion.lock() = SdbCompletionState::default();
        Ok(())
    }

    fn supports_sdb_task_abort(&self) -> bool {
        true
    }

    fn claim_sdb_task(&self, task_tag: TaskTag) -> Result<()> {
        let mask = Self::tag_mask(u32::from(task_tag.value())).ok_or(EINVAL)?;
        let mut state = self.state.completion.lock();

        if state.outstanding & mask == 0 {
            return Err(ENOENT);
        }
        if self.reg.read_utrl_doorbell() & mask == 0 {
            return Err(EAGAIN);
        }

        // This is the irrevocable ABORT TASK decision boundary. From here on,
        // completion collection must not treat the target transfer as a valid
        // completion, even if composing or publishing the TM request later
        // fails. Recovery owns it until UTRLCLR or a stronger stop proof shows
        // that the controller has released the slot.
        state.outstanding &= !mask;
        state.polled &= !mask;
        Ok(())
    }

    fn finish_sdb_task(&self, task_tag: TaskTag) -> Result<RequestDmaAccessEnded> {
        let slot = u32::from(task_tag.value());
        self.reg.clear_utrl_slot(slot)?;
        self.reg
            .wait_for_utrl_slot_clear(slot, 1000, SDB_TASK_ABORT_CLEAR_TIMEOUT_MS)?;
        Ok(RequestDmaAccessEnded::new(task_tag))
    }

    fn supports_sdb_logical_reset(&self) -> bool {
        true
    }

    fn claim_sdb_logical_unit(
        &self,
        task_mask: u32,
        task_claimed_mask: u32,
    ) -> Result<()> {
        if task_mask == 0
            || task_claimed_mask & !task_mask != 0
            || task_claimed_mask.count_ones() > 1
        {
            return Err(EINVAL);
        }
        let additional_mask = task_mask & !task_claimed_mask;
        let mut state = self.state.completion.lock();
        if state.outstanding & task_claimed_mask != 0
            || state.polled & task_claimed_mask != 0
        {
            return Err(EIO);
        }
        if state.outstanding & additional_mask != additional_mask {
            return Err(ENOENT);
        }
        if self.reg.read_utrl_doorbell() & additional_mask != additional_mask {
            return Err(EAGAIN);
        }

        // LOGICAL RESET invalidates every task in the addressed LU. Exclude
        // all matching SDB slots from normal completion collection as one
        // commitment before publishing the task-management request.
        state.outstanding &= !additional_mask;
        state.polled &= !additional_mask;
        Ok(())
    }

    fn finish_sdb_logical_unit(
        &self,
        task_mask: u32,
    ) -> Result<SdbRequestsDmaAccessEnded> {
        self.reg.clear_utrl_slots(task_mask)?;
        self.reg.wait_for_utrl_slots_clear(
            task_mask,
            1000,
            SDB_TASK_ABORT_CLEAR_TIMEOUT_MS,
        )?;
        Ok(SdbRequestsDmaAccessEnded {
            task_mask,
        })
    }

    fn dump_state(&self, tag: usize, reason: &str) {
        let state = self.state.completion.lock();

        pr_err!(
            "[RUFS] ufs_queue: SDB dump reason={} tag={} outstanding=0x{:x} polled=0x{:x}\n",
            reason,
            tag,
            state.outstanding,
            state.polled,
        );
    }
}
