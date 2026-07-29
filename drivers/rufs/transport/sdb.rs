// SPDX-License-Identifier: GPL-2.0

//! UFSHCI single doorbell transport.

use crate::reg::UfsReg;
use crate::transport::{CompletedRequests, SubmissionOutcome, UfsHwQueue, UfsTransferOps};
use kernel::sync::{barrier, Arc, SpinLockIrq};
use kernel::{new_spinlock_irq, prelude::*};

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
        barrier::dma_wmb();
        self.reg.ring_utrl_doorbell(tag);
        SubmissionOutcome::Submitted
    }

    pub(crate) fn poll(&self, completed: &mut CompletedRequests) -> Result<()> {
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
        requests: &mut CompletedRequests,
    ) -> Result<()> {
        let mut state = state.completion.lock();
        let doorbell = reg.read_utrl_doorbell();
        let completed = !doorbell & state.outstanding;
        let eligible = match source {
            SdbCompletionSource::Interrupt => completed & !state.polled,
            SdbCompletionSource::Poll => completed & state.polled,
        };
        if eligible != 0 {
            barrier::dma_rmb();
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

    fn collect_completions(&self, completed: &mut CompletedRequests) -> Result<()> {
        Self::collect_state_completions(
            &self.reg,
            &self.state,
            SdbCompletionSource::Interrupt,
            completed,
        )
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
