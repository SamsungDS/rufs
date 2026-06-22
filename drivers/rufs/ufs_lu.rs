// SPDX-License-Identifier: GPL-2.0

//! Per-logical-unit state for the Rust UFS driver.

#![allow(dead_code)]

use kernel::block::{
    error::{code, BlkResult},
    mq::{self, gen_disk::GenDisk, gen_disk::GenDiskBuilder, IdleRequest, Operations, TagSet},
    SECTOR_SIZE,
};
use kernel::bindings;
use kernel::sync::{Arc, ArcBorrow, Mutex, SpinLock};
use kernel::types::{ARef, OwnableRefCounted, Owned};
use kernel::{new_mutex, new_spinlock, prelude::*};
use crate::ufs_queue::*;

const SECTOR_SIZE_U64: u64 = SECTOR_SIZE as u64;
const MAX_DISCARD_SEGMENTS: u16 = 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum UfsLuState {
    Reset,
    Operational,
    Error,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct UfsLuGeometry {
    logical_block_size: u32,
    physical_block_size: u32,
    alignment: u32,
    capacity_blocks: u64,
    sectors_per_block: u64,
}

impl UfsLuGeometry {
    pub(crate) fn new(
        logical_block_size: u32,
        physical_block_size: u32,
        alignment: u32,
        capacity_blocks: u64,
    ) -> Result<Self> {
        if logical_block_size < SECTOR_SIZE
            || logical_block_size % SECTOR_SIZE != 0
            || !logical_block_size.is_power_of_two()
        {
            return Err(EINVAL);
        }

        if physical_block_size != 0
            && (physical_block_size < logical_block_size
                || physical_block_size % logical_block_size != 0)
        {
            return Err(EINVAL);
        }

        Ok(Self {
            logical_block_size,
            physical_block_size: if physical_block_size == 0 {
                logical_block_size
            } else {
                physical_block_size
            },
            alignment,
            capacity_blocks,
            sectors_per_block: u64::from(logical_block_size / SECTOR_SIZE),
        })
    }

    pub(crate) fn from_logical_block_shift(
        logical_block_shift: u8,
        capacity_blocks: u64,
    ) -> Result<Self> {
        let logical_block_size = 1u32
            .checked_shl(u32::from(logical_block_shift))
            .ok_or(EINVAL)?;

        Self::new(logical_block_size, logical_block_size, 0, capacity_blocks)
    }

    pub(crate) fn logical_block_size(&self) -> u32 {
        self.logical_block_size
    }

    pub(crate) fn physical_block_size(&self) -> u32 {
        self.physical_block_size
    }

    pub(crate) fn alignment(&self) -> u32 {
        self.alignment
    }

    pub(crate) fn capacity_blocks(&self) -> u64 {
        self.capacity_blocks
    }

    pub(crate) fn sectors_per_block(&self) -> u64 {
        self.sectors_per_block
    }

    pub(crate) fn capacity_sectors(&self) -> Option<u64> {
        self.capacity_blocks.checked_mul(self.sectors_per_block)
    }

    pub(crate) fn max_discard_sectors(&self) -> Result<u32> {
        let sectors_per_block = u32::try_from(self.sectors_per_block).map_err(|_| EOVERFLOW)?;
        let remainder = u32::MAX.checked_rem(sectors_per_block).ok_or(EINVAL)?;

        u32::MAX.checked_sub(remainder).ok_or(EOVERFLOW)
    }

    pub(crate) fn sectors_to_logical(&self, sectors: u64) -> u64 {
        sectors / self.sectors_per_block
    }

    pub(crate) fn logical_to_sectors(&self, blocks: u64) -> Option<u64> {
        blocks.checked_mul(self.sectors_per_block)
    }

    pub(crate) fn bytes_to_logical(&self, bytes: u64) -> u64 {
        self.sectors_to_logical(bytes / SECTOR_SIZE_U64)
    }

    pub(crate) fn logical_to_bytes(&self, blocks: u64) -> Option<u64> {
        self.logical_to_sectors(blocks)?.checked_mul(SECTOR_SIZE_U64)
    }
}

#[pin_data]
pub(crate) struct UfsLu {
    queue: Arc<UfsQueue>,
    lun: u8,
    geometry: UfsLuGeometry,
    hw_queue_depth: usize,
    queue_depth: usize,

    #[pin]
    state: SpinLock<UfsLuState>,

    #[pin]
    disk: Mutex<Option<Arc<GenDisk<UfsLuBlockOps>>>>,
}

impl UfsLu {
    pub(crate) fn new(
        queue: Arc<UfsQueue>,
        lun: u8,
        geometry: UfsLuGeometry,
        hw_queue_depth: usize,
        queue_depth: usize,
    ) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                queue,
                lun,
                geometry,
                hw_queue_depth,
                queue_depth,
                state <- new_spinlock!(UfsLuState::Reset),
                disk <- new_mutex!(None),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn init_disk(self: &Arc<Self>, tagset: Arc<TagSet<UfsLuBlockOps>>) -> Result<()> {
        let capacity_sectors = self.geometry.capacity_sectors().ok_or(EOVERFLOW)?;
        let disk = GenDiskBuilder::new()
            .logical_block_size(self.geometry.logical_block_size())?
            .physical_block_size(self.geometry.physical_block_size())?
            .max_hw_discard_sectors(self.geometry.max_discard_sectors()?)
            .discard_granularity(self.geometry.logical_block_size())
            .max_discard_segments(MAX_DISCARD_SEGMENTS)
            .queue_depth(u32::try_from(self.queue_depth).map_err(|_| EOVERFLOW)?)?
            .capacity_sectors(capacity_sectors)
            .build(fmt!("ufs{}", self.lun), tagset, self.clone())?;

        let mut current = self.disk.lock();
        if current.is_some() {
            return Err(EBUSY);
        }

        current.replace(disk);
        self.set_state(UfsLuState::Operational);
        Ok(())
    }

    pub(crate) fn lun(&self) -> u8 {
        self.lun
    }

    pub(crate) fn geometry(&self) -> UfsLuGeometry {
        self.geometry
    }

    pub(crate) fn state(&self) -> UfsLuState {
        *self.state.lock()
    }

    pub(crate) fn set_state(&self, state: UfsLuState) {
        *self.state.lock() = state;
    }

    pub(crate) fn is_operational(&self) -> bool {
        self.state() == UfsLuState::Operational
    }

    fn build_scsi_cmd(&self, op: bindings::req_op, lba: u64, blocks: u64) -> Result<UfsSCSICmd> {
        let blocks = u32::try_from(blocks).map_err(|_| EINVAL)?;
        let data_len = u32::try_from(
            u64::from(self.geometry.logical_block_size())
                .checked_mul(u64::from(blocks))
                .ok_or(EOVERFLOW)?,
        )
        .map_err(|_| EOVERFLOW)?;

        match op {
            bindings::req_op_REQ_OP_READ => {
                Ok(UfsSCSICmd::read_write(self.lun, false, lba, blocks, data_len, false))
            }
            bindings::req_op_REQ_OP_WRITE => {
                Ok(UfsSCSICmd::read_write(self.lun, true, lba, blocks, data_len, false))
            }
            bindings::req_op_REQ_OP_FLUSH => Ok(UfsSCSICmd::flush(self.lun)),
            bindings::req_op_REQ_OP_DISCARD => {
                Ok(UfsSCSICmd::unmap(self.lun, lba, blocks))
            }
            _ => Err(ENOTSUPP),
        }
    }

    fn acquire_request(&self, hw_queue: usize, tag: usize) -> Result<Arc<UfsRequest>> {
        let tag = self.global_tag(hw_queue, tag)?;
        self.queue.acquire(tag)
    }

    fn global_tag(&self, hw_queue: usize, tag: usize) -> Result<usize> {
        if tag >= self.hw_queue_depth {
            return Err(EINVAL);
        }

        hw_queue
            .checked_mul(self.hw_queue_depth)
            .and_then(|base| base.checked_add(tag))
            .ok_or(EOVERFLOW)
    }

    fn timeout_request(&self, hw_queue: usize, tag: usize) -> mq::RequestTimeoutStatus {
        let global_tag = match self.global_tag(hw_queue, tag) {
            Ok(global_tag) => global_tag,
            Err(e) => {
                pr_err!(
                    "[RUFS] ufs_lu: invalid timeout request tag={} hctx={} errno={}\n",
                    tag,
                    hw_queue,
                    e.to_errno(),
                );
                return mq::RequestTimeoutStatus::RetryLater;
            },
        };

        if self.queue.timeout(global_tag) {
            mq::RequestTimeoutStatus::Completed
        } else {
            mq::RequestTimeoutStatus::RetryLater
        }
    }
}

pub(crate) struct UfsLuBlockOps;

// Hand a request that never reached the device back to the block layer.
//
// The request reference is still held by the slot, so reclaim it and complete
// it once. Because the command was not submitted, no completion can race for
// the request.
fn complete_unsubmitted(request: &Arc<UfsRequest>, e: Error) {
    let Some(rq) = request.take_block_request() else {
        return;
    };

    let rq = OwnableRefCounted::try_from_shared(rq)
        .map_err(|_e| kernel::error::code::EIO)
        .expect("Failed to complete request");

    if e == EBUSY {
        rq.requeue(true);
    } else {
        rq.end(bindings::BLK_STS_IOERR as u8);
    }
}

#[vtable]
impl Operations for UfsLuBlockOps {
    type RequestData = ();
    type QueueData = Arc<UfsLu>;
    type HwData = KBox<u32>;
    type TagSetData = KBox<UfsQueueMap>;

    fn new_request_data() -> impl PinInit<Self::RequestData> {}

    fn queue_rq(
        _hw_data: &u32,
        lu: ArcBorrow<'_, UfsLu>,
        rq: Owned<IdleRequest<Self>>,
        _is_last: bool,
    ) -> BlkResult {
        let op = rq.command() as bindings::req_op;
        let sector = rq.sector();
        let sectors = rq.sectors();
        let geometry = lu.geometry();
        let mask = geometry.sectors_per_block() - 1;
        let tag = usize::try_from(rq.tag()).map_err(|_| EINVAL)?;
        let hw_queue = usize::try_from(rq.queue_index()).map_err(|_| EINVAL)?;

        let cmd = match op {
            bindings::req_op_REQ_OP_READ | bindings::req_op_REQ_OP_WRITE => {
                if sectors == 0 {
                    pr_debug!("[RUFS] ufs_lu: zero-length request on LU {}\n", lu.lun());
                    rq.start().end_ok();
                    return Ok(());
                }

                if sector.checked_add(u64::from(sectors)).ok_or(EINVAL)?
                    > geometry.capacity_sectors().ok_or(EOVERFLOW)?
                {
                    pr_warn!(
                        "[RUFS] ufs_lu: request exceeds LU {} capacity sector={} sectors={}\n",
                        lu.lun(),
                        sector,
                        sectors,
                    );
                    rq.start().end(bindings::BLK_STS_INVAL as u8);
                    return Ok(());
                }

                if (sector & mask) != 0 || (u64::from(sectors) & mask) != 0 {
                    pr_warn!(
                        "[RUFS] ufs_lu: unaligned request on LU {} sector={} sectors={} spb={}\n",
                        lu.lun(),
                        sector,
                        sectors,
                        geometry.sectors_per_block(),
                    );
                    rq.start().end(bindings::BLK_STS_INVAL as u8);
                    return Ok(());
                }

                let lba = geometry.sectors_to_logical(sector);
                let blocks = geometry.sectors_to_logical(u64::from(sectors));
                let cmd = lu.build_scsi_cmd(op, lba, blocks)?;

                pr_debug!(
                    "[RUFS] ufs_lu: LU {} op={} lba={} blocks={}\n",
                    lu.lun(),
                    op,
                    lba,
                    blocks,
                );

                cmd
            }
            bindings::req_op_REQ_OP_FLUSH => {
                pr_debug!("[RUFS] ufs_lu: flush request on LU {}\n", lu.lun());
                lu.build_scsi_cmd(op, 0, 0)?
            }
            bindings::req_op_REQ_OP_DISCARD => {
                if sectors == 0 {
                    pr_debug!("[RUFS] ufs_lu: zero-length discard on LU {}\n", lu.lun());
                    rq.start().end_ok();
                    return Ok(());
                }

                if sector.checked_add(u64::from(sectors)).ok_or(EINVAL)?
                    > geometry.capacity_sectors().ok_or(EOVERFLOW)?
                {
                    pr_warn!(
                        "[RUFS] ufs_lu: discard exceeds LU {} capacity sector={} sectors={}\n",
                        lu.lun(),
                        sector,
                        sectors,
                    );
                    rq.start().end(bindings::BLK_STS_INVAL as u8);
                    return Ok(());
                }

                if (sector & mask) != 0 || (u64::from(sectors) & mask) != 0 {
                    pr_warn!(
                        "[RUFS] ufs_lu: unaligned discard on LU {} sector={} sectors={} spb={}\n",
                        lu.lun(),
                        sector,
                        sectors,
                        geometry.sectors_per_block(),
                    );
                    rq.start().end(bindings::BLK_STS_INVAL as u8);
                    return Ok(());
                }

                let lba = geometry.sectors_to_logical(sector);
                let blocks = geometry.sectors_to_logical(u64::from(sectors));
                let cmd = lu.build_scsi_cmd(op, lba, blocks)?;

                pr_debug!(
                    "[RUFS] ufs_lu: discard LU {} lba={} blocks={}\n",
                    lu.lun(),
                    lba,
                    blocks,
                );

                cmd
            }
            _ => {
                pr_warn!(
                    "[RUFS] ufs_lu: unsupported request op={} on LU {}\n",
                    op,
                    lu.lun(),
                );
                rq.start().end(bindings::BLK_STS_NOTSUPP as u8);
                return Ok(());
            }
        };

        // Resolve the request slot for this tag before taking shared ownership
        // of the block request. While the request is still an `IdleRequest` it
        // is owned by the block layer (refcount zero), so a not-yet-recycled
        // slot can be handed back to the block layer without leaking a
        // reference.
        //
        // The completion path frees the blk-mq tag (allowing this tag to be
        // dispatched again) just before it resets the slot to idle, so a fresh
        // dispatch can briefly observe the slot as still busy. Signal that with
        // `BLK_STS_DEV_RESOURCE` so the block layer retries, rather than
        // converting the busy error into `BLK_STS_IOERR` through `?`.
        let request = match lu.acquire_request(hw_queue, tag) {
            Ok(request) => request,
            Err(e) if e == EBUSY => return Err(code::BLK_STS_DEV_RESOURCE),
            Err(e) => return Err(e.into()),
        };

        // From here the driver takes shared ownership of the request and is
        // responsible for completing it exactly once. The completion path
        // reclaims unique ownership of this single reference, so the driver
        // must never keep a second one while the command is in flight.
        let rq = OwnableRefCounted::into_shared(rq.start());
        if let Err(e) = request.compose_block_request(rq, cmd, hw_queue) {
            complete_unsubmitted(&request, e);
            return Ok(());
        }

        if let Err(e) = request.submit() {
            complete_unsubmitted(&request, e);
        }

        Ok(())
    }

    fn commit_rqs(_hw_data: &u32, _queue_data: ArcBorrow<'_, UfsLu>) {}

    fn init_hctx(_tagset_data: &UfsQueueMap, hctx_idx: u32) -> Result<Self::HwData> {
        // Remember which hardware queue this context drives, so `poll` can find
        // the matching backend completion queue.
        Ok(KBox::new(hctx_idx, GFP_KERNEL)?)
    }

    fn complete(rq: ARef<mq::Request<Self>>) {
        OwnableRefCounted::try_from_shared(rq)
            .map_err(|_| EBUSY)
            .expect("rufs: request completion failed")
            .end_ok();
    }

    fn request_timeout(
        tag_set: &TagSet<Self>,
        queue_id: u32,
        tag: u32,
    ) -> mq::RequestTimeoutStatus {
        let Some(request) = tag_set.tag_to_rq(queue_id, tag) else {
            pr_err!(
                "[RUFS] ufs_lu: timeout for unknown request hctx={} tag={}\n",
                queue_id,
                tag,
            );
            return mq::RequestTimeoutStatus::RetryLater;
        };

        request
            .queue_data()
            .timeout_request(queue_id as usize, tag as usize)
    }

    fn poll(
        hw_data: &u32,
        queue_data: ArcBorrow<'_, UfsLu>,
        batch: &mut mq::IoCompletionBatch<Self>,
    ) -> Result<bool> {
        Ok(queue_data.queue.poll(*hw_data as usize, batch))
    }

    fn map_queues(tag_set: Pin<&mut TagSet<Self>>) {
        let layout = *tag_set.data();
        let default_queues = layout.default_queues() as u32;
        let read_queues = layout.read_queues() as u32;
        let poll_queues = layout.poll_queues() as u32;

        let mut offset = 0;
        let result = tag_set.update_maps(|mut qmap| {
            let queue_count = match qmap.kind() {
                mq::QueueType::Default => default_queues,
                mq::QueueType::Read => read_queues,
                mq::QueueType::Poll => poll_queues,
            };
            qmap.set_queue_count(queue_count);
            qmap.set_offset(offset);
            offset += queue_count;
            qmap.map_queues();
        });

        if result.is_err() {
            pr_err!("[RUFS] ufs_lu: failed to update blk-mq queue maps\n");
        }
    }
}
