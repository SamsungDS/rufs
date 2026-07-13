// SPDX-License-Identifier: GPL-2.0

//! Per-logical-unit state for the Rust UFS driver.

#![allow(dead_code)]

use crate::ufs_dma::{MAX_PRD_ENTRIES, PRDT_DATA_BYTE_COUNT_MAX};
use crate::ufs_queue::*;
use kernel::bindings;
use kernel::block::error::code::BLK_STS_IOERR;
use kernel::block::mq::gen_disk::BoundGenDisk;
use kernel::block::mq::LimitsBuilder;
use kernel::block::mq::RequestQueue;
use kernel::sync::atomic::{Acquire, Atomic, Relaxed};
use kernel::sync::{Arc, Mutex, SpinLock};
use kernel::types::{OwnableRefCounted, Owned};
use kernel::{
    block::{
        error::BlkResult,
        mq::{
            self, dma_map_iter::DmaMapMempool, gen_disk::GenDisk, IdleRequest, Operations, TagSet,
        },
        SECTOR_SIZE,
    },
    sync::aref::ARef,
};
use kernel::{new_mutex, new_spinlock, prelude::*};

const SECTOR_SIZE_U64: u64 = SECTOR_SIZE as u64;
const MAX_DISCARD_SEGMENTS: u16 = 1;
const MAX_SECTORS: u32 = 1024 * 1024 / 512;

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
        self.logical_to_sectors(blocks)?
            .checked_mul(SECTOR_SIZE_U64)
    }
}

#[pin_data]
pub(crate) struct UfsLu {
    pub(crate) queue: Arc<UfsQueue>,
    lun: u8,
    geometry: UfsLuGeometry,
    queue_depth: u32,

    #[pin]
    state: SpinLock<UfsLuState>,

    #[pin]
    disk: Mutex<Option<BoundGenDisk<UfsLuBlockOps>>>,
}

impl UfsLu {
    pub(crate) fn new(
        queue: Arc<UfsQueue>,
        lun: u8,
        geometry: UfsLuGeometry,
        queue_depth: u32,
    ) -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                queue,
                lun,
                geometry,
                queue_depth,
                state <- new_spinlock!(UfsLuState::Reset),
                disk <- new_mutex!(None),
            }),
            GFP_KERNEL,
        )
    }

    pub(crate) fn init_disk(self: &Arc<Self>) -> Result<()> {
        let capacity_sectors = self.geometry.capacity_sectors().ok_or(EOVERFLOW)?;

        let limits = LimitsBuilder::<UfsLuBlockOps>::new()
            .logical_block_size(self.geometry.logical_block_size())?
            .physical_block_size(self.geometry.physical_block_size())?
            .max_hw_discard_sectors(self.geometry.max_discard_sectors()?)
            .discard_granularity(self.geometry.logical_block_size())
            .max_discard_segments(MAX_DISCARD_SEGMENTS)
            .max_hw_sectors(MAX_SECTORS)
            .max_segments(u16::try_from(MAX_PRD_ENTRIES).map_err(|_| EOVERFLOW)?)
            .max_segment_size(PRDT_DATA_BYTE_COUNT_MAX)
            .build()?;

        let request_queue = RequestQueue::new(
            self.queue.tags.clone(),
            limits,
            KBox::new(QueueData::Lu(self.clone()), GFP_KERNEL)?,
            self.queue_depth,
        )?;

        let disk =
            GenDisk::new_for_queue(fmt!("ufs{}", self.lun), request_queue, capacity_sectors, ())?;

        let mut current = self.disk.lock();
        if current.is_some() {
            return Err(EBUSY);
        }

        current.replace(disk);
        self.set_state(UfsLuState::Operational);
        Ok(())
    }

    pub(crate) fn remove_disk(&self) {
        self.set_state(UfsLuState::Reset);
        let disk = self.disk.lock().take();
        drop(disk);
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

    fn build_scsi_cmd(&self, command: mq::Command, lba: u64, blocks: u64) -> Result<UfsSCSICmd> {
        let blocks = u32::try_from(blocks).map_err(|_| EINVAL)?;
        let data_len = u32::try_from(
            u64::from(self.geometry.logical_block_size())
                .checked_mul(u64::from(blocks))
                .ok_or(EOVERFLOW)?,
        )
        .map_err(|_| EOVERFLOW)?;

        match command {
            mq::Command::Read => Ok(UfsSCSICmd::read_write(
                self.lun, false, lba, blocks, data_len, false,
            )),
            mq::Command::Write => Ok(UfsSCSICmd::read_write(
                self.lun, true, lba, blocks, data_len, false,
            )),
            mq::Command::Flush => Ok(UfsSCSICmd::flush(self.lun)),
            mq::Command::Discard => Ok(UfsSCSICmd::unmap(self.lun, lba, blocks)),
            _ => Err(ENOTSUPP),
        }
    }
}

pub(crate) struct UfsLuBlockOps;

// Hand a request that never reached the device back to the block layer.
//
// Because the command was not submitted, no hardware completion can race with
// returning the request to blk-mq.
fn complete_unsubmitted(rq: ARef<mq::Request<UfsLuBlockOps>>, e: Error) {
    rq.data_ref().inner.lock().clear();
    let rq = OwnableRefCounted::try_from_shared(rq)
        .map_err(|_e| kernel::error::code::EIO)
        .expect("Failed to complete request");

    if e == EBUSY {
        rq.requeue(true);
    } else {
        rq.end(bindings::BLK_STS_IOERR as u8);
    }
}

#[pin_data]
pub(crate) struct UfsRequestData {
    #[pin]
    pub(crate) inner: SpinLock<UfsRequestInner>,
    pub(crate) status: Atomic<u32>,
}

pub(crate) struct TagSetData {
    pub(crate) dma_vec_mempool: DmaMapMempool<MAX_PRD_ENTRIES>,
    pub(crate) queue_map: UfsQueueMap,
}

pub(crate) enum QueueData {
    Dev(Arc<UfsQueue>),
    Lu(Arc<UfsLu>),
}

#[vtable]
impl Operations for UfsLuBlockOps {
    type RequestData = UfsRequestData;
    type QueueData = KBox<QueueData>;
    type HwData = KBox<u32>;
    type TagSetData = KBox<TagSetData>;
    type GenDiskData = ();

    fn new_request_data() -> impl PinInit<Self::RequestData> {
        pin_init!(UfsRequestData {
            inner <- new_spinlock!(UfsRequestInner::default()),
            status: Atomic::new(u32::from(bindings::BLK_STS_OK)),
        })
    }

    fn queue_rq(
        _hw_data: &u32,
        lu: &QueueData,
        rq: Owned<IdleRequest<Self>>,
        _is_last: bool,
    ) -> BlkResult {
        let command = rq.command();
        let sector = rq.sector();
        let sectors = rq.sectors();

        let cmd = match command {
            mq::Command::Read | mq::Command::Write => {
                let QueueData::Lu(lu) = lu else {
                    return Err(BLK_STS_IOERR);
                };
                let geometry = lu.geometry();
                let mask = geometry.sectors_per_block() - 1;
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
                let cmd = lu.build_scsi_cmd(command, lba, blocks)?;

                pr_debug!(
                    "[RUFS] ufs_lu: LU {} command={} lba={} blocks={}\n",
                    lu.lun(),
                    command,
                    lba,
                    blocks,
                );

                cmd
            }
            mq::Command::Flush => {
                let QueueData::Lu(lu) = lu else {
                    return Err(BLK_STS_IOERR);
                };
                pr_debug!("[RUFS] ufs_lu: flush request on LU {}\n", lu.lun());
                lu.build_scsi_cmd(command, 0, 0)?
            }
            mq::Command::Discard => {
                let QueueData::Lu(lu) = lu else {
                    return Err(BLK_STS_IOERR);
                };
                let geometry = lu.geometry();
                let mask = geometry.sectors_per_block() - 1;
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
                let cmd = lu.build_scsi_cmd(command, lba, blocks)?;

                pr_debug!(
                    "[RUFS] ufs_lu: discard LU {} lba={} blocks={}\n",
                    lu.lun(),
                    lba,
                    blocks,
                );

                cmd
            }
            mq::Command::DriverIn | mq::Command::DriverOut => {
                let rq = OwnableRefCounted::into_shared(rq.start());
                if let Err(e) = UfsRequestData::compose_dev_request(&rq) {
                    complete_unsubmitted(rq, e);
                    return Ok(());
                }
                if let Err((rq, e)) = UfsRequestData::submit(rq) {
                    complete_unsubmitted(rq, e);
                }
                return Ok(());
            }
            _ => {
                pr_warn!("[RUFS] ufs_lu: unsupported request command={}\n", command,);
                rq.start().end(bindings::BLK_STS_NOTSUPP as u8);
                return Ok(());
            }
        };

        // From here the driver takes shared ownership of the request and is
        // responsible for completing it exactly once. Normal completion hands
        // this reference to blk-mq completion; requeue and poll fallback paths
        // reclaim unique ownership first because those APIs require it.
        let rq = OwnableRefCounted::into_shared(rq.start());

        if let Err(e) = UfsRequestData::compose_scsi_cmd(&rq, cmd) {
            complete_unsubmitted(rq, e);
            return Ok(());
        }

        if let Err((rq, e)) = UfsRequestData::submit(rq) {
            complete_unsubmitted(rq, e);
        }

        Ok(())
    }

    fn commit_rqs(_hw_data: &u32, _queue_data: &QueueData) {}

    fn init_hctx(_tagset_data: &TagSetData, hctx_idx: u32) -> Result<Self::HwData> {
        // Remember which hardware queue this context drives, so `poll` can find
        // the matching backend completion queue.
        Ok(KBox::new(hctx_idx, GFP_KERNEL)?)
    }

    fn complete(rq: ARef<mq::Request<Self>>) {
        let rq = OwnableRefCounted::try_from_shared(rq)
            .map_err(|_| EBUSY)
            .expect("rufs: request completion failed");
        let status = rq.data_ref().status.load(Acquire);
        rq.data_ref()
            .status
            .store(u32::from(bindings::BLK_STS_OK), Relaxed);
        let status = u8::try_from(status).unwrap_or(bindings::BLK_STS_IOERR as u8);

        rq.end(status);
    }

    fn request_timeout(
        tag_set: &TagSet<Self>,
        queue_id: u32,
        tag: u32,
    ) -> mq::RequestTimeoutStatus {
        let status = if let Some(rq) = tag_set.tag_to_rq(queue_id, tag) {
            UfsRequestData::timeout(rq)
        } else {
            true
        };

        if status {
            mq::RequestTimeoutStatus::Completed
        } else {
            mq::RequestTimeoutStatus::RetryLater
        }
    }

    fn poll(
        hw_data: &u32,
        queue_data: &QueueData,
        batch: &mut mq::IoCompletionBatch<Self>,
    ) -> Result<bool> {
        let QueueData::Lu(lu) = queue_data else {
            return Err(EIO);
        };
        Ok(lu.queue.poll(*hw_data as usize, batch))
    }

    fn map_queues(tag_set: Pin<&mut TagSet<Self>>) {
        let layout = tag_set.data().queue_map;
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
