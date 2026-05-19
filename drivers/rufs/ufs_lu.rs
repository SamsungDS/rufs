// SPDX-License-Identifier: GPL-2.0

//! Per-logical-unit state for the Rust UFS driver.

#![allow(dead_code)]

use kernel::block::{
    error::BlkResult,
    mq::{self, gen_disk::GenDisk, gen_disk::GenDiskBuilder, IdleRequest, Operations, TagSet},
    SECTOR_SIZE,
};
use kernel::bindings;
use kernel::sync::{Arc, ArcBorrow, Mutex, SpinLock};
use kernel::types::{ARef, OwnableRefCounted, Owned};
use kernel::{new_mutex, new_spinlock, prelude::*};
use crate::ufs_queue::*;

const SECTOR_SIZE_U64: u64 = SECTOR_SIZE as u64;

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

    #[pin]
    state: SpinLock<UfsLuState>,

    #[pin]
    disk: Mutex<Option<Arc<GenDisk<UfsLuBlockOps>>>>,
}

impl UfsLu {
    pub(crate) fn new(queue: Arc<UfsQueue>, lun: u8, geometry: UfsLuGeometry) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                queue,
                lun,
                geometry,
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
            bindings::req_op_REQ_OP_DISCARD => Ok(UfsSCSICmd::unmap(self.lun, 24)),
            _ => Err(ENOTSUPP),
        }
    }

    fn compose_request(
        &self,
        tag: usize,
        cmd: UfsSCSICmd,
        rq: &ARef<mq::Request<UfsLuBlockOps>>,
    ) -> Result<Arc<UfsRequest>> {
        let request = self.queue.acquire(tag)?;
        request.set_block_request(rq.clone())?;
        request.compose(UfsCmd::SCSI(cmd))?;
        Ok(request)
    }
}

pub(crate) struct UfsLuBlockOps;

#[vtable]
impl Operations for UfsLuBlockOps {
    type RequestData = ();
    type QueueData = Arc<UfsLu>;
    type HwData = ();
    type TagSetData = ();

    fn new_request_data() -> impl PinInit<Self::RequestData> {}

    fn queue_rq(
        _hw_data: (),
        lu: ArcBorrow<'_, UfsLu>,
        rq: Owned<IdleRequest<Self>>,
        _is_last: bool,
    ) -> BlkResult {
        let op = rq.command() as bindings::req_op;
        let sector = rq.sector();
        let sectors = rq.sectors();
        let tag = rq.tag() as usize;
        let geometry = lu.geometry();
        let mask = geometry.sectors_per_block() - 1;

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
                pr_warn!(
                    "[RUFS] ufs_lu: discard request is not supported yet on LU {} sector={} sectors={}\n",
                    lu.lun(),
                    sector,
                    sectors,
                );
                rq.start().end(bindings::BLK_STS_NOTSUPP as u8);
                return Ok(());
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

        // Convert the request into a shared reference and keep it with the
        // hardware command, so the completion path can hand it back to the
        // block layer once the command finishes.
        let rq = OwnableRefCounted::into_shared(rq.start());
        let request = lu.compose_request(tag, cmd, &rq)?;

        if request.submit().is_err() {
            // `submit` dropped the request reference it stored, so we hold the
            // only one and can complete it here.
            if let Ok(rq) = OwnableRefCounted::try_from_shared(rq) {
                rq.end(bindings::BLK_STS_IOERR as u8);
            }
        }

        Ok(())
    }

    fn commit_rqs(_hw_data: (), _queue_data: ArcBorrow<'_, UfsLu>) {}

    fn init_hctx(_tagset_data: (), _hctx_idx: u32) -> Result<Self::HwData> {
        Ok(())
    }

    fn complete(rq: ARef<mq::Request<Self>>) {
        OwnableRefCounted::try_from_shared(rq)
            .map_err(|_| EBUSY)
            .expect("rufs: request completion failed")
            .end_ok();
    }
}
