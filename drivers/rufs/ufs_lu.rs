// SPDX-License-Identifier: GPL-2.0

//! Per-logical-unit state for the Rust UFS driver.

#![allow(dead_code)]

use kernel::block::{
    error::BlkResult,
    mq::{self, gen_disk::GenDisk, gen_disk::GenDiskBuilder, IdleRequest, Operations, TagSet},
    SECTOR_SIZE,
};
use kernel::sync::{Arc, ArcBorrow, Mutex, SpinLock};
use kernel::types::{ARef, OwnableRefCounted, Owned};
use kernel::{new_mutex, new_spinlock, prelude::*};

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
    lun: u8,
    geometry: UfsLuGeometry,

    #[pin]
    state: SpinLock<UfsLuState>,

    #[pin]
    disk: Mutex<Option<Arc<GenDisk<UfsLuBlockOps>>>>,
}

impl UfsLu {
    pub(crate) fn new(lun: u8, geometry: UfsLuGeometry) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
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
        _queue_data: ArcBorrow<'_, UfsLu>,
        rq: Owned<IdleRequest<Self>>,
        _is_last: bool,
    ) -> BlkResult {
        rq.start().end_ok();
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
