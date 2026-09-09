// SPDX-License-Identifier: GPL-2.0

//! UFS physical region descriptor table mapping.

use core::mem::ManuallyDrop;

use crate::hci::descriptor::{PrdEntry, MAX_PRD_ENTRIES};
use crate::lu::UfsLuBlockOps;
use crate::protocol::scsi::{UfsSCSICmd, UfsUnmapParameterList};
use kernel::bindings;
use kernel::block::error::BlkError;
use kernel::block::mq::dma_map_iter::{DmaMapIterMapped, DmaMapMempool};
use kernel::block::mq::dma_map_single::DetachedStreamingInFlight;
use kernel::sync::aref::ARef;
use kernel::types::Owned;
use kernel::{block::mq, device, prelude::*};

pub(crate) const PRDT_DATA_BYTE_COUNT_MAX: u32 = 0x00040000;

const PRDT_DATA_BYTE_COUNT_PAD: usize = 4;
pub(crate) enum UfsPreparedMapping {
    None,
    Sg(DmaMapIterMapped<MAX_PRD_ENTRIES>),
    Single(Option<DetachedStreamingInFlight<KBox<UfsUnmapParameterList>>>),
}

impl UfsPreparedMapping {
    /// Marks the descriptor containing this mapping visible to hardware.
    pub(crate) fn publish(self) -> UfsActiveMapping {
        UfsActiveMapping {
            mapping: ManuallyDrop::new(self),
        }
    }

    /// Reclaims a single-buffer mapping after device access has ended.
    ///
    /// # Safety
    ///
    /// The mapping must not have been published or the device must have
    /// finished accessing it.
    unsafe fn reclaim_single(&mut self) {
        if let Self::Single(mapping) = self {
            if let Some(mapping) = mapping.take() {
                // SAFETY: Forwarded from this function's safety requirements.
                let mapping = unsafe { mapping.complete() };
                drop(mapping);
            }
        }
    }
}

impl Drop for UfsPreparedMapping {
    fn drop(&mut self) {
        // SAFETY: A mapping that remains in this prepared type has never
        // crossed the hardware publication boundary.
        unsafe { self.reclaim_single() };
    }
}

/// A data mapping owned by a command that may be visible to hardware.
pub(crate) struct UfsActiveMapping {
    mapping: ManuallyDrop<UfsPreparedMapping>,
}

impl UfsActiveMapping {
    /// Reclaims a mapping after hardware can no longer access it.
    ///
    /// # Safety
    ///
    /// The device must have completed the command or the controller must have
    /// invalidated every command that could refer to this mapping.
    pub(crate) unsafe fn complete(self) {
        let mut this = ManuallyDrop::new(self);

        // SAFETY: `this` cannot run `Drop`, so the mapping is moved exactly
        // once and is not accessed again.
        let mut mapping = unsafe { ManuallyDrop::take(&mut this.mapping) };
        // SAFETY: Forwarded from this function's safety requirements.
        unsafe { mapping.reclaim_single() };
        drop(mapping);
    }
}

impl Drop for UfsActiveMapping {
    fn drop(&mut self) {
        if matches!(&*self.mapping, UfsPreparedMapping::None) {
            // SAFETY: The None variant contains no DMA mapping and can be
            // released without proving that hardware access has ended.
            unsafe { ManuallyDrop::drop(&mut self.mapping) };
            return;
        }

        // Do not release an active mapping without proof that device access
        // has ended. Leaking is safer than allowing DMA into reused memory.
        pr_warn!("rufs: active DMA mapping dropped without completion; leaking the mapping\n");
    }
}

pub(crate) struct UfsPrdt {
    mapping: UfsPreparedMapping,
    entry_count: usize,
}

impl UfsPrdt {
    pub(crate) fn map<F>(
        dev: &ARef<device::Device>,
        cmd: UfsSCSICmd,
        rq: &Owned<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
        mut write_entry: F,
    ) -> Result<Self>
    where
        F: FnMut(usize, PrdEntry) -> Result,
    {
        if cmd.data_len() == 0 {
            return Ok(Self {
                mapping: UfsPreparedMapping::None,
                entry_count: 0,
            });
        }

        let mut iter = rq
            .dma_map_iter(dev, mempool.clone())
            .map_err(map_dma_error)?;
        let mut remaining = cmd.data_len();
        let mut entry_count = 0;

        loop {
            let segment_address = iter.address();
            let segment_length = iter.length();
            if segment_length == 0 || segment_length > remaining {
                return Err(EINVAL);
            }

            append_entries(
                &mut entry_count,
                segment_address,
                segment_length,
                &mut write_entry,
            )?;

            remaining -= segment_length;
            if remaining == 0 {
                break;
            }

            iter.next().map_err(map_dma_error)?;
        }

        let iter = iter.finish();

        Ok(Self {
            mapping: UfsPreparedMapping::Sg(iter),
            entry_count,
        })
    }

    pub(crate) fn single(mapping: DetachedStreamingInFlight<KBox<UfsUnmapParameterList>>) -> Self {
        Self {
            mapping: UfsPreparedMapping::Single(Some(mapping)),
            entry_count: 1,
        }
    }

    pub(crate) fn entry_count(&self) -> usize {
        self.entry_count
    }

    pub(crate) fn into_mapping(self) -> UfsPreparedMapping {
        self.mapping
    }
}

fn map_dma_error(error: BlkError) -> Error {
    match error.to_blk_status() {
        bindings::BLK_STS_RESOURCE | bindings::BLK_STS_DEV_RESOURCE => EBUSY,
        _ => EIO,
    }
}

fn append_entries<F>(
    entry_count: &mut usize,
    segment_address: u64,
    segment_length: u32,
    write_entry: &mut F,
) -> Result<()>
where
    F: FnMut(usize, PrdEntry) -> Result,
{
    if segment_length == 0 || segment_length % PRDT_DATA_BYTE_COUNT_PAD as u32 != 0 {
        return Err(EINVAL);
    }

    let mut segment_offset = 0;
    while segment_offset < segment_length {
        if *entry_count == MAX_PRD_ENTRIES {
            return Err(EINVAL);
        }

        let length = core::cmp::min(PRDT_DATA_BYTE_COUNT_MAX, segment_length - segment_offset);
        let address = segment_address
            .checked_add(u64::from(segment_offset))
            .ok_or(EOVERFLOW)?;

        write_entry(*entry_count, PrdEntry::new(address, length)?)?;
        *entry_count += 1;
        segment_offset += length;
    }

    Ok(())
}
