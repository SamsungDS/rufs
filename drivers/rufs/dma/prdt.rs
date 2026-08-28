// SPDX-License-Identifier: GPL-2.0

//! UFS physical region descriptor table mapping.

use crate::hci::descriptor::{PrdEntry, MAX_PRD_ENTRIES};
use crate::lu::UfsLuBlockOps;
use crate::protocol::scsi::UfsSCSICmd;
use kernel::block::mq::dma_map_iter::{DmaMapIterMapped, DmaMapMempool};
use kernel::dma::Coherent;
use kernel::sync::aref::ARef;
use kernel::types::Owned;
use kernel::{block::mq, device, prelude::*};

pub(crate) const PRDT_DATA_BYTE_COUNT_MAX: u32 = 0x00040000;

const PRDT_DATA_BYTE_COUNT_PAD: usize = 4;
const UNMAP_PARAM_LIST_SIZE: usize = 24;

pub(crate) enum UfsPrdtMapping {
    Sg(DmaMapIterMapped<'static, MAX_PRD_ENTRIES, UfsLuBlockOps>),
    Unmap(UfsUnmapMapping),
}

pub(crate) struct UfsUnmapMapping {
    _dev: ARef<device::Device>,
    buffer: Coherent<[u8]>,
}

pub(crate) struct UfsPrdt {
    mapping: Option<UfsPrdtMapping>,
    entries: KVec<PrdEntry>,
}

impl UfsPrdt {
    pub(crate) fn map(
        dev: &ARef<device::Device>,
        cmd: UfsSCSICmd,
        rq: &Owned<mq::Request<UfsLuBlockOps>>,
        mempool: &DmaMapMempool<MAX_PRD_ENTRIES>,
    ) -> Result<Self> {
        if cmd.data_len() == 0 {
            return Ok(Self {
                mapping: None,
                entries: KVec::new(),
            });
        }

        if cmd.is_unmap() {
            return Self::map_unmap(dev, cmd);
        }

        let mut iter = rq
            .dma_map_iter(dev, mempool.clone())
            .map_err(|_| ENOMEM)?;
        let mut remaining = cmd.data_len();
        let mut entries = KVec::new();

        loop {
            let segment_address = iter.address();
            let segment_length = iter.length();
            if segment_length == 0 || segment_length > remaining {
                return Err(EINVAL);
            }

            append_entries(&mut entries, segment_address, segment_length)?;

            remaining -= segment_length;
            if remaining == 0 {
                break;
            }

            iter.next()?;
        }

        // SAFETY: The mapping is stored in this request's private data. blk-mq
        // keeps the request alive by its tag until RUFS takes and drops the
        // mapping before completing or requeuing the request.
        let iter = unsafe { iter.finish_detached() };

        Ok(Self {
            mapping: Some(UfsPrdtMapping::Sg(iter)),
            entries,
        })
    }

    fn map_unmap(dev: &ARef<device::Device>, cmd: UfsSCSICmd) -> Result<Self> {
        if cmd.unmap_blocks() == 0 {
            return Err(EINVAL);
        }

        let mut data = [0u8; UNMAP_PARAM_LIST_SIZE];

        // TODO: Define a type for this parameter list.
        data[0..2].copy_from_slice(&22u16.to_be_bytes());
        data[2..4].copy_from_slice(&16u16.to_be_bytes());
        data[8..16].copy_from_slice(&cmd.unmap_lba().to_be_bytes());
        data[16..20].copy_from_slice(&cmd.unmap_blocks().to_be_bytes());

        // SAFETY: The bound RUFS instance owns `dev` for the lifetime of every
        // request-private mapping created through this path.
        let bound_dev = unsafe { dev.as_bound() };

        // TODO: Consider using a DMA pool instead of allocating for each unmap.
        let buffer = Coherent::from_slice(bound_dev, &data, GFP_ATOMIC)?;
        let mapping = UfsUnmapMapping {
            _dev: dev.clone(),
            buffer,
        };

        let mut entries = KVec::with_capacity(1, GFP_ATOMIC)?;
        entries.push(
            PrdEntry::new(mapping.buffer.dma_handle(), UNMAP_PARAM_LIST_SIZE as u32)?,
            GFP_ATOMIC,
        )?;

        Ok(Self {
            mapping: Some(UfsPrdtMapping::Unmap(mapping)),
            entries,
        })
    }

    pub(crate) fn entries(&self) -> &[PrdEntry] {
        &self.entries
    }

    pub(crate) fn into_mapping(self) -> Option<UfsPrdtMapping> {
        self.mapping
    }
}

fn append_entries(
    entries: &mut KVec<PrdEntry>,
    segment_address: u64,
    segment_length: u32,
) -> Result<()> {
    if segment_length == 0 || segment_length % PRDT_DATA_BYTE_COUNT_PAD as u32 != 0 {
        return Err(EINVAL);
    }

    let mut segment_offset = 0;
    while segment_offset < segment_length {
        if entries.len() == MAX_PRD_ENTRIES {
            return Err(EINVAL);
        }

        let length = core::cmp::min(PRDT_DATA_BYTE_COUNT_MAX, segment_length - segment_offset);
        let address = segment_address
            .checked_add(u64::from(segment_offset))
            .ok_or(EOVERFLOW)?;

        entries.push(PrdEntry::new(address, length)?, GFP_ATOMIC)?;
        segment_offset += length;
    }

    Ok(())
}
