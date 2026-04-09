// SPDX-License-Identifier: GPL-2.0

use crate::{
    alloc::mempool::{MemPool, MemPoolBox},
    bindings,
    block::error::{BlkError, BlkResult},
    device::Device,
    dma::{DataDirection, DmaAddress},
    prelude::*,
    sync::{aref::ARef, Arc},
    types::Opaque,
};

use super::{Operations, Request};

/// Type of the [`MemPool`] used to allocate storage for dma vector descriptors in [`DmaMapIter`].
pub type DmaMapMempool<const N: usize> = Arc<MemPool<[DmaVector; N]>>;

/// A descriptor of a memory segment that is mapped for DMA.
#[derive(Zeroable, Clone, Copy)]
pub struct DmaVector {
    address: DmaAddress,
    length: u32,
}

/// An iterator that DMA maps segments of a block request.
pub struct DmaMapIter<'a, const N: usize, T: Operations> {
    iter: Opaque<bindings::blk_dma_iter>,
    request: &'a Request<T>,
    mapping: DmaMapping<N>,
}

impl<'a, const N: usize, T: Operations> DmaMapIter<'a, N, T> {
    fn resource_error() -> BlkError {
        BlkError::from_blk_status(bindings::BLK_STS_RESOURCE)
    }

    fn iterator_error(&self) -> BlkError {
        // SAFETY: `self.iter` is a valid `blk_dma_iter` owned by `self`.
        let status = unsafe { (*self.iter.get()).status };
        if status == bindings::BLK_STS_OK {
            BlkError::from_blk_status(bindings::BLK_STS_IOERR)
        } else {
            BlkError::from_blk_status(status)
        }
    }

    pub(crate) fn new(
        rq: &'a Request<T>,
        device: &Device,
        mempool: DmaMapMempool<N>,
    ) -> BlkResult<Self> {
        if N == 0 {
            return Err(Self::resource_error());
        }

        let mut this = Self {
            iter: Opaque::zeroed(),
            request: rq,
            mapping: DmaMapping::new(rq.dma_direction(), device, mempool)?,
        };

        this.start()?;
        Ok(this)
    }

    fn start(&mut self) -> BlkResult {
        // SAFETY: `self.request` is a valid request, `self.mapping.device` is a valid device,
        // and `self.mapping.state` and `self.iter` are valid allocations owned by `self`.
        let ok = unsafe {
            bindings::blk_rq_dma_map_iter_start(
                self.request.as_raw(),
                self.mapping.device.as_raw(),
                self.mapping.state.get(),
                self.iter.get(),
            )
        };

        if ok {
            // SAFETY: `self.iter` is a valid `blk_dma_iter` owned by `self`. By C API contract,
            // the `p2pdma` union member is valid after a successful call to
            // `blk_rq_dma_map_iter_start`.
            self.mapping.map_type = unsafe { (*self.iter.get()).p2pdma.map };
            self.add_vector();
            Ok(())
        } else {
            Err(self.iterator_error())
        }
    }

    fn add_vector(&mut self) {
        self.mapping.add_vector(self.address(), self.length());
    }

    /// Advance the iterator to the next DMA mapped segment.
    #[expect(clippy::should_implement_trait)]
    pub fn next(&mut self) -> BlkResult {
        if self.mapping.dma_vector_count == N {
            return Err(Self::resource_error());
        }

        // SAFETY: `self.request` is a valid request, `self.mapping.device` is a valid device,
        // and `self.iter` is a valid allocation owned by `self`.
        let ok = unsafe {
            bindings::blk_rq_dma_map_iter_next(
                self.request.as_raw(),
                self.mapping.device.as_raw(),
                self.iter.get(),
            )
        };

        if ok {
            self.add_vector();
            Ok(())
        } else {
            Err(self.iterator_error())
        }
    }

    /// Return the DMA address of the current segment.
    pub fn address(&self) -> DmaAddress {
        // SAFETY: `self.iter` is a valid `blk_dma_iter` owned by `self`.
        unsafe { (*self.iter.get()).addr }
    }

    /// Return the length of the current segment.
    pub fn length(&self) -> u32 {
        // SAFETY: `self.iter` is a valid `blk_dma_iter` owned by `self`.
        unsafe { (*self.iter.get()).len }
    }

    /// Consume the iterator and return the completed mapping result.
    pub fn finish(self) -> DmaMapIterMapped<N> {
        let Self {
            iter: _,
            request: _,
            mapping,
        } = self;
        DmaMapIterMapped { _inner: mapping }
    }
}

struct DmaMapping<const N: usize> {
    state: Opaque<bindings::dma_iova_state>,
    device: ARef<Device>,
    dma_vectors: MemPoolBox<[DmaVector; N]>,
    dma_vector_count: usize,
    mapped_length: usize,
    direction: DataDirection,
    map_type: bindings::pci_p2pdma_map_type,
}

impl<const N: usize> DmaMapping<N> {
    fn new(
        direction: DataDirection,
        device: &Device,
        mempool: DmaMapMempool<N>,
    ) -> BlkResult<Self> {
        let dma_vectors = mempool
            .alloc_zeroed(GFP_ATOMIC)
            .map_err(|_| BlkError::from_blk_status(bindings::BLK_STS_RESOURCE))?;

        Ok(Self {
            state: Opaque::zeroed(),
            device: device.into(),
            dma_vectors,
            dma_vector_count: 0,
            mapped_length: 0,
            direction,
            map_type: bindings::pci_p2pdma_map_type_PCI_P2PDMA_MAP_UNKNOWN,
        })
    }

    fn add_vector(&mut self, address: DmaAddress, length: u32) {
        self.dma_vectors[self.dma_vector_count] = DmaVector { address, length };
        self.dma_vector_count += 1;
        self.mapped_length += length as usize;
    }
}

impl<const N: usize> Drop for DmaMapping<N> {
    fn drop(&mut self) {
        // Some mappings can be released from the IOVA state alone. Otherwise,
        // release every direct mapping recorded while iterating the request.
        // SAFETY: `self.device` is a valid device, `self.state` is a valid allocation owned by
        // `self`, and `self.mapped_length`, `self.map_type` and `self.direction` describe the
        // mapping established when iterating the request.
        if !unsafe {
            bindings::blk_dma_unmap(
                self.device.as_raw(),
                self.state.get(),
                self.mapped_length,
                self.map_type,
                self.direction.into(),
            )
        } {
            let attrs =
                if self.map_type == bindings::pci_p2pdma_map_type_PCI_P2PDMA_MAP_THRU_HOST_BRIDGE {
                    bindings::DMA_ATTR_MMIO as usize
                } else {
                    0
                };

            for mapping in &self.dma_vectors[0..self.dma_vector_count] {
                // SAFETY: `self.device` is a valid device and `mapping` describes a DMA mapping
                // that was established with `dma_map_phys` and not yet released.
                unsafe {
                    bindings::dma_unmap_phys(
                        self.device.as_raw(),
                        mapping.address,
                        mapping.length as usize,
                        self.direction.into(),
                        attrs,
                    )
                };
            }
        }
    }
}

// SAFETY: `DmaMapping` owns its mapping state and can be dropped from any
// thread while the DMA device remains bound through `device`.
unsafe impl<const N: usize> Send for DmaMapping<N> {}

/// A set of mapped pages produced by [`DmaMapIter`].
///
/// The mapping owns a reference to its DMA device and no longer borrows the
/// block request used to construct it. Drivers must still release it before
/// the device is unbound. This matches the device-lifetime requirement of the
/// underlying DMA mapping API.
pub struct DmaMapIterMapped<const N: usize> {
    _inner: DmaMapping<N>,
}

impl<const N: usize> Unpin for DmaMapIterMapped<N> {}
