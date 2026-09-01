// SPDX-License-Identifier: GPL-2.0

//! Single-buffer streaming DMA mappings for asynchronous block requests.

use crate::{
    device,
    dma::{ContiguousBuffer, DataDirection, DmaAddress, Streaming, StreamingInFlight},
    prelude::*,
    sync::aref::ARef,
};
use core::mem::ManuallyDrop;

/// A streaming DMA mapping that owns the device reference needed to detach it
/// from the borrow used to create it.
///
/// This is intended for block requests whose DMA mappings remain live after
/// the request submission callback returns. It preserves the state model of
/// [`Streaming`], while the caller supplies the driver-bound lifetime
/// guarantee that cannot currently cross asynchronous blk-mq callbacks.
pub struct DetachedStreaming<C: ContiguousBuffer> {
    // This field must be dropped before `dev`, because dropping it unmaps the
    // buffer through the device pointer.
    inner: Streaming<'static, C>,
    dev: ARef<device::Device>,
}

impl<C: ContiguousBuffer> DetachedStreaming<C> {
    /// Maps `container` for streaming DMA and detaches the mapping from the
    /// device borrow.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the device remains bound until this mapping
    /// and every state derived from it are either dropped after completion or
    /// deliberately leaked.
    pub unsafe fn new(
        dev: &device::Device,
        container: C,
        direction: DataDirection,
    ) -> Result<Self> {
        let dev: ARef<device::Device> = dev.into();

        // SAFETY: The caller guarantees that the device remains bound for the
        // effective lifetime of the detached mapping. `dev` keeps the device
        // allocation alive, and is dropped after `inner` unmaps the buffer.
        let bound = unsafe { dev.as_bound() };
        // SAFETY: The returned reference is only stored in `inner`. The owned
        // device reference and the caller's binding guarantee remain attached
        // to every state derived from this value.
        let bound = unsafe {
            core::mem::transmute::<
                &device::Device<device::Bound>,
                &'static device::Device<device::Bound>,
            >(bound)
        };
        let inner = Streaming::new(bound, container, direction)?;

        Ok(Self { inner, dev })
    }

    /// Returns the size of the mapping in bytes.
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Hands the DMA address to the asynchronous request owner.
    pub fn submit(self) -> DetachedStreamingInFlight<C> {
        let Self { inner, dev } = self;

        DetachedStreamingInFlight {
            inner: ManuallyDrop::new(inner.submit()),
            dev: ManuallyDrop::new(dev),
        }
    }

    /// Unmaps the buffer and returns its backing storage.
    pub fn into_inner(self) -> C {
        self.inner.into_inner()
    }
}

/// A detached single-buffer mapping whose DMA address may be visible to the
/// device.
pub struct DetachedStreamingInFlight<C: ContiguousBuffer> {
    inner: ManuallyDrop<StreamingInFlight<'static, C>>,
    dev: ManuallyDrop<ARef<device::Device>>,
}

impl<C: ContiguousBuffer> DetachedStreamingInFlight<C> {
    /// Returns the DMA address to program into the device.
    pub fn dma_handle(&self) -> DmaAddress {
        self.inner.dma_handle()
    }

    /// Returns the size of the mapping in bytes.
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Takes the mapping back after the device has stopped accessing it.
    ///
    /// # Safety
    ///
    /// The device must have finished accessing the buffer.
    pub unsafe fn complete(self) -> DetachedStreaming<C> {
        let mut this = ManuallyDrop::new(self);

        // SAFETY: `this` cannot run its `Drop` implementation, so each field
        // is moved exactly once and is not accessed again.
        let inner = unsafe { ManuallyDrop::take(&mut this.inner) };
        // SAFETY: Same as above.
        let dev = unsafe { ManuallyDrop::take(&mut this.dev) };
        // SAFETY: Forwarded from this function's safety requirements.
        let inner = unsafe { inner.complete() };

        DetachedStreaming { inner, dev }
    }
}

impl<C: ContiguousBuffer> Drop for DetachedStreamingInFlight<C> {
    fn drop(&mut self) {
        // Neither field is dropped. Keeping both the mapping storage and the
        // device reference alive avoids unmapping memory that hardware may
        // still access.
        pr_warn!(
            "DetachedStreamingInFlight dropped without complete(); leaking the mapping, storage, and device reference\n"
        );
    }
}
