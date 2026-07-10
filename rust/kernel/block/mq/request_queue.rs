// SPDX-License-Identifier: GPL-2.0

use super::{
    request::SyncRequest, Command, Feature, Features, Operations, TagSet //
};
use crate::{
    error::from_err_ptr,
    owned::Owned,
    prelude::*,
    sync::{aref::ARef, Arc, Refcount},
    types::{
        ForeignOwnable,
        Opaque,
        RefCounted,
        ScopeGuard, //
    }, //
};
use core::{
    marker::PhantomData,
    mem::ManuallyDrop,
    ops::Deref,
    ptr::NonNull, //
};

/// TODO
pub struct LimitsBuilder<T> {
    logical_block_size: u32,
    physical_block_size: u32,
    max_hw_discard_sectors: u32,
    discard_granularity: u32,
    max_discard_segments: u16,
    virt_boundary_mask: usize,
    max_hw_sectors: u32,
    max_sectors: u32,
    max_segments: u16,
    max_segment_size: u32,
    rotational: bool,
    write_cache: bool,
    forced_unit_access: bool,
    #[cfg(CONFIG_BLK_DEV_ZONED)]
    zoned: bool,
    #[cfg(CONFIG_BLK_DEV_ZONED)]
    zone_size_sectors: u32,
    #[cfg(CONFIG_BLK_DEV_ZONED)]
    zone_append_max_sectors: u32,
    _p: PhantomData<T>,
}

impl<T: Operations> Default for LimitsBuilder<T> {
    fn default() -> Self {
        Self {
            rotational: false,
            logical_block_size: bindings::PAGE_SIZE as u32,
            physical_block_size: bindings::PAGE_SIZE as u32,
            max_hw_discard_sectors: 0,
            discard_granularity: 0,
            max_discard_segments: 0,
            #[cfg(CONFIG_BLK_DEV_ZONED)]
            zoned: false,
            #[cfg(CONFIG_BLK_DEV_ZONED)]
            zone_size_sectors: 0,
            #[cfg(CONFIG_BLK_DEV_ZONED)]
            zone_append_max_sectors: 0,
            write_cache: false,
            forced_unit_access: false,
            max_hw_sectors: 0,
            max_sectors: 0,
            max_segments: 0,
            max_segment_size: 0,
            virt_boundary_mask: 0,
            _p: PhantomData,
        }
    }
}

impl<T: Operations> LimitsBuilder<T> {
    /// TODO
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the rotational media attribute for the device to be built.
    pub fn rotational(mut self, rotational: bool) -> Self {
        self.rotational = rotational;
        self
    }

    /// Validate block size by verifying that it is between 512 and `PAGE_SIZE`,
    /// and that it is a power of two.
    pub fn validate_block_size(size: u32) -> Result {
        if !(512..=bindings::PAGE_SIZE as u32).contains(&size) || !size.is_power_of_two() {
            Err(EINVAL)
        } else {
            Ok(())
        }
    }

    /// Set the logical block size of the device to be built.
    ///
    /// This method will check that block size is a power of two and between 512
    /// and 4096. If not, an error is returned and the block size is not set.
    ///
    /// This is the smallest unit the storage device can address. It is
    /// typically 4096 bytes.
    pub fn logical_block_size(mut self, block_size: u32) -> Result<Self> {
        Self::validate_block_size(block_size)?;
        self.logical_block_size = block_size;
        Ok(self)
    }

    /// Set the physical block size of the device to be built.
    ///
    /// This method will check that block size is a power of two and between 512
    /// and 4096. If not, an error is returned and the block size is not set.
    ///
    /// This is the smallest unit a physical storage device can write
    /// atomically. It is usually the same as the logical block size but may be
    /// bigger. One example is SATA drives with 4096 byte physical block size
    /// that expose a 512 byte logical block size to the operating system.
    pub fn physical_block_size(mut self, block_size: u32) -> Result<Self> {
        Self::validate_block_size(block_size)?;
        self.physical_block_size = block_size;
        Ok(self)
    }

    /// Mark this device as a zoned block device.
    #[cfg(CONFIG_BLK_DEV_ZONED)]
    pub fn zoned(mut self, enable: bool) -> Self {
        self.zoned = enable;
        self
    }

    /// Set the zone size of this block device.
    #[cfg(CONFIG_BLK_DEV_ZONED)]
    pub fn zone_size(mut self, sectors: u32) -> Self {
        self.zone_size_sectors = sectors;
        self
    }

    /// Set the max zone append size for this block device.
    #[cfg(CONFIG_BLK_DEV_ZONED)]
    pub fn zone_append_max(mut self, sectors: u32) -> Self {
        self.zone_append_max_sectors = sectors;
        self
    }

    /// Declare that this device supports forced unit access.
    pub fn forced_unit_access(mut self, enable: bool) -> Self {
        self.forced_unit_access = enable;
        self
    }

    /// Declare that this device has a write-back cache.
    pub fn write_cache(mut self, enable: bool) -> Self {
        self.write_cache = enable;
        self
    }

    /// Maximum size of a command in 512 byte sectors.
    pub fn max_sectors(mut self, sectors: u32) -> Self {
        self.max_sectors = sectors;
        self
    }

    /// Set the I/O segment memory alignment mask for the block device. I/O requests to this device
    /// will be split between segments wherever either the memory address of the end of the previous
    /// segment or the memory address of the beginning of the current segment is not aligned to
    /// virt_boundary_mask + 1 bytes.
    pub fn virt_boundary_mask(mut self, mask: usize) -> Self {
        self.virt_boundary_mask = mask;
        self
    }

    /// Set the maximum amount of sectors the underlying hardware device can
    /// discard/trim in a single operation.
    ///
    /// Setting 0 (default) here will cause the disk to report discard not
    /// supported.
    pub fn max_hw_discard_sectors(mut self, max_hw_discard_sectors: u32) -> Self {
        self.max_hw_discard_sectors = max_hw_discard_sectors;
        self
    }

    /// Set the granularity of discard operations, in bytes.
    ///
    /// Devices that support discard may internally allocate space in units that
    /// are bigger than the logical block size. This value indicates the size of
    /// the internal allocation unit in bytes. The beginning and the size of a
    /// discard request should be aligned to this granularity for the discard to
    /// take effect. If 0 is set here, the granularity is set to match the
    /// physical block size of the device.
    pub fn discard_granularity(mut self, discard_granularity: u32) -> Self {
        self.discard_granularity = discard_granularity;
        self
    }

    /// Set the maximum number of scatter/gather entries in a discard request.
    ///
    /// This is the maximum number of discontiguous ranges the underlying
    /// hardware device can discard/trim in a single operation.
    pub fn max_discard_segments(mut self, max_discard_segments: u16) -> Self {
        self.max_discard_segments = max_discard_segments;
        self
    }

    /// Maximum hardware I/O size in 512 byte sectors.
    pub fn max_hw_sectors(mut self, sectors: u32) -> Self {
        self.max_hw_sectors = sectors;
        self
    }

    /// Maximum number of segments per request.
    pub fn max_segments(mut self, segments: u16) -> Self {
        self.max_segments = segments;
        self
    }

    /// Maximum size of a segment in bytes.
    pub fn max_segment_size(mut self, size: u32) -> Self {
        self.max_segment_size = size;
        self
    }

    /// TODO
    pub fn build(self) -> Result<Limits> {
        let mut lim: bindings::queue_limits = pin_init::zeroed();

        lim.logical_block_size = self.logical_block_size;
        lim.physical_block_size = self.physical_block_size;
        lim.max_hw_discard_sectors = self.max_hw_discard_sectors;
        lim.discard_granularity = self.discard_granularity;
        lim.max_discard_segments = self.max_discard_segments;
        lim.max_hw_sectors = self.max_hw_sectors;
        lim.max_sectors = self.max_sectors;
        lim.max_segments = self.max_segments;
        lim.max_segment_size = self.max_segment_size;
        lim.virt_boundary_mask = self.virt_boundary_mask;
        if self.rotational {
            lim.features = Feature::Rotational.into();
        }

        #[cfg(CONFIG_BLK_DEV_ZONED)]
        if self.zoned {
            if !T::HAS_REPORT_ZONES {
                return Err(EINVAL);
            }

            lim.features |= Feature::Zoned;
            lim.chunk_sectors = self.zone_size_sectors;
            lim.max_hw_zone_append_sectors = self.zone_append_max_sectors;
        }

        if self.write_cache {
            lim.features |= Feature::WriteCache;
        }

        if self.forced_unit_access {
            lim.features |= Feature::ForcedUnitAccess;
        }

        Ok(Limits(lim))
    }
}

/// TODO
#[repr(transparent)]
pub struct Limits(bindings::queue_limits);

impl Limits {
    pub(crate) fn as_raw(&mut self) -> *mut bindings::queue_limits {
        &raw mut self.0
    }

    /// TODO
    pub fn features(&self) -> Features {
        Features::try_from(self.0.features).expect("Expect valid flags")
    }
}

/// A structure describing the queues associated with a block device.
///
/// Owned by a [`GenDisk`].
///
/// # Invariants
///
/// - `self.0` is a valid `bindings::request_queue`.
/// - `self.0.queuedata` is a valid `T::QueueData`.
#[repr(transparent)]
pub struct RequestQueue<T>(Opaque<bindings::request_queue>, PhantomData<T>);

impl<T> RequestQueue<T>
where
    T: Operations,
{
    /// Allocate a new [`RequestQueue`].
    pub fn new(
        tagset: Arc<TagSet<T>>,
        mut limits: Limits,
        queue_data: T::QueueData,
        queue_depth: u32,
    ) -> Result<BoundRequestQueue<T>> {
        if queue_depth == 0 {
            return Err(EINVAL);
        }

        let data = queue_data.into_foreign();

        let recover_data = ScopeGuard::new(|| {
            // SAFETY: `data` was created by the call to `into_foreign()` above.
            drop(unsafe { T::QueueData::from_foreign(data) });
        });

        let mq = from_err_ptr(unsafe {
            bindings::blk_mq_alloc_queue(
                tagset.into_raw().cast_mut().cast(),
                limits.as_raw(),
                data,
            )
        })?;

        recover_data.dismiss();

        // TODO: Not sure this is required, only used by QOS.
        if queue_depth != 0 {
            // SAFETY: `blk_mq_alloc_queue` returned a valid and initialized queue above.
            unsafe { bindings::blk_set_queue_depth(mq, queue_depth) };
        }

        Ok(BoundRequestQueue(
            unsafe { ARef::from_raw(NonNull::new_unchecked(mq.cast())) },
            PhantomData,
        ))
    }

    /// Create a [`RequestQueue`] from a raw `bindings::request_queue` pointer
    ///
    /// # Safety
    ///
    /// - `ptr` must be valid for use as a reference for the duration of `'a`.
    /// - `ptr` must have been initialized as part of [`GenDiskBuilder::build`].
    pub(crate) unsafe fn from_raw<'a>(ptr: *const bindings::request_queue) -> &'a Self {
        // INVARIANT:
        // - By function safety requirements, `ptr` is a valid `request_queue`.
        // - By function safety requirement `ptr` was initialized by [`GenDiskBuilder::build`], and
        //   thus `queuedata` was set to point to a valid `T::QueueData`.
        //
        // SAFETY: By function safety requirements `ptr` is valid for use as a reference.
        unsafe { &*ptr.cast() }
    }

    fn as_raw(&self) -> *const bindings::request_queue {
        self.0.get()
    }

    /// Get the driver private data associated with this [`RequestQueue`].
    pub fn queue_data(&self) -> <T::QueueData as ForeignOwnable>::Borrowed<'_> {
        // SAFETY: By type invariant, `queuedata` is a valid `T::QueueData`.
        unsafe { T::QueueData::borrow((*self.0.get()).queuedata) }
    }

    /// Stop all hardware queues of this [`RequestQueue`].
    pub fn stop_hw_queues(&self) {
        // SAFETY: By type invariant, `self.0` is a valid `request_queue`.
        unsafe { bindings::blk_mq_stop_hw_queues(self.0.get()) }
    }

    /// Start all hardware queues of this [`RequestQueue`].
    ///
    /// This function will mark the queues as ready and if necessary, schedule the queues to run.
    pub fn start_stopped_hw_queues_async(&self) {
        // SAFETY: By type invariant, `self.0` is a valid `request_queue`.
        unsafe { bindings::blk_mq_start_stopped_hw_queues(self.0.get(), true) }
    }

    /// Allocate a synchronous request.
    pub fn alloc_sync_request(&self, command: Command) -> Result<Owned<SyncRequest<T>>> {
        let rq = from_err_ptr(unsafe {
            bindings::blk_mq_alloc_request(self.0.get(), command.as_raw(), 0)
        })?;
        // SAFETY: `rq` is valid and will be owned by new `SyncRequest`.
        Ok(unsafe { SyncRequest::from_raw(rq) })
    }

    /// TODO
    pub fn tag_set(&self) -> Arc<TagSet<T>> {
        // SAFETY: By type invariant, `self.0` is a valid `request_queue`.
        let tag_set_ptr = unsafe { (*self.0.get()).tag_set };
        // SAFETY: We called `into_raw` during construction.
        ManuallyDrop::new(unsafe { Arc::from_raw(tag_set_ptr.cast::<TagSet<T>>()) })
            .deref()
            .clone()
    }

    /// TODO
    pub fn limits(&self) -> &Limits {
        unsafe { &*(&raw const (*self.0.get()).limits).cast::<Limits>() }
    }
}

unsafe impl<T: Operations> RefCounted for RequestQueue<T> {
    fn inc_ref(&self) {
        #[cfg_attr(not(debug_assertions), allow(unused_variables))]
        let ret = unsafe { bindings::blk_get_queue(self.0.get()) };
        debug_assert!(ret, "Queue is dying");
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        let queue_ptr = unsafe { (*obj.as_ptr()).0.get() };
        let refcount_ptr = unsafe { (&raw mut (*queue_ptr).refs).cast::<Refcount>() };
        let refcount_ref = unsafe { &*refcount_ptr };

        if refcount_ref.dec_and_test() {
            let tagset = unsafe { (*queue_ptr).tag_set };
            // SAFETY: The pointer owns a refcount on the tagset.
            drop(unsafe { Arc::from_raw(tagset.cast::<TagSet<T>>()) });

            let queuedata = unsafe { (*queue_ptr).queuedata };
            // SAFETY: `queue.queuedata` was created by `GenDisk::new` with
            // a call to `ForeignOwnable::into_foreign` to create `queuedata`.
            // `ForeignOwnable::from_foreign` is only called here.
            drop(unsafe { T::QueueData::from_foreign(queuedata) });

            // SAFETY: The refcount is zero.
            unsafe { bindings::blk_free_queue(queue_ptr) };
        }
    }
}

/// TODO
#[repr(transparent)]
pub struct BoundRequestQueue<T: Operations>(ARef<RequestQueue<T>>, PhantomData<T>);

impl<T: Operations> BoundRequestQueue<T> {
    // TODO
    pub(crate) fn into_raw(self) -> *mut bindings::request_queue {
        ManuallyDrop::new(self).0.as_raw().cast_mut()
    }

    // TODO
    pub(crate) unsafe fn from_raw(request_queue: *mut bindings::request_queue) -> Self {
        BoundRequestQueue(
            unsafe { ARef::from_raw(NonNull::new_unchecked(request_queue).cast()) },
            PhantomData,
        )
    }
}

impl<T: Operations> Drop for BoundRequestQueue<T> {
    fn drop(&mut self) {
        let queue = self.0.deref().0.get();

        // SAFETY: We own the queue binding.
        unsafe { bindings::blk_mq_destroy_queue(queue) }
    }
}

impl<T: Operations> Deref for BoundRequestQueue<T> {
    type Target = ARef<RequestQueue<T>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

unsafe impl<T> Send for BoundRequestQueue<T>
where
    T: Operations,
    T::QueueData: Send,
{
}

unsafe impl<T> Sync for BoundRequestQueue<T>
where
    T: Operations,
    T::QueueData: Sync,
{
}
