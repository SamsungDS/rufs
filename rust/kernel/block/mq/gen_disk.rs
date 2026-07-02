// SPDX-License-Identifier: GPL-2.0

//! Generic disk abstraction.
//!
//! C header: [`include/linux/blkdev.h`](srctree/include/linux/blkdev.h)
//! C header: [`include/linux/blk-mq.h`](srctree/include/linux/blk-mq.h)

use crate::{
    bindings,
    block::mq::{
        operations::OperationsVTable,
        Operations,
        RequestQueue,
        TagSet, //
    },
    error::{
        from_err_ptr,
        Result, //
    },
    fmt::{
        self,
        Write, //
    },
    prelude::*,
    static_lock_class,
    str::NullTerminatedFormatter,
    sync::{
        aref::ARef,
        Arc, //
    },
    types::{
        ForeignOwnable,
        Opaque,
        RefCounted,
        ScopeGuard, //
    },
};
use core::{
    marker::PhantomData,
    ops::Deref,
    ptr::NonNull, //
};

#[cfg(CONFIG_BLK_DEV_ZONED)]
use super::Feature;
use super::{request_queue::Limits, BoundRequestQueue};

/// A generic block device.
///
/// # Invariants
///
///  - `gendisk` must always point to an initialized and valid `struct gendisk`.
///  - `self.gendisk.queue.queuedata` is initialized by a call to `ForeignOwnable::into_foreign`.
#[repr(transparent)]
pub struct GenDisk<T: Operations> {
    gendisk: Opaque<bindings::gendisk>,
    _p: PhantomData<T>,
}

impl<T: Operations> GenDisk<T> {
    /// TODO
    pub fn new_for_queue(
        name: fmt::Arguments<'_>,
        request_queue: BoundRequestQueue<T>,
        capacity_sectors: u64,
        data: T::GenDiskData,
    ) -> Result<BoundGenDisk<T>> {
        #[cfg(CONFIG_BLK_DEV_ZONED)]
        let features = request_queue.limits().features();

        // SAFETY: TODO
        let gendisk = from_err_ptr(unsafe {
            bindings::blk_mq_alloc_disk_for_queue(
                request_queue.into_raw(),
                static_lock_class!().as_ptr(),
            )
        })?;

        // SAFETY: `gendisk` is a valid pointer as we initialized it above
        unsafe { (*gendisk).fops = Self::build_vtable() };

        let mut writer = NullTerminatedFormatter::new(
            // SAFETY: `gendisk` is valid and initialized. We have exclusive
            // access, since the disk is not added to the VFS yet.
            unsafe { &mut (*gendisk).disk_name },
        )
        .ok_or(EINVAL)?;
        writer.write_fmt(name)?;

        // SAFETY: `disk.gendisk` is valid and initialized. `set_capacity` takes a
        // lock to synchronize this operation, so we will not race.
        unsafe { bindings::set_capacity(gendisk, capacity_sectors) };

        let data = data.into_foreign();
        unsafe { (*gendisk).private_data = data };

        let guard = ScopeGuard::new(|| unsafe { ForeignOwnable::from_foreign(data) });

        // SAFETY: `disk.gendisk` is valid and initialized.
        crate::error::to_result(unsafe {
            bindings::device_add_disk(core::ptr::null_mut(), gendisk, core::ptr::null_mut())
        })?;

        guard.dismiss();

        #[cfg(CONFIG_BLK_DEV_ZONED)]
        if features.contains(Feature::Zoned) {
            // SAFETY: `disk.gendisk` is valid and was added to the VFS above.
            unsafe { bindings::blk_revalidate_disk_zones(gendisk) };
        }

        // SAFETY: We one a refcount from the allocation of the disk.
        let disk = unsafe { ARef::from_raw(NonNull::new_unchecked(gendisk).cast()) };

        Ok(BoundGenDisk(disk, PhantomData))
    }

    /// Build a new `GenDisk` and add it to the VFS.
    pub fn new(
        name: fmt::Arguments<'_>,
        tagset: Arc<TagSet<T>>,
        queue_data: T::QueueData,
        queue_limits: Limits,
        gendisk_data: T::GenDiskData,
        capacity_sectors: u64,
        queue_depth: u32,
    ) -> Result<BoundGenDisk<T>> {
        let queue = RequestQueue::new(tagset, queue_limits, queue_data, queue_depth)?;

        Self::new_for_queue(name, queue, capacity_sectors, gendisk_data)
    }

    const VTABLE: bindings::block_device_operations = bindings::block_device_operations {
        submit_bio: None,
        open: None,
        release: None,
        ioctl: None,
        compat_ioctl: None,
        check_events: None,
        unlock_native_capacity: None,
        getgeo: None,
        set_read_only: None,
        swap_slot_free_notify: None,
        report_zones: if T::HAS_REPORT_ZONES {
            Some(OperationsVTable::<T>::report_zones_callback)
        } else {
            None
        },
        devnode: None,
        alternative_gpt_sector: None,
        get_unique_id: None,
        // TODO: Set to THIS_MODULE. Waiting for const_refs_to_static feature to
        // be merged (unstable in rustc 1.78 which is staged for linux 6.10)
        // <https://github.com/rust-lang/rust/issues/119618>
        owner: core::ptr::null_mut(),
        pr_ops: core::ptr::null_mut(),
        free_disk: Some(Self::release),
        poll_bio: None,
    };

    pub(crate) const fn build_vtable() -> &'static bindings::block_device_operations {
        &Self::VTABLE
    }

    /// Get the [`RequestQueue`] associated with this [`GenDisk`].
    pub fn queue(&self) -> &RequestQueue<T> {
        // SAFETY: By type invariant, self is a valid gendisk.
        unsafe { RequestQueue::from_raw((*self.gendisk.get()).queue) }
    }

    /// Get the private data associated with this [`GenDisk`].
    pub fn disk_data(&self) -> <T::GenDiskData as ForeignOwnable>::Borrowed<'_> {
        // SAFETY: By type invariant, self is a valid gendisk.
        unsafe { T::GenDiskData::borrow((*self.gendisk.get()).private_data) }
    }

    extern "C" fn release(this: *mut bindings::gendisk) {
        // SAFETY: We own a bound disk that we dissolved to a pointer during
        // construction.
        drop(unsafe { BoundRequestQueue::<T>::from_raw((*this).queue) });

        let disk_data = unsafe { (*this).private_data };
        // SAFETY: `this.private` was created by `GenDiskBuilder::build` with a
        // call to `ForeignOwnable::into_foreign`.
        // `ForeignOwnable::from_foreign` is only called here.
        drop(unsafe { T::GenDiskData::from_foreign(disk_data) });
    }

    // # Safety: TODO
    pub(crate) unsafe fn form_raw<'a>(ptr: *const bindings::gendisk) -> &'a Self {
        // SAFETY: Self is transparent.
        unsafe { &*ptr.cast() }
    }
}

unsafe impl<T: Operations> RefCounted for GenDisk<T> {
    fn inc_ref(&self) {
        unsafe { bindings::get_device(&raw mut (*(*self.gendisk.get()).part0).bd_device) };
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        unsafe { bindings::put_disk((*obj.as_ptr()).gendisk.get()) };
    }
}

// SAFETY: `GenDisk` is an owned pointer to a `struct gendisk` and an `Arc` to a
// `TagSet`. It is safe to send this to other threads as long as these two are `Send`.
unsafe impl<T> Send for GenDisk<T>
where
    T: Operations,
    T::QueueData: Send,
    Arc<TagSet<T>>: Send,
{
}

// SAFETY: `GenDisk` is an owned pointer to a `struct gendisk` and an `Arc` to a `TagSet`. It is
// safe to reference these from multiple threads if the `Arc` and the `gendisk` private data is
// `Sync`.
unsafe impl<T> Sync for GenDisk<T>
where
    T: Operations,
    T::QueueData: Sync,
    Arc<TagSet<T>>: Sync,
{
}

/// TODO
pub struct BoundGenDisk<T: Operations>(ARef<GenDisk<T>>, PhantomData<T>);

impl<T: Operations> Drop for BoundGenDisk<T> {
    fn drop(&mut self) {
        let disk = self.0.deref().gendisk.get();

        // SAFETY: We own the disk binding;
        unsafe { bindings::del_gendisk(disk) };
    }
}

impl<T: Operations> Deref for BoundGenDisk<T> {
    type Target = ARef<GenDisk<T>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
