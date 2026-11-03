// SPDX-License-Identifier: GPL-2.0

#![allow(dead_code)]

use crate::lu::{QueueData, UfsLuBlockOps};
use crate::host::UfsHostShared;
use crate::protocol::query::*;
use crate::protocol::UfsCmd;
use crate::queue::*;
use kernel::block::mq::{self, BoundRequestQueue, LimitsBuilder, RequestQueue};
use kernel::io::poll::read_poll_timeout;
use kernel::sync::{Arc, Mutex};
use kernel::time::{delay::fsleep, Delta, Instant, Monotonic};
use kernel::prelude::*;

const FDEVICE_COMPL_TIMEOUT_MS: i64 = 1500;
const FDEVICE_COMPL_TICK_US: i64 = 500;

#[derive(Default)]
pub(crate) struct UfsDevInfo {
    max_lu: usize,
    num_lu: usize,
    num_wlu: usize,
    manufacturer_id: u16,
    spec_version: u16,
    queue_depth: usize,
    rtt_cap: u8,
    luns_avail: usize,
    luns: KVec<UfsLuInfo>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct UfsLuInfo {
    enabled: bool,
    logical_block_shift: u8,
    logical_block_count: u64,
    queue_depth: usize,
}

impl UfsLuInfo {
    fn from_desc(desc: UnitDesc) -> Self {
        Self {
            enabled: desc.enabled(),
            logical_block_shift: desc.logical_block_shift(),
            logical_block_count: desc.logical_block_count(),
            queue_depth: desc.lu_queue_depth(),
        }
    }

    pub(crate) fn enabled(&self) -> bool {
        self.enabled
    }

    pub(crate) fn logical_block_shift(&self) -> u8 {
        self.logical_block_shift
    }

    pub(crate) fn logical_block_count(&self) -> u64 {
        self.logical_block_count
    }

    pub(crate) fn queue_depth(&self) -> usize {
        self.queue_depth
    }
}

#[pin_data]
pub(crate) struct UfsDev {
    ufs_queue: Arc<UfsQueue>,
    pub(crate) request_queue: BoundRequestQueue<UfsLuBlockOps>,

    info: Arc<Mutex<UfsDevInfo>>,

}

impl UfsDev {
    pub(crate) fn complete_recovery_dev_init(
        recovery_transfer: RecoveryTransferLease<'_>,
    ) -> Result<RecoveryTransferLease<'_>> {
        let set_device_init = UfsDevCmd::query()
            .set_flag(FlagIdn::FDeviceInit, 0, 0)
            .get_device()?;
        let (mut recovery_transfer, _) =
            recovery_transfer.execute_device_command(set_device_init)?;
        let start: Instant<Monotonic> = Instant::now();

        loop {
            let read_device_init = UfsDevCmd::query()
                .read_flag(FlagIdn::FDeviceInit, 0, 0)
                .get_device()?;
            let (next, cmd) =
                recovery_transfer.execute_device_command(read_device_init)?;
            recovery_transfer = next;

            if cmd.get_device()?.get_query()?.get_flag_value()? == 0 {
                return Ok(recovery_transfer);
            }
            if start.elapsed() > Delta::from_millis(FDEVICE_COMPL_TIMEOUT_MS) {
                pr_err!("[RUFS] ufs_dev: recovery fDeviceInit was not cleared\n");
                return Err(EBUSY);
            }
            fsleep(Delta::from_micros(FDEVICE_COMPL_TICK_US));
        }
    }

    fn recovery_read_desc<'a>(
        recovery_transfer: RecoveryTransferLease<'a>,
        idn: DescIdn,
        index: u8,
    ) -> Result<(RecoveryTransferLease<'a>, Desc)> {
        let read_desc = UfsDevCmd::query()
            .read_desc(idn, index, 0)
            .get_device()?;
        let (recovery_transfer, cmd) =
            recovery_transfer.execute_device_command(read_desc)?;
        let desc = cmd
            .get_device()?
            .get_query()?
            .get_read_desc()?
            .desc;
        Ok((recovery_transfer, desc))
    }

    pub(crate) fn validate_recovery_params<'a>(
        recovery_transfer: RecoveryTransferLease<'a>,
        info: &Mutex<UfsDevInfo>,
    ) -> Result<RecoveryTransferLease<'a>> {
        let (recovery_transfer, geometry) =
            Self::recovery_read_desc(recovery_transfer, DescIdn::Geometry, 0)?;
        let geometry = geometry.get_geometry()?;
        let current_max_lu = match geometry.max_number_lu() {
            1 => 32,
            _ => 8,
        };
        if info.lock().max_lu != current_max_lu {
            pr_err!("[RUFS] ufs_dev: geometry changed after controller reset\n");
            return Err(EIO);
        }

        let (mut recovery_transfer, device) =
            Self::recovery_read_desc(recovery_transfer, DescIdn::Device, 0)?;
        let device = device.get_device()?;
        let num_lu = {
            let info = info.lock();
            if info.manufacturer_id != device.manufacturer_id()
                || info.spec_version != device.spec_version()
                || info.queue_depth != device.queue_depth()
                || info.rtt_cap != device.device_rtt_cap()
                || info.num_lu != device.number_lu() as usize
                || info.num_wlu != device.number_wlu() as usize
                || info.luns_avail != (device.number_lu() + device.number_wlu()) as usize
            {
                pr_err!("[RUFS] ufs_dev: device parameters changed after controller reset\n");
                return Err(EIO);
            }
            info.num_lu
        };

        for lun in 0..num_lu {
            let lun = u8::try_from(lun).map_err(|_| EOVERFLOW)?;
            let (next, desc) =
                Self::recovery_read_desc(recovery_transfer, DescIdn::Unit, lun)?;
            recovery_transfer = next;
            let current = UfsLuInfo::from_desc(desc.get_unit()?);
            let matches = info
                .lock()
                .luns
                .get(lun as usize)
                .is_some_and(|expected| *expected == current);
            if !matches {
                pr_err!(
                    "[RUFS] ufs_dev: LU {} topology changed after controller reset\n",
                    lun,
                );
                return Err(EIO);
            }
        }

        Ok(recovery_transfer)
    }

    pub(crate) fn new(
        ufs_queue: Arc<UfsQueue>,
        host: Arc<UfsHostShared>,
        info: Arc<Mutex<UfsDevInfo>>,
    ) -> Result<Arc<Self>> {
        let limits = LimitsBuilder::<UfsLuBlockOps>::new().build()?;

        let request_queue = RequestQueue::new(
            ufs_queue.tags.clone(),
            limits,
            KBox::new(QueueData::dev(ufs_queue.clone(), host), GFP_KERNEL)?,
            ufs_queue.tags.queue_depth(),
        )?;

        let this = Arc::pin_init(
            try_pin_init!(Self {
                ufs_queue,
                request_queue,
                info,
            }),
            GFP_KERNEL,
        )?;

        Ok(this)
    }

    fn submit(&self, cmd: UfsCmd) -> Result<UfsCmd> {
        let mut rq = self
            .request_queue
            .alloc_sync_request(mq::Command::DriverOut)?;
        rq.data_ref().inner.lock().prepare_device(cmd)?;
        rq.as_pin_mut().execute(true)?;
        let result = rq.data_ref().inner.lock().take_device_completion();
        result
    }

    fn nop(&self) -> Result<()> {
        self.submit(UfsDevCmd::nop())?;
        Ok(())
    }

    pub(crate) fn verify_dev_init(&self) -> Result<()> {
        self.nop()
    }

    fn read_desc(&self, idn: DescIdn, index: u8, selector: u8) -> Result<Desc> {
        let cmd = self.submit(UfsDevCmd::query().read_desc(idn, index, selector))?;
        Ok(cmd.get_device()?.get_query()?.get_read_desc()?.desc)
    }

    fn read_attr(&self, idn: AttrIdn, index: u8, selector: u8) -> Result<u64> {
        let cmd = self.submit(UfsDevCmd::query().read_attr(idn, index, selector))?;
        cmd.get_device()?.get_query()?.get_attr_value()
    }

    fn read_unit_desc(&self, lun: u8) -> Result<UnitDesc> {
        self.read_desc(DescIdn::Unit, lun, 0)?.get_unit()
    }

    pub(crate) fn lu_info(&self, lun: usize) -> Result<UfsLuInfo> {
        self.info.lock().luns.get(lun).copied().ok_or(EINVAL)
    }

    pub(crate) fn num_lu(&self) -> usize {
        self.info.lock().num_lu
    }

    fn write_attr(&self, idn: AttrIdn, index: u8, selector: u8, value: u64) -> Result<()> {
        let cmd = self.submit(UfsDevCmd::query().write_attr(idn, index, selector, value))?;
        if cmd.get_device()?.get_query()?.get_attr_value()? == value {
            Ok(())
        } else {
            Err(EIO)
        }
    }

    fn read_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<u8> {
        let cmd = self.submit(UfsDevCmd::query().read_flag(idn, index, selector))?;
        cmd.get_device()?.get_query()?.get_flag_value()
    }

    fn set_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<()> {
        self.submit(UfsDevCmd::query().set_flag(idn, index, selector))?;
        Ok(())
    }

    fn clear_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<()> {
        self.submit(UfsDevCmd::query().clear_flag(idn, index, selector))?;
        Ok(())
    }

    fn toggle_flag(&self, idn: FlagIdn, index: u8, selector: u8) -> Result<u8> {
        let cmd = self.submit(UfsDevCmd::query().toggle_flag(idn, index, selector))?;
        cmd.get_device()?.get_query()?.get_flag_value()
    }

    pub(crate) fn complete_dev_init(&self) -> Result<()> {
        self.set_flag(FlagIdn::FDeviceInit, 0, 0)?;

        let result = read_poll_timeout(
            || self.read_flag(FlagIdn::FDeviceInit, 0, 0),
            |flag: &u8| *flag == 0,
            Delta::from_micros(FDEVICE_COMPL_TICK_US),
            Delta::from_millis(FDEVICE_COMPL_TIMEOUT_MS),
        );
        match result {
            Ok(_) => Ok(()),
            Err(ETIMEDOUT) => {
                pr_err!("[RUFS] ufs_dev: fDeviceInit was not cleared\n");
                Err(EBUSY)
            }
            Err(e) => {
                pr_err!(
                    "[RUFS] ufs_dev: failed to read fDeviceInit errno={}\n",
                    e.to_errno(),
                );
                Err(e)
            }
        }
    }

    pub(crate) fn device_params_init(&self) -> Result<()> {
        self.get_geometry_info()?;
        self.get_device_info()?;
        let num_lu = self.info.lock().num_lu;
        let mut luns = KVec::with_capacity(num_lu, GFP_KERNEL)?;
        for lun in 0..num_lu {
            let lun = u8::try_from(lun).map_err(|_| EOVERFLOW)?;
            let desc = self.read_unit_desc(lun)?;
            luns.push(UfsLuInfo::from_desc(desc), GFP_KERNEL)?;
        }
        self.info.lock().luns = luns;
        Ok(())
    }

    fn get_geometry_info(&self) -> Result<GeometryDesc> {
        let desc = self.read_desc(DescIdn::Geometry, 0, 0)?.get_geometry()?;
        match desc.max_number_lu() {
            1 => {
                self.info.lock().max_lu = 32;
            }
            _ => {
                self.info.lock().max_lu = 8;
            }
        }

        Ok(desc)
    }

    fn get_device_info(&self) -> Result<DeviceDesc> {
        let desc = self.read_desc(DescIdn::Device, 0, 0)?.get_device()?;
        self.info.lock().manufacturer_id = desc.manufacturer_id();
        self.info.lock().spec_version = desc.spec_version();
        self.info.lock().queue_depth = desc.queue_depth();
        self.info.lock().rtt_cap = desc.device_rtt_cap();
        self.info.lock().num_lu = desc.number_lu() as usize;
        self.info.lock().num_wlu = desc.number_wlu() as usize;
        self.info.lock().luns_avail = (desc.number_lu() + desc.number_wlu()) as usize;

        Ok(desc)
    }
}
