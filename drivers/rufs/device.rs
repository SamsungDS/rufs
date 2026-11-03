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
use kernel::time::Delta;
use kernel::{new_mutex, prelude::*};

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
}

#[pin_data]
pub(crate) struct UfsDev {
    ufs_queue: Arc<UfsQueue>,
    pub(crate) request_queue: BoundRequestQueue<UfsLuBlockOps>,

    #[pin]
    pub(crate) info: Mutex<UfsDevInfo>,

}

impl UfsDev {
    pub(crate) fn new(
        ufs_queue: Arc<UfsQueue>,
        host: Arc<UfsHostShared>,
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
                info <- new_mutex!(UfsDevInfo::default()),
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

    pub(crate) fn read_unit_desc(&self, lun: u8) -> Result<UnitDesc> {
        self.read_desc(DescIdn::Unit, lun, 0)?.get_unit()
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
