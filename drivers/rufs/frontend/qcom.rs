// SPDX-License-Identifier: GPL-2.0

//! Qualcomm platform frontend for the UFS driver.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use kernel::{
    clk::{Clk, Hertz},
    device,
    device::Core,
    gpio::OptionalOutput,
    interconnect::Path,
    io::{poll::read_poll_timeout, register, Io},
    macros::vtable,
    new_mutex,
    of,
    opp,
    phy::{Phy, UfsMode},
    platform,
    prelude::*,
    regulator,
    reset::OptionalExclusive,
    str::CString,
    sync::{aref::ARef, Arc, Mutex},
    time::{delay::fsleep, Delta},
};

use crate::host::UfsHost;
use crate::reg::{
    McqQueueConfigLayout, McqRegisterLayout, McqRegisterRegion,
    UfsMcqOprInfo, UfsMcqOprSet, UfsReg,
};
use crate::resource::{HciMmio, HostResources, McqMmio};
use crate::uic::{UfsPaLayerAttr, UfsUic};
use crate::variant::{NotifyPhase, UfsVariantOps};

pub(crate) struct UfsQcom;

#[derive(Clone, Copy)]
pub(crate) enum UfsQcomVariant {
    Eliza,
    Kaanapali,
    Sm8550,
    Sm8650,
    Sm8750,
    X1e80100,
}

impl UfsQcomVariant {
    const fn name(self) -> &'static str {
        match self {
            Self::Eliza => "eliza",
            Self::Kaanapali => "kaanapali",
            Self::Sm8550 => "sm8550",
            Self::Sm8650 => "sm8650",
            Self::Sm8750 => "sm8750",
            Self::X1e80100 => "x1e80100",
        }
    }

    fn max_clock_rate(self) -> Result<Hertz> {
        match self {
            Self::Sm8550 | Self::X1e80100 => Ok(Hertz::from_mhz(300)),
            Self::Eliza | Self::Kaanapali | Self::Sm8650 | Self::Sm8750 => {
                Ok(Hertz::from_mhz(403))
            }
        }
    }

    const fn has_mcq_resource(self) -> bool {
        matches!(self, Self::Eliza)
    }
}

const CLOCK_NAMES: [&CStr; 8] = [
    c"core_clk",
    c"bus_aggr_clk",
    c"iface_clk",
    c"core_clk_unipro",
    c"ref_clk",
    c"tx_lane0_sync_clk",
    c"rx_lane0_sync_clk",
    c"rx_lane1_sync_clk",
];

const CORE_CLOCK_INDEX: usize = 0;
const UNIPRO_CORE_CLOCK_INDEX: usize = 3;
const CONTROL_CLOCK_COUNT: usize = 5;

const MAX_MEMORY_BANDWIDTH: u32 = 7_643_136;
const MAX_CONFIG_BANDWIDTH: u32 = 819_200;

const MCQ_CONFIG_BASE: usize = 0x1c000;
const MCQ_QUEUE_CONFIG_STRIDE: usize = 0x40;
const MCQ_SQD_OFFSET: usize = 0x5000;
const MCQ_SQIS_OFFSET: usize = 0x5040;
const MCQ_CQD_OFFSET: usize = 0x5080;
const MCQ_CQIS_OFFSET: usize = 0x50c0;
const MCQ_OPERATION_STRIDE: usize = 0x100;
const QCOM_MAX_MCQ_ACTIVE_COMMANDS: usize = 64;

const MPHY_TX_FSM_STATE: u32 = 0x41;
const TX_FSM_HIBERN8: u32 = 0x1;
const PA_LOCAL_TX_LCC_ENABLE: u32 = 0x155e;
const PA_TXHSADAPTTYPE: u32 = 0x15d4;
const PA_INITIAL_ADAPT: u32 = 0x1;
const PA_NO_ADAPT: u32 = 0x3;
const UFS_HS_G4: u32 = 4;
const PA_VS_CLK_CFG: u32 = 0x9004;
const PA_VS_CLK_CFG_MASK: u32 = 0x1ff;
const DL_VS_CLK_CFG: u32 = 0xa00b;
const DL_VS_CLK_CFG_MASK: u32 = 0x3ff;
const PA_VS_CORE_CLK_40NS_CYCLES: u32 = 0x9007;
const DME_VS_CORE_CLK_CTRL: u32 = 0xd002;

const PA_HS_MODE_A: u32 = 1;
const CORE_CLK_DIV_EN: u32 = 1 << 8;
const DME_HW_CGC_EN: u32 = 1 << 9;
const CORE_CLK_CYCLES_MASK: u32 = 0xff;
const CORE_CLK_CYCLES_MASK_V4: u32 = 0x0fff << 16;
const CORE_CLK_40NS_CYCLES_MASK: u32 = 0x7f;

#[derive(Default)]
struct UfsQcomOppOps;

#[vtable]
impl opp::ConfigOps for UfsQcomOppOps {
    fn config_clks(
        dev: &device::Device,
        _table: &opp::Table,
        selected: &opp::OPP,
        _scaling_down: bool,
    ) -> Result {
        for (index, name) in CLOCK_NAMES.into_iter().enumerate() {
            let rate = selected.freq(Some(u32::try_from(index).map_err(|_| EOVERFLOW)?));
            if rate.as_hz() == 0 {
                continue;
            }

            Clk::get(dev, Some(name))?.set_rate(rate)?;
        }
        Ok(())
    }
}

struct UfsQcomOpp {
    _table: opp::Table,
    _config: opp::ConfigToken,
}

impl UfsQcomOpp {
    fn new(dev: &device::Device<device::Bound>, max_rate: Hertz) -> Result<Self> {
        let mut names = KVec::new();
        for name in CLOCK_NAMES {
            names.push(CString::try_from(name)?, GFP_KERNEL)?;
        }

        let config = opp::Config::<UfsQcomOppOps>::new()
            .set_clk_names(names)?
            .set(dev)?;
        let dev_ref: ARef<device::Device> = dev.into();
        let table = opp::Table::from_of(&dev_ref, 0)?;
        let selected = table.opp_from_freq(
            max_rate,
            Some(true),
            Some(0),
            opp::SearchType::Exact,
        )?;
        table.set_opp(&selected)?;

        Ok(Self {
            _table: table,
            _config: config,
        })
    }
}

register! {
    QCOM_SYS1CLK_1US(u32) @ 0xc0 { 31:0 value; }
    QCOM_PARAM0(u32) @ 0xd0 { 6:4 max_hs_gear; }
    QCOM_CFG0(u32) @ 0xd8 { 5:5 qunipro_g4_select => bool; }
    QCOM_CFG1(u32) @ 0xdc {
        26:26 device_ref_clock_enable => bool;
        0:0 qunipro_select => bool;
    }
    QCOM_CFG2(u32) @ 0xe0 { 7:0 clock_gating; }
    QCOM_HW_VERSION(u32) @ 0xe4 {
        31:28 major;
        27:16 minor;
        15:0 step;
    }
}

struct UfsQcomClocks {
    clocks: KVec<Clk>,
    control_enabled: usize,
    lanes_enabled: AtomicBool,
}

impl UfsQcomClocks {
    fn new(dev: &device::Device<device::Bound>) -> Result<Self> {
        let mut this = Self {
            clocks: KVec::new(),
            control_enabled: 0,
            lanes_enabled: AtomicBool::new(false),
        };

        for name in CLOCK_NAMES {
            let clock = Clk::get(dev, Some(name))?;
            this.clocks.push(clock, GFP_KERNEL)?;
        }

        while this.control_enabled < CONTROL_CLOCK_COUNT {
            this.clocks[this.control_enabled].prepare_enable()?;
            this.control_enabled += 1;
        }

        Ok(this)
    }

    fn enable_lanes(&self) -> Result {
        if self.lanes_enabled.load(Ordering::Acquire) {
            return Ok(());
        }

        let mut enabled = 0;
        for clock in &self.clocks[CONTROL_CLOCK_COUNT..] {
            if let Err(e) = clock.prepare_enable() {
                for clock in self.clocks[CONTROL_CLOCK_COUNT..CONTROL_CLOCK_COUNT + enabled]
                    .iter()
                    .rev()
                {
                    clock.disable_unprepare();
                }
                return Err(e);
            }
            enabled += 1;
        }

        self.lanes_enabled.store(true, Ordering::Release);
        Ok(())
    }

    fn disable_lanes(&self) {
        if !self.lanes_enabled.swap(false, Ordering::AcqRel) {
            return;
        }

        for clock in self.clocks[CONTROL_CLOCK_COUNT..].iter().rev() {
            clock.disable_unprepare();
        }
    }

    fn cycles_per_microsecond(&self, index: usize) -> u32 {
        let hz = self.clocks[index].rate().as_hz() as u64;
        let hz = core::cmp::max(hz, 1_000_000);

        ((hz + 999_999) / 1_000_000) as u32
    }
}

impl Drop for UfsQcomClocks {
    fn drop(&mut self) {
        self.disable_lanes();
        for clock in self.clocks[..self.control_enabled].iter().rev() {
            clock.disable_unprepare();
        }
    }
}

struct UfsQcomInterconnect {
    _ddr: Path,
    _cpu: Path,
}

impl UfsQcomInterconnect {
    fn new(dev: &device::Device<device::Bound>) -> Result<Self> {
        let ddr = Path::get(dev, c"ufs-ddr")?;
        let cpu = Path::get(dev, c"cpu-ufs")?;

        ddr.set_bw(0, MAX_MEMORY_BANDWIDTH)?;
        if let Err(e) = cpu.set_bw(0, MAX_CONFIG_BANDWIDTH) {
            let _ = ddr.set_bw(0, 0);
            return Err(e);
        }

        Ok(Self {
            _ddr: ddr,
            _cpu: cpu,
        })
    }
}

struct UfsQcomPlatform {
    has_mcq_resource: bool,
    // Serialize PHY transitions and retain OPP resources until shutdown.
    lifecycle: Arc<Mutex<Option<UfsQcomOpp>>>,
    clocks: UfsQcomClocks,
    _interconnect: UfsQcomInterconnect,
    reset: OptionalExclusive,
    device_reset: OptionalOutput,
    phy: Phy,
    hardware_major: AtomicU32,
    phy_gear: AtomicU32,
}

impl UfsQcomPlatform {
    fn enable_supplies(dev: &device::Device<device::Bound>) -> Result {
        let Some(fwnode) = dev.fwnode() else {
            return Ok(());
        };

        for (property, supply) in [
            (c"vcc-supply", c"vcc"),
            (c"vccq-supply", c"vccq"),
            (c"vccq2-supply", c"vccq2"),
        ] {
            if fwnode.property_present(property) {
                regulator::devm_enable(dev, supply)?;
            }
        }
        Ok(())
    }

    fn new(dev: &device::Device<device::Bound>, variant: UfsQcomVariant) -> Result<Self> {
        Self::enable_supplies(dev)?;
        let opp = UfsQcomOpp::new(dev, variant.max_clock_rate()?)?;

        Ok(Self {
            has_mcq_resource: variant.has_mcq_resource(),
            lifecycle: Arc::pin_init(new_mutex!(Some(opp)), GFP_KERNEL)?,
            clocks: UfsQcomClocks::new(dev)?,
            _interconnect: UfsQcomInterconnect::new(dev)?,
            reset: OptionalExclusive::get(dev, c"rst")?,
            // Hold the attached device in reset until `device_reset()` runs.
            device_reset: OptionalOutput::get(dev, c"reset", true)?,
            phy: Phy::get(dev, c"ufsphy")?,
            hardware_major: AtomicU32::new(0),
            phy_gear: AtomicU32::new(0),
        })
    }

    fn hardware_major(&self, reg: &UfsReg) -> Result<u32> {
        Ok(reg.hci_access()?.read(QCOM_HW_VERSION).major().get())
    }

    fn initial_phy_config(&self, reg: &UfsReg) -> Result<(u32, UfsMode)> {
        let access = reg.hci_access()?;
        let major = access.read(QCOM_HW_VERSION).major().get();
        let gear = if major < 4 {
            2
        } else {
            access.read(QCOM_PARAM0).max_hs_gear().get()
        };
        if gear == 0 {
            return Err(EINVAL);
        }

        let mode = if major == 5 && gear == 5 {
            UfsMode::HighSpeedA
        } else {
            UfsMode::HighSpeedB
        };
        Ok((gear, mode))
    }

    fn select_qunipro(&self, reg: &UfsReg, hardware_major: u32) -> Result {
        let access = reg.hci_access()?;
        access.update(QCOM_CFG1, |value| {
            value
                .with_qunipro_select(true)
                .with_device_ref_clock_enable(true)
        });
        access.read(QCOM_CFG1);

        if hardware_major >= 5 {
            access.update(QCOM_CFG0, |value| value.with_qunipro_g4_select(false));
            access.read(QCOM_CFG0);
        }

        Ok(())
    }

    fn power_up(&self, reg: &UfsReg) -> Result {
        let _lifecycle = self.lifecycle.lock();

        self.clocks.disable_lanes();
        self.phy.shutdown();

        self.reset.assert()?;
        fsleep(Delta::from_micros(200));
        self.reset.deassert()?;
        fsleep(Delta::from_millis(1));

        let hardware_major = self.hardware_major(reg)?;
        let (gear, mode) = self.initial_phy_config(reg)?;
        if let Err(e) = (|| {
            self.phy.init()?;
            self.phy.set_ufs_mode(mode, gear as i32)?;
            self.phy.power_on()?;
            self.phy.calibrate()?;
            self.select_qunipro(reg, hardware_major)?;
            self.clocks.enable_lanes()
        })() {
            self.clocks.disable_lanes();
            self.phy.shutdown();
            return Err(e);
        }

        self.hardware_major.store(hardware_major, Ordering::Release);
        self.phy_gear.store(gear, Ordering::Release);
        Ok(())
    }

    fn check_hibern8(&self, uic: &UfsUic) -> Result {
        read_poll_timeout(
            || uic.dme_get_sel(MPHY_TX_FSM_STATE, 0),
            |state| *state == TX_FSM_HIBERN8,
            Delta::from_micros(100),
            Delta::from_millis(100),
        )?;
        Ok(())
    }

    fn set_unipro_clock_cycles(&self, uic: &UfsUic, hardware_major: u32) -> Result {
        let cycles = self.clocks.cycles_per_microsecond(UNIPRO_CORE_CLOCK_INDEX);
        let (mask, value) = if hardware_major >= 4 {
            (CORE_CLK_CYCLES_MASK_V4, cycles << 16)
        } else {
            (CORE_CLK_CYCLES_MASK, cycles)
        };
        if value & !mask != 0 {
            return Err(ERANGE);
        }

        let mut register = uic.dme_get(DME_VS_CORE_CLK_CTRL)?;
        register &= !(mask | CORE_CLK_DIV_EN);
        register |= value;
        uic.dme_set(DME_VS_CORE_CLK_CTRL, register)?;

        if hardware_major < 4 {
            return Ok(());
        }

        let cycles_40ns = match cycles {
            403 => 16,
            300 => 12,
            202 => 8,
            150 => 6,
            100 => 4,
            75 => 3,
            38 => 2,
            _ => return Err(EINVAL),
        };
        let mut register = uic.dme_get(PA_VS_CORE_CLK_40NS_CYCLES)?;
        register &= !CORE_CLK_40NS_CYCLES_MASK;
        register |= cycles_40ns;
        uic.dme_set(PA_VS_CORE_CLK_40NS_CYCLES, register)
    }

    fn configure_link_startup(&self, reg: &UfsReg, uic: &UfsUic) -> Result {
        self.check_hibern8(uic)?;
        self.enable_unipro_clock_gating(uic);

        let core_cycles = self.clocks.cycles_per_microsecond(CORE_CLOCK_INDEX);
        reg.hci_access()?
            .write_reg(QCOM_SYS1CLK_1US::zeroed().with_value(core_cycles));
        reg.hci_access()?.read(QCOM_SYS1CLK_1US);

        self.set_unipro_clock_cycles(uic, self.hardware_major(reg)?)?;
        uic.dme_set(PA_LOCAL_TX_LCC_ENABLE, 0)
    }

    fn dme_set_bits(&self, uic: &UfsUic, attr: u32, mask: u32) -> Result {
        let value = uic.dme_get(attr)?;
        uic.dme_set(attr, value | mask)
    }

    fn enable_unipro_clock_gating(&self, uic: &UfsUic) {
        for (attr, mask) in [
            (DL_VS_CLK_CFG, DL_VS_CLK_CFG_MASK),
            (PA_VS_CLK_CFG, PA_VS_CLK_CFG_MASK),
            (DME_VS_CORE_CLK_CTRL, DME_HW_CGC_EN),
        ] {
            if let Err(e) = self.dme_set_bits(uic, attr, mask) {
                pr_warn!(
                    "[RUFS] Qualcomm: failed to enable clock gating attr={:#x} errno={}\n",
                    attr,
                    e.to_errno(),
                );
                break;
            }
        }
    }

    fn configure_adaptation(&self, uic: &UfsUic, gear: u32) {
        let adapt = if gear >= UFS_HS_G4 {
            PA_INITIAL_ADAPT
        } else {
            PA_NO_ADAPT
        };
        if let Err(e) = uic.dme_set(PA_TXHSADAPTTYPE, adapt) {
            pr_warn!(
                "[RUFS] Qualcomm: failed to configure PA adaptation errno={}\n",
                e.to_errno(),
            );
        }
    }
}

impl UfsVariantOps for UfsQcomPlatform {
    fn mcq_register_layout(&self, _reg: &UfsReg) -> Result<McqRegisterLayout> {
        if !self.has_mcq_resource {
            return Err(ENODEV);
        }

        let operation = |offset| {
            UfsMcqOprInfo::new(
                McqRegisterRegion::Mcq,
                offset,
                MCQ_CONFIG_BASE + offset,
                MCQ_OPERATION_STRIDE,
            )
        };

        Ok(McqRegisterLayout::new(
            McqQueueConfigLayout::new(
                McqRegisterRegion::Mcq,
                0,
                MCQ_QUEUE_CONFIG_STRIDE,
            ),
            UfsMcqOprSet::new(
                operation(MCQ_SQD_OFFSET),
                operation(MCQ_SQIS_OFFSET),
                operation(MCQ_CQD_OFFSET),
                operation(MCQ_CQIS_OFFSET),
            ),
        ))
    }

    fn mcq_enabled(&self) -> bool {
        self.has_mcq_resource
    }

    fn constrain_mcq_active_commands(&self, reported: usize) -> usize {
        core::cmp::min(reported, QCOM_MAX_MCQ_ACTIVE_COMMANDS)
    }

    fn device_reset(&self) -> Result<()> {
        if !self.device_reset.is_present() {
            return Ok(());
        }

        self.device_reset.set_value(true)?;
        fsleep(Delta::from_micros(10));
        self.device_reset.set_value(false)?;
        fsleep(Delta::from_micros(10));
        Ok(())
    }

    fn hce_enable_notify(&self, reg: &UfsReg, phase: NotifyPhase) -> Result<()> {
        match phase {
            NotifyPhase::Pre => self.power_up(reg),
            NotifyPhase::Post => {
                reg.hci_access()?
                    .update(QCOM_CFG2, |value| value.with_clock_gating(0xff));
                reg.hci_access()?.read(QCOM_CFG2);
                Ok(())
            }
        }
    }

    fn link_startup_notify(&self, reg: &UfsReg, uic: &UfsUic, phase: NotifyPhase) -> Result<()> {
        match phase {
            NotifyPhase::Pre => self.configure_link_startup(reg, uic),
            NotifyPhase::Post => Ok(()),
        }
    }

    fn constrain_power_mode(&self, mut desired: UfsPaLayerAttr) -> Result<UfsPaLayerAttr> {
        let phy_gear = self.phy_gear.load(Ordering::Acquire);
        if phy_gear == 0 {
            return Err(EINVAL);
        }

        desired.gear_rx = core::cmp::min(desired.gear_rx, phy_gear);
        desired.gear_tx = core::cmp::min(desired.gear_tx, phy_gear);
        if self.hardware_major.load(Ordering::Acquire) == 5 && phy_gear == 5 {
            desired.hs_rate = PA_HS_MODE_A;
        }
        Ok(desired)
    }

    fn power_mode_notify(
        &self,
        _reg: &UfsReg,
        uic: &UfsUic,
        mode: UfsPaLayerAttr,
        phase: NotifyPhase,
    ) -> Result<()> {
        if matches!(phase, NotifyPhase::Pre)
            && self.hardware_major.load(Ordering::Acquire) >= 4
        {
            self.configure_adaptation(uic, mode.gear_tx);
        }
        Ok(())
    }

    fn shutdown(&self, _reg: &UfsReg) {
        let mut lifecycle = self.lifecycle.lock();

        self.clocks.disable_lanes();
        self.phy.shutdown();

        // Remove static OPPs before generic PM-domain detach so a later bind
        // can configure the device's OPP table again.
        let opp = lifecycle.take();
        drop(lifecycle);
        drop(opp);
    }
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <UfsQcom as platform::Driver>::IdInfo,
    [
        (
            of::DeviceId::new(c"qcom,eliza-ufshc"),
            UfsQcomVariant::Eliza,
        ),
        (
            of::DeviceId::new(c"qcom,kaanapali-ufshc"),
            UfsQcomVariant::Kaanapali,
        ),
        (
            of::DeviceId::new(c"qcom,x1e80100-ufshc"),
            UfsQcomVariant::X1e80100,
        ),
        (
            of::DeviceId::new(c"qcom,sm8550-ufshc"),
            UfsQcomVariant::Sm8550,
        ),
        (
            of::DeviceId::new(c"qcom,sm8650-ufshc"),
            UfsQcomVariant::Sm8650,
        ),
        (
            of::DeviceId::new(c"qcom,sm8750-ufshc"),
            UfsQcomVariant::Sm8750,
        ),
    ]
);

#[pin_data]
pub(crate) struct UfsQcomData<'a> {
    pdev: &'a platform::Device,
    #[pin]
    host: UfsHost,
}

impl platform::Driver for UfsQcom {
    type IdInfo = UfsQcomVariant;
    type Data<'bound> = UfsQcomData<'bound>;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        variant: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        pin_init::pin_init_scope(move || {
            let variant = *variant.ok_or(ENODEV)?;
            let hci = HciMmio::from_platform(pdev)?;
            let mcq = if variant.has_mcq_resource() {
                let request = pdev.io_request_by_name(c"mcq").ok_or(ENODEV)?;
                Some(McqMmio::from_platform(
                    request.iomap()?.into_devres()?,
                ))
            } else {
                None
            };
            let platform = UfsQcomPlatform::new(pdev.as_ref(), variant)?;

            dev_info!(
                pdev.as_ref(),
                "RUFS Qualcomm frontend: variant={}\n",
                variant.name(),
            );

            let resources = HostResources::new(
                pdev.as_ref().into(),
                hci,
                mcq,
                KBox::new(platform, GFP_KERNEL)? as KBox<dyn UfsVariantOps>,
            )?;
            let controller_irq = pdev.irq_by_index(0)?;
            let host = UfsHost::new(resources, controller_irq);

            Ok(try_pin_init!(UfsQcomData { pdev, host <- host }))
        })
    }

    fn unbind(pdev: &platform::Device<Core<'_>>, this: Pin<&Self::Data<'_>>) {
        dev_dbg!(pdev.as_ref(), "Remove Rust Qualcomm UFS driver.\n");
        this.host.shutdown();
    }
}
