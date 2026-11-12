// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Apple AOP ambient light sensor driver
//!
//! Copyright (C) The Asahi Linux Contributors

use kernel::{
    bindings, c_str,
    device::Core,
    ffi::{c_int, c_ulong, c_void},
    firmware::Firmware,
    iio::common::aop_sensors::{AopSensorData, IIORegistration, MessageProcessor},
    module_platform_driver, of, platform,
    prelude::*,
    soc::apple::aop::{EPICService, AOP},
    sync::Arc,
    types::ForeignOwnable,
};

const EPIC_SUBTYPE_GET_AOP_PROPERTY: u16 = 0xa;
const EPIC_SUBTYPE_SET_ALS_PROPERTY: u16 = 0x4;
const LUX_OFFSET_CT720: usize = 0x1d;
const LUX_OFFSET_VD6286: usize = 0x28;
/* ALS property 0: report interval (us). 200 ms awake; in system suspend the AOP sending a
 * report every 200 ms wakes CPU0 ~5 times per second, so stretch it to 1 h meanwhile. */
const ALS_PROP_INTERVAL: u32 = 0;
const ALS_INTERVAL_US: u32 = 200000;
const ALS_SUSPEND_INTERVAL_US: u32 = 3_600_000_000;

fn get_lux_offset(aop: &dyn AOP, dev: &platform::Device, svc: &EPICService) -> Result<usize> {
    let name = get_aop_property(aop, svc, 0xf, 16)?.1;
    match name.as_slice() {
        b"Redbird\0" => Ok(LUX_OFFSET_VD6286),
        b"FireFish2\0" => Ok(LUX_OFFSET_CT720),
        _ => {
            dev_warn!(
                dev.as_ref(),
                "Unknown sensor type {:?}",
                core::str::from_utf8(&name)
            );
            Err(EIO)
        }
    }
}

fn enable_als(aop: &dyn AOP, dev: &platform::Device, svc: &EPICService) -> Result<()> {
    let fw = Firmware::request(c_str!("apple/aop-als-cal.bin"), dev.as_ref())?;
    set_als_property(aop, svc, 0xb, fw.data())?;
    set_als_property(aop, svc, ALS_PROP_INTERVAL, &ALS_INTERVAL_US.to_le_bytes())?;

    Ok(())
}

fn get_aop_property(
    aop: &dyn AOP,
    svc: &EPICService,
    tag: u32,
    data_len: usize,
) -> Result<(u32, KVec<u8>)> {
    let mut buf = KVec::new();
    buf.resize(8, 0, GFP_KERNEL)?;
    buf[4..8].copy_from_slice(&tag.to_le_bytes());
    aop.epic_call_ret(svc, EPIC_SUBTYPE_GET_AOP_PROPERTY, &buf, data_len)
}

fn set_als_property(aop: &dyn AOP, svc: &EPICService, tag: u32, data: &[u8]) -> Result<u32> {
    let mut buf = KVec::new();
    buf.resize(data.len() + 8, 0, GFP_KERNEL)?;
    buf[8..].copy_from_slice(data);
    buf[4..8].copy_from_slice(&tag.to_le_bytes());
    aop.epic_call(svc, EPIC_SUBTYPE_SET_ALS_PROPERTY, &buf)
}

fn f32_to_u32(f: u32) -> u32 {
    if f & 0x80000000 != 0 {
        return 0;
    }
    let exp = ((f & 0x7f800000) >> 23) as i32 - 127;
    if exp < 0 {
        return 0;
    }
    if exp == 128 && f & 0x7fffff != 0 {
        return 0;
    }
    let mant = f & 0x7fffff | 0x800000;
    if exp <= 23 {
        return mant >> (23 - exp);
    }
    if exp >= 32 {
        return u32::MAX;
    }
    mant << (exp - 23)
}

struct MsgProc(usize);

impl MessageProcessor for MsgProc {
    fn process(&self, message: &[u8]) -> u32 {
        let offset = self.0;
        let raw = u32::from_le_bytes(message[offset..offset + 4].try_into().unwrap());
        f32_to_u32(raw)
    }
}

/// PM notifier (Rust platform drivers have no suspend/resume hooks here): slows the ALS
/// reports down before system suspend and restores them after.
struct AlsPm {
    nb: bindings::notifier_block,
    aop: Arc<dyn AOP>,
    svc: EPICService,
}

unsafe extern "C" fn als_pm_notify(
    nb: *mut bindings::notifier_block,
    action: c_ulong,
    _data: *mut c_void,
) -> c_int {
    // SAFETY: nb is the `nb` field of an AlsPm kept alive until unregister_pm_notifier()
    let pm = unsafe { &*kernel::container_of!(nb, AlsPm, nb) };
    let us = match action as u32 {
        bindings::PM_SUSPEND_PREPARE => ALS_SUSPEND_INTERVAL_US,
        bindings::PM_POST_SUSPEND => ALS_INTERVAL_US,
        _ => return 0, /* NOTIFY_DONE */
    };
    match set_als_property(pm.aop.as_ref(), &pm.svc, ALS_PROP_INTERVAL, &us.to_le_bytes()) {
        Ok(ret) => pr_info!("aop-als: report interval {} us (ret {:#x})\n", us, ret),
        Err(e) => pr_warn!("aop-als: setting report interval {} us failed: {:?}\n", us, e),
    }
    0 /* NOTIFY_DONE */
}

struct AlsPmHandle(*mut AlsPm);

// SAFETY: AlsPm is only touched by the PM notifier and by drop; Arc<dyn AOP> is Send + Sync.
unsafe impl Send for AlsPmHandle {}

impl AlsPmHandle {
    fn new(aop: Arc<dyn AOP>, svc: EPICService) -> Result<Self> {
        let pm = KBox::new(
            AlsPm {
                nb: bindings::notifier_block::default(),
                aop,
                svc,
            },
            GFP_KERNEL,
        )?;
        let p = KBox::into_raw(pm);
        // SAFETY: p is a valid, heap-allocated AlsPm that stays put until drop
        unsafe {
            (*p).nb.notifier_call = Some(als_pm_notify);
            let ret = bindings::register_pm_notifier(&raw mut (*p).nb);
            if ret != 0 {
                drop(KBox::from_raw(p));
                return Err(Error::from_errno(ret));
            }
        }
        Ok(Self(p))
    }
}

impl Drop for AlsPmHandle {
    fn drop(&mut self) {
        // SAFETY: self.0 came from KBox::into_raw in new() and was registered there
        unsafe {
            bindings::unregister_pm_notifier(&raw mut (*self.0).nb);
            let pm = KBox::from_raw(self.0);
            /* without this the AOP keeps calling into the unloaded module (oops in recv_message) */
            pm.aop.remove_fakehid_listener(&pm.svc);
        }
    }
}

struct IIOAopAlsDriver {
    /* dropped first: unregisters the notifier and the fakehid listener */
    _pm: AlsPmHandle,
    _reg: IIORegistration<MsgProc>,
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    (),
    [(of::DeviceId::new(c_str!("apple,aop-als")), ())]
);

impl platform::Driver for IIOAopAlsDriver {
    type IdInfo = ();
    type Data<'bound> = IIOAopAlsDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        _info: Option<&'bound ()>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let dev = pdev.as_ref();
        let parent = dev.parent().unwrap();
        // SAFETY: our parent is AOP, and AopDriver is repr(transparent) for Arc<dyn Aop>
        let adata_ptr = unsafe { Pin::<KBox<Arc<dyn AOP>>>::borrow(parent.get_drvdata()) };
        let adata = (&*adata_ptr).clone();
        // SAFETY: AOP sets the platform data correctly
        let service = unsafe { *((*dev.as_raw()).platform_data as *const EPICService) };
        let ty = bindings::BINDINGS_IIO_LIGHT;
        let offset = get_lux_offset(adata.as_ref(), pdev, &service)?;
        let data = AopSensorData::new(dev.into(), ty, MsgProc(offset))?;
        adata.add_fakehid_listener(service, data.clone())?;
        /* from here on a probe error drops pm, which removes the listener again */
        let pm = AlsPmHandle::new(adata.clone(), service)?;
        enable_als(adata.as_ref(), pdev, &service)?;
        let info_mask = 1 << bindings::BINDINGS_IIO_CHAN_INFO_PROCESSED;
        let reg = IIORegistration::<MsgProc>::new(
            data,
            c"aop-sensors-als",
            ty,
            info_mask,
            &THIS_MODULE,
        )?;
        Ok(IIOAopAlsDriver { _pm: pm, _reg: reg })
    }
}

module_platform_driver! {
    type: IIOAopAlsDriver,
    name: "iio_aop_als",
    description: "AOP ambient light sensor driver",
    license: "Dual MIT/GPL",
    alias: ["platform:iio_aop_als"],
    firmware: ["apple/aop-als-cal.bin"],
}
