use crate::{dtb::get_mmio_address, time};
use core::ptr::read_volatile;

static RTC_DR: usize = 0x000;
// QEMU virt machine PL031 address
const QEMU_VIRT_PL031_ADDR: usize = 0x9010000;

/// Initialize RTC from device tree
pub unsafe fn init(fdt: &fdt::Fdt) {
    if let Some(node) = fdt.find_compatible(&["arm,pl031"]) {
        match node.reg()
            .and_then(|mut iter| iter.next())
            .and_then(|region| get_mmio_address(fdt, &node, &region))
        {
            Some(phys) => {
                let mut rtc = Pl031rtc { phys };
                let seconds = rtc.time();
                info!("PL031 RTC at {:#x}, time={} seconds", rtc.phys, seconds);
                *time::START.lock() = (seconds as u128) * time::NANOS_PER_SEC;
            }
            None => {
                warn!("No PL031 RTC registers");
            }
        }
    } else {
        warn!("No PL031 RTC found");
    }
}

/// Initialize RTC using QEMU virt machine's known PL031 address
/// Call this as fallback when DTB is not available (ACPI boot)
pub unsafe fn init_qemu_virt() {
    let mut rtc = Pl031rtc { phys: QEMU_VIRT_PL031_ADDR };
    let seconds = rtc.time();
    if seconds > 0 {
        info!("PL031 RTC (QEMU virt) at {:#x}, time={} seconds", QEMU_VIRT_PL031_ADDR, seconds);
        *time::START.lock() = (seconds as u128) * time::NANOS_PER_SEC;
    } else {
        warn!("PL031 RTC read returned 0, time not set");
    }
}

struct Pl031rtc {
    pub phys: usize,
}

impl Pl031rtc {
    unsafe fn read(&self, reg: usize) -> u32 {
        unsafe { read_volatile((crate::PHYS_OFFSET + self.phys + reg) as *const u32) }
    }

    pub fn time(&mut self) -> u64 {
        let seconds = unsafe { self.read(RTC_DR) } as u64;
        seconds
    }
}
