use crate::time::NANOS_PER_SEC;

pub fn monotonic_absolute() -> u128 {
    //TODO: aarch64 generic timer counter
    let ticks: usize;
    unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) ticks) };
    let freq: usize;
    unsafe { core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq) };

    ticks as u128 * NANOS_PER_SEC / freq as u128
}

pub fn monotonic_resolution() -> u128 {
    let freq: usize;
    unsafe { core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq) };

    NANOS_PER_SEC / freq as u128
}

/// Busy-wait delay using the aarch64 cycle counter
/// Used to mitigate QEMU/HVF spurious WFI wakeups
pub fn delay_microseconds(micros: u64) {
    let freq: u64;
    unsafe { core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq) };

    // Calculate ticks needed for the delay
    let ticks_needed = (freq * micros) / 1_000_000;

    let start: u64;
    unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) start) };

    loop {
        let now: u64;
        unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) now) };

        if now.wrapping_sub(start) >= ticks_needed {
            break;
        }

        // Hint to CPU that we're spinning
        unsafe { core::arch::asm!("yield") };
    }
}
