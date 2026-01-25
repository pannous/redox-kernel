//! Interrupt instructions

use core::arch::asm;

#[macro_use]
pub mod handler;

pub mod exception;
pub mod irq;
pub mod syscall;
pub mod trace;

pub use self::handler::InterruptStack;

/// Clear interrupts
#[inline(always)]
pub unsafe fn disable() {
    unsafe {
        asm!("msr daifset, #2");
    }
}

/// Set interrupts and halt
/// This will atomically wait for the next interrupt
/// Performing enable followed by halt is not guaranteed to be atomic, use this instead!
#[inline(always)]
pub unsafe fn enable_and_halt() {
    unsafe {
        use core::sync::atomic::{AtomicU64, Ordering};
        static WFI_MEASUREMENTS: AtomicU64 = AtomicU64::new(0);
        static WFI_SHORT_RETURNS: AtomicU64 = AtomicU64::new(0);
        static WFI_GOOD_HALTS: AtomicU64 = AtomicU64::new(0);
        static LOGGED_INIT: AtomicU64 = AtomicU64::new(0);

        if LOGGED_INIT.fetch_add(1, Ordering::Relaxed) == 0 {
            warn!("WFI measurement enabled - will log every 10,000 calls");
        }

        let cycles_before: u64;
        asm!("mrs {}, cntvct_el0", out(reg) cycles_before, options(nomem, nostack));

        // DSB ensures memory ops complete, then we enable interrupts, then WFI
        asm!("dsb sy", "msr daifclr, #2", "wfi", "nop");

        let cycles_after: u64;
        asm!("mrs {}, cntvct_el0", out(reg) cycles_after, options(nomem, nostack));

        let cycles_elapsed = cycles_after.wrapping_sub(cycles_before);

        // Virtual timer runs at ~24MHz on most ARM systems
        // 1000 cycles = ~40µs, 24000 cycles = ~1ms
        let total = WFI_MEASUREMENTS.fetch_add(1, Ordering::Relaxed);

        if cycles_elapsed < 1000 {
            // WFI returned very quickly (< 40µs)
            let short_count = WFI_SHORT_RETURNS.fetch_add(1, Ordering::Relaxed);
            if total % 10_000 == 0 {
                warn!("WFI quick return: {} cycles ({} short / {} total = {}%)",
                      cycles_elapsed, short_count, total, (short_count * 100) / total);
            }
        } else {
            // WFI actually halted for a reasonable time
            let good_count = WFI_GOOD_HALTS.fetch_add(1, Ordering::Relaxed);
            if total % 10_000 == 0 {
                warn!("WFI halted properly: {} cycles ({} good / {} total = {}%)",
                      cycles_elapsed, good_count, total, (good_count * 100) / total);
            }
        }
    }
}

/// Set interrupts and nop
/// This will enable interrupts and allow the IF flag to be processed
/// Simply enabling interrupts does not gurantee that they will trigger, use this instead!
#[inline(always)]
pub unsafe fn enable_and_nop() {
    unsafe {
        asm!("msr daifclr, #2", "nop");
    }
}

/// Halt instruction
#[inline(always)]
pub unsafe fn halt() {
    unsafe {
        // DSB ensures TLB/memory ops complete before WFI (required for HVF)
        asm!("dsb sy", "wfi");
    }
}
