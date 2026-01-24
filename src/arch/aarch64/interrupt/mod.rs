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
        static WFI_SHORT_COUNT: AtomicU64 = AtomicU64::new(0);

        // Read DAIF before enabling interrupts to verify state
        let daif_before: u64;
        asm!("mrs {}, daif", out(reg) daif_before, options(nomem, nostack));

        // Read ARM event register before WFI
        let event_before: u64;
        asm!("mrs {}, pmccntr_el0", out(reg) event_before, options(nomem, nostack));

        // Read cycle counter before WFI
        let cycles_before: u64;
        asm!("mrs {}, cntvct_el0", out(reg) cycles_before, options(nomem, nostack));

        // Enable interrupts FIRST, then halt - same order as x86 sti;hlt
        // DSB ensures memory ops complete, then we enable interrupts, then WFI
        // WFI with interrupts enabled will properly wait for next interrupt
        asm!("dsb sy", "msr daifclr, #2", "wfi", "nop");

        // Read cycle counter after WFI
        let cycles_after: u64;
        asm!("mrs {}, cntvct_el0", out(reg) cycles_after, options(nomem, nostack));

        // Read DAIF after to verify interrupts were enabled
        let daif_after: u64;
        asm!("mrs {}, daif", out(reg) daif_after, options(nomem, nostack));

        let cycles_elapsed = cycles_after.wrapping_sub(cycles_before);

        // Virtual timer runs at ~24MHz on most ARM systems
        // So 24000 cycles = ~1ms, expect at least that for a real halt
        // If WFI returns in <100 cycles (~4µs), it's essentially immediate
        if cycles_elapsed < 100 {
            let count = WFI_SHORT_COUNT.fetch_add(1, Ordering::Relaxed);
            if count % 10000 == 0 {
                warn!(
                    "WFI returned immediately: {} cycles, DAIF before={:#x} after={:#x}, event={:#x} (count={})",
                    cycles_elapsed, daif_before, daif_after, event_before, count
                );
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
