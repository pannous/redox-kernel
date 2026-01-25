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
        static WFI_SKIPPED_COUNT: AtomicU64 = AtomicU64::new(0);

        // Check GIC for pending interrupts BEFORE enabling interrupts and WFI
        // This prevents the race condition where an interrupt arrives between
        // enabling interrupts and executing WFI, causing immediate return.
        //
        // ICC_HPPIR1_EL1 = Highest Priority Pending Interrupt Register
        // Returns 1023 if no interrupt is pending (read-only, no side effects)
        let pending_irq: u64;
        asm!("mrs {}, icc_hppir1_el1", out(reg) pending_irq, options(nomem, nostack));

        // Read cycle counter before for diagnostics
        let cycles_before: u64;
        asm!("mrs {}, cntvct_el0", out(reg) cycles_before, options(nomem, nostack));

        if pending_irq == 1023 {
            // No pending interrupt - safe to execute WFI
            // Enable interrupts FIRST, then halt - same order as x86 sti;hlt
            // DSB ensures memory ops complete, then we enable interrupts, then WFI
            // WFI with interrupts enabled will properly wait for next interrupt
            asm!("dsb sy", "msr daifclr, #2", "wfi", "nop");
        } else {
            // Interrupt already pending - skip WFI, just enable interrupts
            // The pending interrupt will be handled immediately after return
            asm!("msr daifclr, #2", "nop");

            // Track how often we skip WFI due to pending interrupts
            let skip_count = WFI_SKIPPED_COUNT.fetch_add(1, Ordering::Relaxed);
            if skip_count % 100000 == 0 {
                trace!(
                    "WFI skipped due to pending IRQ {}: count={}",
                    pending_irq,
                    skip_count
                );
            }
        }

        // Read cycle counter after for diagnostics
        let cycles_after: u64;
        asm!("mrs {}, cntvct_el0", out(reg) cycles_after, options(nomem, nostack));

        let cycles_elapsed = cycles_after.wrapping_sub(cycles_before);

        // Virtual timer runs at ~24MHz on most ARM systems
        // So 24000 cycles = ~1ms, expect at least that for a real halt
        // If WFI returns in <100 cycles (~4µs), it's essentially immediate
        // This should now be rare with the pending interrupt check
        if cycles_elapsed < 100 && pending_irq == 1023 {
            let count = WFI_SHORT_COUNT.fetch_add(1, Ordering::Relaxed);
            if count % 10000 == 0 {
                warn!(
                    "WFI still returned immediately despite no pending IRQ: {} cycles (count={})",
                    cycles_elapsed, count
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
