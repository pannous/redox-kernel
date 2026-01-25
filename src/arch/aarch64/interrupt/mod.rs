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
        static WFI_RETRY_COUNT: AtomicU64 = AtomicU64::new(0);

        // Retry WFI if it returns immediately, with maximum of 3 attempts
        // This handles the race condition where an interrupt arrives between
        // enabling interrupts and executing WFI.
        for attempt in 0..3 {
            let cycles_before: u64;
            asm!("mrs {}, cntvct_el0", out(reg) cycles_before, options(nomem, nostack));

            // Enable interrupts and execute WFI
            // DSB ensures memory ops complete, then we enable interrupts, then WFI
            asm!("dsb sy", "msr daifclr, #2", "wfi", "nop");

            let cycles_after: u64;
            asm!("mrs {}, cntvct_el0", out(reg) cycles_after, options(nomem, nostack));

            let cycles_elapsed = cycles_after.wrapping_sub(cycles_before);

            // Virtual timer runs at ~24MHz on most ARM systems
            // 24000 cycles = ~1ms, we expect at least that for a real halt
            // If WFI slept for >1000 cycles (~40µs), consider it successful
            if cycles_elapsed >= 1000 {
                // WFI actually slept - success!
                return;
            }

            // WFI returned too quickly - likely an interrupt was already pending
            // Track short WFI returns
            let short_count = WFI_SHORT_COUNT.fetch_add(1, Ordering::Relaxed);
            if short_count % 100000 == 0 {
                trace!(
                    "WFI returned quickly: {} cycles, attempt {} (count={})",
                    cycles_elapsed,
                    attempt,
                    short_count
                );
            }

            // If this was the last attempt, give up and return
            if attempt == 2 {
                let retry_count = WFI_RETRY_COUNT.fetch_add(1, Ordering::Relaxed);
                if retry_count % 100000 == 0 {
                    debug!(
                        "WFI retry limit reached: {} cycles on final attempt (count={})",
                        cycles_elapsed, retry_count
                    );
                }
                return;
            }

            // Otherwise, retry WFI (loop continues)
            // Small spin to allow pending interrupt to be handled
            for _ in 0..10 {
                core::hint::spin_loop();
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
