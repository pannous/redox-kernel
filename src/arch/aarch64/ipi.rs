#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum IpiKind {
    Wakeup = 0,
    Tlb = 1,
    Switch = 2,
    Pit = 3,
    Kstop = 4,
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum IpiTarget {
    Current = 1,
    Other = 2,
    All = 3,
}

#[inline(always)]
pub fn ipi(kind: IpiKind, target: IpiTarget) {
    if cfg!(not(feature = "multi_core")) {
        return;
    }

    use crate::arch::device::ROOT_IC_IDX;
    use crate::dtb::irqchip::IRQ_CHIP;
    use core::sync::atomic::Ordering;

    trace!(
        "IPI: CPU {} send {:?} to {:?}",
        crate::cpu_id().get(),
        kind,
        target
    );

    // Increment IPI send counter
    crate::percpu::PercpuBlock::current().stats.add_ipi_sent();

    unsafe {
        let ic_idx = ROOT_IC_IDX.load(Ordering::Relaxed);
        let ic = &mut IRQ_CHIP.irq_chip_list.chips[ic_idx].ic;
        ic.send_sgi(kind, target);
    }
}

#[inline(always)]
pub fn ipi_single(_kind: IpiKind, _target: &crate::percpu::PercpuBlock) {
    if cfg!(not(feature = "multi_core")) {
        return;
    }

    debug!(
        "IPI: CPU {} send {:?} to CPU {}",
        crate::cpu_id().get(),
        _kind,
        _target.cpu_id.get()
    );
    // FIXME implement
}

/// Handle received IPI (Software Generated Interrupt)
///
/// Called from interrupt handler when an SGI is received.
/// Processes the IPI based on its kind and sends EOI to GIC.
pub fn handle_ipi(sgi_num: u32) {
    use crate::arch::device::ROOT_IC_IDX;
    use crate::context;
    use crate::dtb::irqchip::IRQ_CHIP;
    use crate::percpu::PercpuBlock;
    use crate::sync::CleanLockToken;
    use core::sync::atomic::Ordering;

    // Map SGI number to IpiKind
    let kind = match sgi_num {
        0 => IpiKind::Wakeup,
        1 => IpiKind::Tlb,
        2 => IpiKind::Switch,
        3 => IpiKind::Pit,
        4 => IpiKind::Kstop,
        _ => {
            warn!("IPI: CPU {} received unknown SGI {}", crate::cpu_id().get(), sgi_num);
            unsafe {
                let ic_idx = ROOT_IC_IDX.load(Ordering::Relaxed);
                let ic = &mut IRQ_CHIP.irq_chip_list.chips[ic_idx].ic;
                ic.irq_eoi(sgi_num);
            }
            return;
        }
    };

    trace!(
        "IPI: CPU {} handling {:?} (SGI {})",
        crate::cpu_id().get(),
        kind,
        sgi_num
    );

    // Increment IPI receive counter
    PercpuBlock::current().stats.add_ipi_received();

    // Process the IPI based on its kind
    match kind {
        IpiKind::Wakeup => {
            // Simple wakeup - just acknowledge
            debug!("IPI: CPU {} wakeup", crate::cpu_id().get());
        }

        IpiKind::Tlb => {
            // TLB shootdown - invalidate TLB entries
            debug!("IPI: CPU {} TLB shootdown", crate::cpu_id().get());
            PercpuBlock::current().maybe_handle_tlb_shootdown();
        }

        IpiKind::Switch => {
            // Force context switch
            debug!("IPI: CPU {} forced switch", crate::cpu_id().get());
            let mut token = unsafe { CleanLockToken::new() };
            let _ = context::switch(&mut token);
        }

        IpiKind::Pit => {
            // Timer tick - trigger scheduler tick
            debug!("IPI: CPU {} timer tick", crate::cpu_id().get());
            let mut token = unsafe { CleanLockToken::new() };
            context::switch::tick(&mut token);
        }

        IpiKind::Kstop => {
            // Stop kernel on this CPU
            warn!("IPI: CPU {} kstop (halt)", crate::cpu_id().get());
            loop {
                // Halt CPU - wait for interrupt indefinitely
                unsafe {
                    core::arch::asm!("wfi");
                }
            }
        }
    }

    // Send EOI to GIC
    unsafe {
        let ic_idx = ROOT_IC_IDX.load(Ordering::Relaxed);
        let ic = &mut IRQ_CHIP.irq_chip_list.chips[ic_idx].ic;
        ic.irq_eoi(sgi_num);
    }
}
