use crate::{
    arch::device::ROOT_IC_IDX, dtb::irqchip::IRQ_CHIP, scheme::irq::irq_trigger,
    sync::CleanLockToken,
};
use core::sync::atomic::Ordering;

// use crate::percpu::PercpuBlock;

unsafe fn irq_ack() -> (u32, Option<usize>) {
    unsafe {
        let ic = &mut IRQ_CHIP.irq_chip_list.chips[ROOT_IC_IDX.load(Ordering::Relaxed)].ic;
        let irq = ic.irq_ack();
        (irq, ic.irq_to_virq(irq))
    }
}

exception_stack!(irq_at_el0, |_stack| {
    unsafe {
        let mut token = CleanLockToken::new();
        let (irq, virq) = irq_ack();

        // Check if this is an SGI (Software Generated Interrupt) used for IPIs
        // SGIs use interrupt IDs 0-15 in the GIC
        if irq < 16 {
            // Call IPI handler for SGIs
            crate::ipi::handle_ipi(irq);
        } else if let Some(virq) = virq
            && virq < 1024
        {
            IRQ_CHIP.trigger_virq(virq as u32, &mut token);
        } else {
            println!("unexpected irq num {}", irq);
        }
    }
});

exception_stack!(irq_at_el1, |_stack| {
    unsafe {
        use core::sync::atomic::{AtomicU64, Ordering};
        static IRQ_COUNTER: [AtomicU64; 256] = [const { AtomicU64::new(0) }; 256];
        static IRQ_LOG_INTERVAL: AtomicU64 = AtomicU64::new(0);

        let mut token = CleanLockToken::new();
        let (irq, virq) = irq_ack();

        // Track interrupt counts per IRQ ID
        if (irq as usize) < 256 {
            IRQ_COUNTER[irq as usize].fetch_add(1, Ordering::Relaxed);
        }

        // Log interrupt distribution every 100,000 interrupts
        let total = IRQ_LOG_INTERVAL.fetch_add(1, Ordering::Relaxed);
        if total % 100_000 == 0 {
            // Find top 5 interrupts
            let mut top_irqs: [(u32, u64); 5] = [(0, 0); 5];
            for i in 0..256 {
                let count = IRQ_COUNTER[i].load(Ordering::Relaxed);
                if count > top_irqs[4].1 {
                    top_irqs[4] = (i as u32, count);
                    top_irqs.sort_by(|a, b| b.1.cmp(&a.1));
                }
            }
            warn!("IRQ distribution (top 5): {:?}", top_irqs);
        }

        // Check if this is an SGI (Software Generated Interrupt) used for IPIs
        // SGIs use interrupt IDs 0-15 in the GIC
        if irq < 16 {
            // Call IPI handler for SGIs
            crate::ipi::handle_ipi(irq);
        } else if let Some(virq) = virq
            && virq < 1024
        {
            IRQ_CHIP.trigger_virq(virq as u32, &mut token);
        } else {
            println!("unexpected irq num {}", irq);
        }
    }
});

//TODO
pub unsafe fn trigger(irq: u32, token: &mut CleanLockToken) {
    unsafe {
        // FIXME add_irq accepts a u8 as irq number
        // PercpuBlock::current().stats.add_irq(irq);

        irq_trigger(irq.try_into().unwrap(), token);
        IRQ_CHIP.irq_eoi(irq);
    }
}

/*
pub unsafe fn irq_handler_gentimer(irq: u32) {
    GENTIMER.clear_irq();
    {
        *time::OFFSET.lock() += GENTIMER.clk_freq as u128;
    }

    timeout::trigger();

    context::switch::tick();

    trigger(irq);
    GENTIMER.reload_count();
}
*/
