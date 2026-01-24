/// SMP Diagnostics Module
///
/// This module provides diagnostics and logging for SMP (Symmetric Multiprocessing) activity.
/// It tracks CPU activity, context switches, and IPI events across all CPUs.

use core::sync::atomic::{AtomicU64, Ordering};

/// Global tick counter for periodic SMP reporting
#[allow(dead_code)]
static GLOBAL_TICK_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Log SMP activity summary periodically (called from timer interrupt)
#[allow(dead_code)]
pub fn periodic_log() {
    let tick = GLOBAL_TICK_COUNTER.fetch_add(1, Ordering::Relaxed);

    // Log every 500 ticks (approximately every 5 seconds at 100Hz)
    if tick % 500 == 0 && tick > 0 {
        let cpu_id = crate::cpu_id();
        info!(
            "SMP: === Activity Summary @ tick {} (CPU {}) ===",
            tick,
            cpu_id.get()
        );

        // Get statistics for all CPUs
        let all_stats = crate::percpu::get_all_stats();

        for (cpu_id, stats) in all_stats.iter() {
            info!(
                "SMP: CPU {} - switches: {}, IPIs: sent={} recv={}, time: user={}ms kern={}ms idle={}ms",
                cpu_id.get(),
                stats.context_switches,
                stats.ipis_sent,
                stats.ipis_received,
                stats.user / 1_000_000,
                stats.kernel / 1_000_000,
                stats.idle / 1_000_000,
            );
        }

        let total_switches = crate::cpu_stats::get_context_switch_count();
        let total_contexts = crate::cpu_stats::get_contexts_count();
        info!(
            "SMP: Global - total_contexts: {}, total_switches: {}",
            total_contexts,
            total_switches
        );
    }
}

/// Log when an IPI is sent
#[allow(dead_code)]
#[inline]
pub fn log_ipi_send(from_cpu: crate::cpu_set::LogicalCpuId, kind: &str, target: &str) {
    trace!("IPI: CPU {} send {} to {}", from_cpu.get(), kind, target);
    crate::percpu::PercpuBlock::current().stats.add_ipi_sent();
}

/// Log when an IPI is received
#[allow(dead_code)]
#[inline]
pub fn log_ipi_receive(cpu_id: crate::cpu_set::LogicalCpuId, kind: &str) {
    trace!("IPI: CPU {} received {}", cpu_id.get(), kind);
    crate::percpu::PercpuBlock::current().stats.add_ipi_received();
}
