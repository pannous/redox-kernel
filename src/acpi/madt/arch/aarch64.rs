use alloc::{boxed::Box, vec::Vec};
use core::{hint, sync::atomic::Ordering, arch::asm};

use super::{Madt, MadtEntry};
use crate::{
    device::irqchip::{
        gic::{GenericInterruptController, GicCpuIf, GicDistIf},
        gicv3::{GicV3, GicV3CpuIf},
    },
    dtb::irqchip::{IrqChipItem, IRQ_CHIP},
    memory::{map_device_memory, PhysicalAddress, PAGE_SIZE, allocate_p2frame, KernelMapper},
};

/// Initialize multi-core support from device tree (when ACPI MADT is not available)
pub(super) fn init_from_dtb() {
    // Get CPU count from global
    let cpu_count = crate::CPU_COUNT.load(core::sync::atomic::Ordering::SeqCst);
    info!("SMP: Device tree indicates {} CPU(s)", cpu_count);

    if cpu_count <= 1 {
        info!("SMP: Single CPU system, no secondary CPUs to start");
        return;
    }

    // The GIC should already be initialized from device tree by dtb::init()
    // IRQ_CHIP should contain the GIC controller
    // Now we just need to start secondary CPUs using PSCI

    if cfg!(feature = "multi_core") {
        unsafe {
            start_secondary_cpus_from_dtb(cpu_count as usize);
        }
    } else {
        warn!("SMP: multi_core feature not enabled");
    }
}

pub(super) fn init(madt: Madt) {
    let mut gicd_opt = None;
    let mut giccs = Vec::new();
    for madt_entry in madt.iter() {
        trace!("{:#x?}", madt_entry);
        match madt_entry {
            MadtEntry::Gicc(gicc) => {
                giccs.push(gicc);
            }
            MadtEntry::Gicd(gicd) => {
                if gicd_opt.is_some() {
                    warn!("Only one GICD should be present on a system, ignoring this one");
                } else {
                    gicd_opt = Some(gicd);
                }
            }
            _ => {}
        }
    }

    // Update CPU count based on GICC entries
    let cpu_count = giccs.len() as u32;
    if cpu_count > 0 {
        info!("MADT: Found {} CPU(s)", cpu_count);
        crate::CPU_COUNT.store(cpu_count, core::sync::atomic::Ordering::SeqCst);
    }
    let Some(gicd) = gicd_opt else {
        warn!("No GICD found");
        return;
    };

    // Initialize the distributor interface (shared by all CPUs)
    let mut gic_dist_if = GicDistIf::default();
    unsafe {
        let phys = PhysicalAddress::new(gicd.physical_base_address as usize);
        let virt = map_device_memory(phys, PAGE_SIZE);
        gic_dist_if.init(virt.data());
    };
    debug!("{:#x?}", gic_dist_if);
    info!("GIC distributor initialized, version {}", gicd.gic_version);

    match gicd.gic_version {
        1 | 2 => {
            // GICv2: Initialize all CPU interfaces
            let mut cpu_idx = 0;
            for gicc in &giccs {
                debug!("Initializing GICv2 CPU interface {}", cpu_idx);

                let mut gic_cpu_if = GicCpuIf::default();
                unsafe {
                    let phys = PhysicalAddress::new(gicc.physical_base_address as usize);
                    let virt = map_device_memory(phys, PAGE_SIZE);
                    gic_cpu_if.init(virt.data())
                };
                trace!("GIC CPU {} interface: {:#x?}", cpu_idx, gic_cpu_if);

                // Create controller for this CPU (with cloned distributor)
                let gic = GenericInterruptController {
                    gic_dist_if,  // Copy of distributor interface
                    gic_cpu_if,
                    irq_range: (0, 0),
                };
                let chip = IrqChipItem {
                    phandle: 0,
                    parents: Vec::new(),
                    children: Vec::new(),
                    ic: Box::new(gic),
                };
                unsafe { IRQ_CHIP.irq_chip_list.chips.push(chip) };

                cpu_idx += 1;
            }
            info!("Initialized {} GICv2 CPU interfaces", cpu_idx);
        }
        3 => {
            // GICv3: Initialize all CPU interfaces
            let mut cpu_idx = 0;
            for _gicc in &giccs {
                debug!("Initializing GICv3 CPU interface {}", cpu_idx);

                let mut gic_cpu_if = GicV3CpuIf;
                unsafe { gic_cpu_if.init() };
                trace!("GIC CPU {} interface: {:#x?}", cpu_idx, gic_cpu_if);

                // Create controller for this CPU (with cloned distributor)
                let gic = GicV3 {
                    gic_dist_if,  // Copy of distributor interface
                    gic_cpu_if,
                    //TODO: get GICRs from MADT
                    gicrs: Vec::new(),
                    irq_range: (0, 0),
                };
                let chip = IrqChipItem {
                    phandle: 0,
                    parents: Vec::new(),
                    children: Vec::new(),
                    ic: Box::new(gic),
                };
                unsafe { IRQ_CHIP.irq_chip_list.chips.push(chip) };

                cpu_idx += 1;
            }
            info!("Initialized {} GICv3 CPU interfaces", cpu_idx);
        }
        _ => {
            warn!("Unsupported GIC version {}", gicd.gic_version);
        }
    }
    unsafe { IRQ_CHIP.init(None) };

    // Phase 4: Start secondary CPUs using PSCI
    if cfg!(feature = "multi_core") {
        unsafe {
            start_secondary_cpus(&giccs);
        }
    }
}

/// PSCI function IDs (ARM PSCI Specification v1.0+)
const PSCI_CPU_ON_64: u64 = 0xC4000003;

/// Make a PSCI call using HVC instruction
///
/// PSCI is the Power State Coordination Interface used on ARM to manage CPU power states.
/// We use HVC (Hypervisor Call) which traps to EL2 (hypervisor level).
/// QEMU's virt machine implements PSCI at EL2.
#[inline]
unsafe fn psci_call(function_id: u64, arg0: u64, arg1: u64, arg2: u64) -> i64 {
    let result: i64;
    unsafe {
        asm!(
            "hvc #0",
            inout("x0") function_id => result,
            in("x1") arg0,
            in("x2") arg1,
            in("x3") arg2,
            options(nomem, nostack)
        );
    }
    result
}

/// Start secondary CPUs using PSCI CPU_ON
unsafe fn start_secondary_cpus(giccs: &[&super::MadtGicc]) {
    // Get current CPU's MPIDR to identify BSP
    let mut bsp_mpidr: u64;
    unsafe {
        asm!("mrs {}, mpidr_el1", out(reg) bsp_mpidr, options(nomem, nostack));
    }
    // Mask off non-affinity bits (keep only Aff0, Aff1, Aff2, Aff3)
    bsp_mpidr &= 0xFF00FFFFFF;

    // Get page table physical address
    let page_table_phys = {
        let mapper = KernelMapper::lock();
        mapper.table().phys().data() as u64
    };

    debug!("BSP MPIDR=0x{:x}, page_table=0x{:x}", bsp_mpidr, page_table_phys);

    let mut ap_count = 0;
    for gicc in giccs.iter() {
        let mpidr = gicc.mpidr & 0xFF00FFFFFF;  // Mask to match BSP

        // Skip BSP
        if mpidr == bsp_mpidr {
            debug!("GICC MPIDR=0x{:x} is BSP, skipping", mpidr);
            continue;
        }

        // Skip if CPU is not enabled
        if gicc.flags & 1 == 0 {
            debug!("GICC MPIDR=0x{:x} is disabled, skipping", mpidr);
            continue;
        }

        debug!("Starting AP {} with MPIDR=0x{:x}", ap_count, mpidr);

        // Allocate per-CPU stack (4 * 2MB = 8MB per CPU)
        let stack_frame = allocate_p2frame(4)
            .expect("Failed to allocate stack for AP");
        let stack_start = stack_frame.base().data() + crate::PHYS_OFFSET;
        let stack_end = stack_start + (PAGE_SIZE << 4);  // 128KB stack

        trace!("AP {} stack: 0x{:x} - 0x{:x}", ap_count, stack_start, stack_end);

        // Create KernelArgsAp structure
        let args = crate::arch::start::KernelArgsAp {
            cpu_id: ap_count + 1,  // CPUs numbered 1, 2, 3...
            page_table: page_table_phys,
            stack_start: stack_start as u64,
            stack_end: stack_end as u64,
        };

        // Get entry point address
        let entry_point = crate::arch::start::kstart_ap as *const () as u64;
        let args_ptr = &args as *const _ as u64;

        trace!("PSCI CPU_ON: mpidr=0x{:x}, entry=0x{:x}, context=0x{:x}",
              mpidr, entry_point, args_ptr);

        // Reset AP_READY flag
        crate::arch::start::AP_READY.store(false, Ordering::SeqCst);

        // Call PSCI CPU_ON
        let result = unsafe { psci_call(PSCI_CPU_ON_64, mpidr, entry_point, args_ptr) };

        if result == 0 {
            debug!("PSCI CPU_ON succeeded for AP {}", ap_count);

            // Wait for AP to signal ready (with timeout)
            let mut timeout = 10_000_000;  // ~10 seconds
            while !crate::arch::start::AP_READY.load(Ordering::SeqCst) && timeout > 0 {
                hint::spin_loop();
                timeout -= 1;
            }

            if timeout == 0 {
                warn!("Timeout waiting for AP {} to become ready", ap_count);
            } else {
                debug!("AP {} is ready", ap_count);
            }
        } else {
            warn!("PSCI CPU_ON failed for AP {} with error code {}", ap_count, result);
        }

        ap_count += 1;
    }

    info!("Started {} secondary CPU(s)", ap_count);
}

/// Start secondary CPUs from device tree (without ACPI MADT)
/// Uses standard MPIDR values (0, 1, 2, 3...) for QEMU virt machine
unsafe fn start_secondary_cpus_from_dtb(total_cpus: usize) {
    if total_cpus <= 1 {
        return;
    }

    info!("SMP: Starting {} secondary CPUs from DTB", total_cpus - 1);

    // Get current CPU's MPIDR to identify BSP
    let mut bsp_mpidr: u64;
    unsafe {
        asm!("mrs {}, mpidr_el1", out(reg) bsp_mpidr, options(nomem, nostack));
    }
    // Mask off non-affinity bits (keep only Aff0, Aff1, Aff2, Aff3)
    bsp_mpidr &= 0xFF00FFFFFF;

    // Get page table physical address
    let page_table_phys = {
        let mapper = KernelMapper::lock();
        mapper.table().phys().data() as u64
    };

    debug!("BSP MPIDR=0x{:x}, page_table=0x{:x}", bsp_mpidr, page_table_phys);

    let mut ap_count = 0;

    // Try to start CPUs with MPIDR values 0, 1, 2, 3...
    // Skip the BSP which is usually MPIDR=0
    for cpu_id in 0..total_cpus {
        let mpidr = cpu_id as u64;

        // Skip BSP
        if mpidr == bsp_mpidr {
            debug!("MPIDR=0x{:x} is BSP, skipping", mpidr);
            continue;
        }

        debug!("Starting AP {} with MPIDR=0x{:x}", ap_count, mpidr);

        // Allocate per-CPU stack (4 * 2MB = 8MB per CPU)
        let stack_frame = allocate_p2frame(4)
            .expect("Failed to allocate stack for AP");
        let stack_start = stack_frame.base().data() + crate::PHYS_OFFSET;
        let stack_end = stack_start + (PAGE_SIZE << 4);  // 128KB stack

        trace!("AP {} stack: 0x{:x} - 0x{:x}", ap_count, stack_start, stack_end);

        // Create KernelArgsAp structure
        let args = crate::arch::start::KernelArgsAp {
            cpu_id: cpu_id as u64,  // Use actual CPU ID
            page_table: page_table_phys,
            stack_start: stack_start as u64,
            stack_end: stack_end as u64,
        };

        // Get entry point address
        let entry_point = crate::arch::start::kstart_ap as *const () as u64;
        let args_ptr = &args as *const _ as u64;

        trace!("PSCI CPU_ON: mpidr=0x{:x}, entry=0x{:x}, context=0x{:x}",
              mpidr, entry_point, args_ptr);

        // Reset AP_READY flag
        crate::arch::start::AP_READY.store(false, Ordering::SeqCst);

        // Call PSCI CPU_ON
        let result = unsafe { psci_call(PSCI_CPU_ON_64, mpidr, entry_point, args_ptr) };

        if result == 0 {
            debug!("PSCI CPU_ON succeeded for AP {}", ap_count);

            // Wait for AP to signal ready (with timeout)
            let mut timeout = 10_000_000;  // ~10 seconds
            while !crate::arch::start::AP_READY.load(Ordering::SeqCst) && timeout > 0 {
                hint::spin_loop();
                timeout -= 1;
            }

            if timeout == 0 {
                warn!("Timeout waiting for AP {} to become ready", ap_count);
            } else {
                debug!("AP {} is ready", ap_count);
            }

            ap_count += 1;
        } else {
            debug!("PSCI CPU_ON failed for AP {} (MPIDR=0x{:x}) with error code {} (may be disabled)",
                   ap_count, mpidr, result);
        }
    }

    info!("Started {} secondary CPU(s) from DTB", ap_count);
}
