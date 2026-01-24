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

pub(super) fn init(madt: Madt) {
    warn!("MADT: arch::init ENTERED - Starting Phase 2 GIC multi-CPU initialization");
    let mut gicd_opt = None;
    let mut giccs = Vec::new();
    for madt_entry in madt.iter() {
        debug!("      {:#x?}", madt_entry);
        match madt_entry {
            MadtEntry::Gicc(gicc) => {
                warn!("SMP: Found GICC entry (CPU)");
                giccs.push(gicc);
            }
            MadtEntry::Gicd(gicd) => {
                if gicd_opt.is_some() {
                    warn!("Only one GICD should be present on a system, ignoring this one");
                } else {
                    warn!("SMP: Found GICD entry");
                    gicd_opt = Some(gicd);
                }
            }
            _ => {}
        }
    }

    // Update CPU count based on GICC entries
    let cpu_count = giccs.len() as u32;
    if cpu_count > 0 {
        warn!("SMP: ACPI MADT lists {} CPU(s)", cpu_count);
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
    warn!("{:#x?}", gic_dist_if);
    warn!("SMP: GIC distributor initialized, version {}", gicd.gic_version);

    match gicd.gic_version {
        1 | 2 => {
            // GICv2: Initialize all CPU interfaces
            let mut cpu_idx = 0;
            for gicc in &giccs {
                warn!("SMP: Initializing GICv2 CPU interface {}", cpu_idx);

                let mut gic_cpu_if = GicCpuIf::default();
                unsafe {
                    let phys = PhysicalAddress::new(gicc.physical_base_address as usize);
                    let virt = map_device_memory(phys, PAGE_SIZE);
                    gic_cpu_if.init(virt.data())
                };
                warn!("SMP: GIC CPU {} interface: {:#x?}", cpu_idx, gic_cpu_if);

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
            warn!("SMP: Initialized {} GICv2 CPU interfaces", cpu_idx);
        }
        3 => {
            // GICv3: Initialize all CPU interfaces
            let mut cpu_idx = 0;
            for _gicc in &giccs {
                warn!("SMP: Initializing GICv3 CPU interface {}", cpu_idx);

                let mut gic_cpu_if = GicV3CpuIf;
                unsafe { gic_cpu_if.init() };
                warn!("SMP: GIC CPU {} interface: {:#x?}", cpu_idx, gic_cpu_if);

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
            warn!("SMP: Initialized {} GICv3 CPU interfaces", cpu_idx);
        }
        _ => {
            warn!("unsupported GIC version {}", gicd.gic_version);
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
    warn!("SMP: Starting secondary CPUs using PSCI");

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

    warn!("SMP: BSP MPIDR=0x{:x}, page_table=0x{:x}", bsp_mpidr, page_table_phys);

    let mut ap_count = 0;
    for gicc in giccs.iter() {
        let mpidr = gicc.mpidr & 0xFF00FFFFFF;  // Mask to match BSP

        // Skip BSP
        if mpidr == bsp_mpidr {
            warn!("SMP: GICC MPIDR=0x{:x} is BSP, skipping", mpidr);
            continue;
        }

        // Skip if CPU is not enabled
        if gicc.flags & 1 == 0 {
            warn!("SMP: GICC MPIDR=0x{:x} is disabled, skipping", mpidr);
            continue;
        }

        warn!("SMP: Starting AP {} with MPIDR=0x{:x}", ap_count, mpidr);

        // Allocate per-CPU stack (4 * 2MB = 8MB per CPU)
        let stack_frame = allocate_p2frame(4)
            .expect("Failed to allocate stack for AP");
        let stack_start = stack_frame.base().data() + crate::PHYS_OFFSET;
        let stack_end = stack_start + (PAGE_SIZE << 4);  // 128KB stack

        warn!("SMP: AP {} stack: 0x{:x} - 0x{:x}", ap_count, stack_start, stack_end);

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

        warn!("SMP: PSCI CPU_ON: mpidr=0x{:x}, entry=0x{:x}, context=0x{:x}",
              mpidr, entry_point, args_ptr);

        // Reset AP_READY flag
        crate::arch::start::AP_READY.store(false, Ordering::SeqCst);

        // Call PSCI CPU_ON
        let result = unsafe { psci_call(PSCI_CPU_ON_64, mpidr, entry_point, args_ptr) };

        if result == 0 {
            warn!("SMP: PSCI CPU_ON succeeded for AP {}", ap_count);

            // Wait for AP to signal ready (with timeout)
            let mut timeout = 10_000_000;  // ~10 seconds
            while !crate::arch::start::AP_READY.load(Ordering::SeqCst) && timeout > 0 {
                hint::spin_loop();
                timeout -= 1;
            }

            if timeout == 0 {
                warn!("SMP: Timeout waiting for AP {} to become ready", ap_count);
            } else {
                warn!("SMP: AP {} is ready!", ap_count);
            }
        } else {
            warn!("SMP: PSCI CPU_ON failed for AP {} with error code {}", ap_count, result);
        }

        ap_count += 1;
    }

    warn!("SMP: Started {} secondary CPU(s)", ap_count);
}
