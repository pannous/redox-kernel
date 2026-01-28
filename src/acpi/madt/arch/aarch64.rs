use alloc::{boxed::Box, vec::Vec};
use core::{hint, sync::atomic::Ordering, arch::asm};

use super::{Madt, MadtEntry};
use crate::{
    device::irqchip::{
        gic::{GenericInterruptController, GicCpuIf, GicDistIf},
        gicv3::{GicV3, GicV3CpuIf},
    },
    dtb::{irqchip::{IrqChipItem, IRQ_CHIP}, get_mmio_address},
    memory::{map_device_memory, PhysicalAddress, PAGE_SIZE, allocate_p2frame, KernelMapper},
};

/// Detect GIC version by reading hardware registers
/// Returns the GIC version (2 or 3) if detectable, None otherwise
unsafe fn detect_gic_version_from_hardware(gic_dist: &GicDistIf) -> Option<u8> {
    use core::ptr::read_volatile;

    // GICD_PIDR2 register at offset 0xFE8 contains architecture version
    // Bits [7:4] indicate the GIC architecture version
    const GICD_PIDR2: u32 = 0xFE8;

    let pidr2_addr = (gic_dist.address + GICD_PIDR2 as usize) as *const u32;
    let pidr2 = unsafe { read_volatile(pidr2_addr) };

    // Extract architecture version from bits [7:4]
    let arch_rev = ((pidr2 >> 4) & 0xF) as u8;

    // GICv2 = 0x1 or 0x2, GICv3/v4 = 0x3
    match arch_rev {
        0x1 | 0x2 => {
            info!("Hardware GIC PIDR2=0x{:x}, arch_rev={} (GICv2)", pidr2, arch_rev);
            Some(2)
        }
        0x3 => {
            info!("Hardware GIC PIDR2=0x{:x}, arch_rev={} (GICv3)", pidr2, arch_rev);
            Some(3)
        }
        _ => {
            warn!("Unknown GIC arch_rev={} from PIDR2=0x{:x}", arch_rev, pidr2);
            None
        }
    }
}

/// Detect GIC version from device tree
/// Returns the GIC version (2 or 3) if found, None otherwise
unsafe fn detect_gic_version_from_fdt() -> Option<u8> {
    info!("Detecting GIC version from FDT");
    let fdt = crate::dtb::fdt();
    if fdt.is_none() {
        warn!("FDT not available for GIC version detection");
        return None;
    }
    let fdt = fdt.unwrap();

    // Check for GICv3
    if fdt.find_compatible(&["arm,gic-v3"]).is_some() {
        info!("FDT indicates GICv3");
        return Some(3);
    }

    // Check for GICv2
    if fdt.find_compatible(&["arm,cortex-a15-gic", "arm,gic-400"]).is_some() {
        info!("FDT indicates GICv2");
        return Some(2);
    }

    warn!("No GIC found in FDT");
    None
}

/// Extract GICv3 redistributor addresses from device tree
/// Returns a Vec of (physical_address, size) tuples
unsafe fn get_gicv3_redistributors_from_fdt() -> Vec<(usize, usize)> {
    use fdt::node::NodeProperty;

    warn!("get_gicv3_redistributors_from_fdt() called");
    let mut gicrs = Vec::new();

    // Get the FDT from dtb module
    let fdt_opt = crate::dtb::fdt();
    let Some(fdt) = fdt_opt else {
        warn!("No FDT available for redistributor lookup");
        return gicrs;
    };

    // Look for GICv3 node
    let Some(node) = fdt.find_compatible(&["arm,gic-v3"]) else {
        warn!("No GICv3 node found in device tree");
        return gicrs;
    };

    // Get number of redistributor regions
    let gicr_count = node
        .property("#redistributor-regions")
        .and_then(NodeProperty::as_usize)
        .unwrap_or(1);

    info!("Device tree indicates {} redistributor region(s)", gicr_count);

    // Parse reg property - first entry is GICD, rest are GICRs
    let mut chunks = node.reg().unwrap();

    // Skip first entry (GICD)
    let _ = chunks.next();

    // Read redistributor entries
    for _ in 0..gicr_count {
        if let Some(gicr) = chunks.next() {
            if let Some(addr) = get_mmio_address(&fdt, &node, &gicr) {
                let size = gicr.size.unwrap_or(0x20000);  // Default GICR size
                gicrs.push((addr, size));
                debug!("Found redistributor at phys 0x{:x}, size 0x{:x}", addr, size);
            }
        }
    }

    gicrs
}

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
    info!("GIC distributor initialized, ACPI reports version {}", gicd.gic_version);

    // WORKAROUND: QEMU's ACPI tables may incorrectly report GICv2 even when using GICv3
    // Query the GIC hardware directly to get the real version
    let hw_version = unsafe { detect_gic_version_from_hardware(&gic_dist_if) };
    let actual_gic_version = hw_version.unwrap_or(gicd.gic_version);

    if actual_gic_version != gicd.gic_version {
        warn!("ACPI reports GICv{} but hardware is GICv{} - using hardware version",
              gicd.gic_version, actual_gic_version);
    }

    match actual_gic_version {
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
            info!("GICv3 detected - looking up redistributors from FDT");
            // GICv3: Get redistributor addresses from device tree
            // Even when using ACPI, the FDT contains GIC topology info
            let gicrs = unsafe {
                get_gicv3_redistributors_from_fdt()
            };

            if gicrs.is_empty() {
                warn!("No GICv3 redistributors found - PPIs will not work!");
            } else {
                info!("Found {} GICv3 redistributor(s)", gicrs.len());
            }

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
                    gicrs: gicrs.clone(),  // Share redistributor addresses
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

        // Allocate KernelArgsAp in physical frame (not stack) for AP to access before MMU setup
        let args_frame = allocate_p2frame(1)
            .expect("Failed to allocate args frame for AP");
        let args_phys = args_frame.base().data() as u64;
        let args_virt = (args_phys as usize + crate::PHYS_OFFSET) as *mut crate::arch::start::KernelArgsAp;

        // Get kernel physical base from memory map (needed for args and entry point)
        let kernel_phys_base = crate::startup::memory::kernel_phys_base() as u64;

        // Get the separate identity mapping page table we created
        let idmap_phys = crate::startup::memory::idmap_pg_dir().data() as u64;

        // Read BSP's TCR_EL1 and MAIR_EL1 values to pass to APs
        let tcr_el1: u64;
        let mair_el1: u64;
        unsafe {
            core::arch::asm!(
                "mrs {}, tcr_el1",
                "mrs {}, mair_el1",
                out(reg) tcr_el1,
                out(reg) mair_el1,
            );
        }
        debug!("BSP TCR_EL1=0x{:x}, MAIR_EL1=0x{:x}", tcr_el1, mair_el1);

        // Write args to allocated frame
        unsafe {
            args_virt.write(crate::arch::start::KernelArgsAp {
                cpu_id: ap_count + 1,  // CPUs numbered 1, 2, 3...
                page_table: page_table_phys,
                stack_start: stack_start as u64,
                stack_end: stack_end as u64,
                kernel_phys_base,
                idmap_pg_dir: idmap_phys,  // Use kernel PT (already identity-mapped)
                tcr_el1,   // BSP's TCR value
                mair_el1,  // BSP's MAIR value
            });
        }

        // Get entry point address (PHYSICAL not virtual!)
        let entry_point_virt = crate::arch::start::kstart_ap as *const () as u64;

        // On aarch64, bootloader loads kernel at some physical address
        // and maps it to virtual address at KERNEL_OFFSET (0xFFFF_FF00_0000_0000).
        //
        // We need to convert the current virtual address back to physical.
        use crate::arch::consts::KERNEL_OFFSET;

        // Calculate offset within kernel and add to physical base
        let offset = entry_point_virt - KERNEL_OFFSET as u64;
        let entry_point_phys = kernel_phys_base + offset;

        info!("PSCI CPU_ON: mpidr=0x{:x}, entry_phys=0x{:x}, kernel_pt=0x{:x}, idmap_pt=0x{:x}",
              mpidr, entry_point_phys, page_table_phys, idmap_phys);

        // Reset AP_READY flag
        crate::arch::start::AP_READY.store(false, Ordering::SeqCst);

        // Call PSCI CPU_ON - pass PHYSICAL addresses
        let result = unsafe { psci_call(PSCI_CPU_ON_64, mpidr, entry_point_phys, args_phys) };

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

    // Read AP entry count using new shareable sync block
    let ap_entry_count = crate::arch::smp_sync::read_ap_entry();
    info!("AP_ENTRY_COUNT={} (from shareable sync block)", ap_entry_count);
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

        // Allocate KernelArgsAp in physical frame (not stack) for AP to access before MMU setup
        let args_frame = allocate_p2frame(1)
            .expect("Failed to allocate args frame for AP");
        let args_phys = args_frame.base().data() as u64;
        let args_virt = (args_phys as usize + crate::PHYS_OFFSET) as *mut crate::arch::start::KernelArgsAp;

        // Get kernel physical base from memory map (needed for args and entry point)
        let kernel_phys_base = crate::startup::memory::kernel_phys_base() as u64;

        // Get the separate identity mapping page table we created
        let idmap_phys = crate::startup::memory::idmap_pg_dir().data() as u64;

        // Read BSP's TCR_EL1 and MAIR_EL1 values to pass to APs
        let tcr_el1: u64;
        let mair_el1: u64;
        unsafe {
            core::arch::asm!(
                "mrs {}, tcr_el1",
                "mrs {}, mair_el1",
                out(reg) tcr_el1,
                out(reg) mair_el1,
            );
        }
        debug!("BSP TCR_EL1=0x{:x}, MAIR_EL1=0x{:x}", tcr_el1, mair_el1);

        // Write args to allocated frame
        unsafe {
            args_virt.write(crate::arch::start::KernelArgsAp {
                cpu_id: cpu_id as u64,  // Use actual CPU ID
                page_table: page_table_phys,
                stack_start: stack_start as u64,
                stack_end: stack_end as u64,
                kernel_phys_base,
                idmap_pg_dir: idmap_phys,  // Use kernel PT (already identity-mapped)
                tcr_el1,   // BSP's TCR value
                mair_el1,  // BSP's MAIR value
            });

            // CRITICAL: Ensure args are visible to other CPUs
            // Clean the data cache for this memory region to PoC (Point of Coherency)
            // so the AP can see the data we just wrote
            core::arch::asm!(
                "dc cvac, {addr}",  // Clean data cache by VA to PoC
                "dsb sy",           // Data Synchronization Barrier
                "isb",              // Instruction Synchronization Barrier
                addr = in(reg) args_virt,
                options(nostack)
            );
        }

        // Get entry point address (PHYSICAL not virtual!)
        let entry_point_virt = crate::arch::start::kstart_ap as *const () as u64;

        // On aarch64, bootloader loads kernel at some physical address
        // and maps it to virtual address at KERNEL_OFFSET (0xFFFF_FF00_0000_0000).
        //
        // We need to convert the current virtual address back to physical.
        use crate::arch::consts::KERNEL_OFFSET;

        // Calculate offset within kernel and add to physical base
        let offset = entry_point_virt - KERNEL_OFFSET as u64;
        let entry_point_phys = kernel_phys_base + offset;

        info!("PSCI CPU_ON: mpidr=0x{:x}, entry_phys=0x{:x}, kernel_pt=0x{:x}, idmap_pt=0x{:x}",
              mpidr, entry_point_phys, page_table_phys, idmap_phys);

        // Reset AP_READY flag
        crate::arch::start::AP_READY.store(false, Ordering::SeqCst);

        // Call PSCI CPU_ON - pass PHYSICAL addresses
        let result = unsafe { psci_call(PSCI_CPU_ON_64, mpidr, entry_point_phys, args_phys) };

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
