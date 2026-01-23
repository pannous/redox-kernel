use alloc::{boxed::Box, vec::Vec};

use super::{Madt, MadtEntry};
use crate::{
    device::irqchip::{
        gic::{GenericInterruptController, GicCpuIf, GicDistIf},
        gicv3::{GicV3, GicV3CpuIf},
    },
    dtb::irqchip::{IrqChipItem, IRQ_CHIP},
    memory::{map_device_memory, PhysicalAddress, PAGE_SIZE},
};

pub(super) fn init(madt: Madt) {
    let mut gicd_opt = None;
    let mut giccs = Vec::new();
    for madt_entry in madt.iter() {
        debug!("      {:#x?}", madt_entry);
        match madt_entry {
            MadtEntry::Gicc(gicc) => {
                debug!("SMP: Found GICC entry (CPU)");
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
        info!("SMP: ACPI MADT lists {} CPU(s)", cpu_count);
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
    info!("{:#x?}", gic_dist_if);
    info!("SMP: GIC distributor initialized, version {}", gicd.gic_version);

    match gicd.gic_version {
        1 | 2 => {
            // GICv2: Initialize all CPU interfaces
            let mut cpu_idx = 0;
            for gicc in giccs {
                debug!("SMP: Initializing GICv2 CPU interface {}", cpu_idx);

                let mut gic_cpu_if = GicCpuIf::default();
                unsafe {
                    let phys = PhysicalAddress::new(gicc.physical_base_address as usize);
                    let virt = map_device_memory(phys, PAGE_SIZE);
                    gic_cpu_if.init(virt.data())
                };
                info!("SMP: GIC CPU {} interface: {:#x?}", cpu_idx, gic_cpu_if);

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
            info!("SMP: Initialized {} GICv2 CPU interfaces", cpu_idx);
        }
        3 => {
            // GICv3: Initialize all CPU interfaces
            let mut cpu_idx = 0;
            for _gicc in giccs {
                debug!("SMP: Initializing GICv3 CPU interface {}", cpu_idx);

                let mut gic_cpu_if = GicV3CpuIf;
                unsafe { gic_cpu_if.init() };
                info!("SMP: GIC CPU {} interface: {:#x?}", cpu_idx, gic_cpu_if);

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
            info!("SMP: Initialized {} GICv3 CPU interfaces", cpu_idx);
        }
        _ => {
            warn!("unsupported GIC version {}", gicd.gic_version);
        }
    }
    unsafe { IRQ_CHIP.init(None) };
}
