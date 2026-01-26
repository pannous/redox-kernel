//! This function is where the kernel sets up IRQ handlers
//! It is incredibly unsafe, and should be minimal in nature
//! It must create the IDT with the correct entries, those entries are
//! defined in other files inside of the `arch` module
use core::{
    arch::global_asm,
    cell::SyncUnsafeCell,
    slice,
    sync::atomic::{AtomicBool, Ordering},
};

use fdt::Fdt;

use crate::{
    allocator, device, devices::graphical_debug, dtb, paging, startup::KernelArgs,
    CPU_COUNT,
};

use core::sync::atomic::Ordering as AtomicOrdering;

/// Test of zero values in BSS.
static mut BSS_TEST_ZERO: usize = 0;
/// Test of non-zero values in data.
static mut DATA_TEST_NONZERO: usize = 0xFFFF_FFFF_FFFF_FFFF;

pub static AP_READY: AtomicBool = AtomicBool::new(false);
static BSP_READY: AtomicBool = AtomicBool::new(false);

/// Enumerate CPU cores from device tree
unsafe fn enumerate_cpus_from_dtb(dtb: &Fdt) -> u32 {
    let mut cpu_count = 0;

    debug!("DTB: Starting CPU enumeration from device tree");

    // Debug: try to iterate all nodes to see what's there
    let mut node_count = 0;
    for node in dtb.all_nodes() {
        node_count += 1;
        if node.name.contains("cpu") {
            debug!("DTB: Found node with 'cpu' in name: {}", node.name);
        }
    }
    debug!("DTB: Total nodes in tree: {}", node_count);

    if let Some(cpus_node) = dtb.find_node("/cpus") {
        debug!("DTB: Found /cpus node");

        // Check address-cells and size-cells
        if let Some(addr_cells) = cpus_node.property("#address-cells") {
            debug!("DTB: /cpus #address-cells = {:?}", addr_cells.as_usize());
        }
        if let Some(size_cells) = cpus_node.property("#size-cells") {
            debug!("DTB: /cpus #size-cells = {:?}", size_cells.as_usize());
        }

        // Iterate through all CPU nodes
        let mut child_count = 0;
        for cpu_node in cpus_node.children() {
            child_count += 1;
            let name = cpu_node.name;
            debug!("DTB: Child {}: name={}", child_count, name);

            if name.starts_with("cpu@") || name == "cpu" {
                // Get CPU ID from reg property
                if let Some(reg) = cpu_node.property("reg") {
                    let cpu_id = reg.as_usize().unwrap_or(cpu_count as usize);
                    debug!("DTB: Found CPU {} (reg={})", cpu_count, cpu_id);

                    // Check if CPU is enabled
                    let enabled = cpu_node
                        .property("status")
                        .and_then(|s| s.as_str())
                        .map(|s| s == "okay" || s == "ok")
                        .unwrap_or(true); // Default to enabled if no status property

                    if enabled {
                        cpu_count += 1;

                        // Log enable-method for debugging
                        if let Some(method) = cpu_node.property("enable-method").and_then(|m| m.as_str()) {
                            debug!("  enable-method: {}", method);
                        }
                    } else {
                        debug!("  CPU {} is disabled", cpu_id);
                    }
                } else {
                    // No reg property, count it anyway
                    debug!("DTB: Found CPU {} (no reg property)", cpu_count);
                    cpu_count += 1;
                }
            }
        }
        debug!("DTB: /cpus has {} children", child_count);
    } else {
        debug!("DTB: /cpus node not found!");
    }

    if cpu_count == 0 {
        debug!("DTB: No CPUs found in /cpus node, defaulting to 1");
        cpu_count = 1;
    }

    debug!("DTB: Detected {} CPU(s)", cpu_count);
    cpu_count
}

#[repr(C, align(16))]
struct StackAlign<T>(T);

static STACK: SyncUnsafeCell<StackAlign<[u8; 128 * 1024]>> =
    SyncUnsafeCell::new(StackAlign([0; 128 * 1024]));

global_asm!("
    .globl kstart
    kstart:
        // BSS should already be zero
        adrp x9, {bss_test_zero}
        ldr x9, [x9, :lo12:{bss_test_zero}]
        cbnz x9, .Lkstart_crash
        adrp x9, {data_test_nonzero}
        ldr x9, [x9, :lo12:{data_test_nonzero}]
        cbz x9, .Lkstart_crash

        adrp x1, {stack}
        add x1, x1, :lo12:{stack}
        mov x2, {stack_size}-16
        add sp, x1, x2

        // Setup interrupt handlers
        ldr x9, =exception_vector_base
        msr vbar_el1, x9

        mov lr, 0
        b {start}

    .Lkstart_crash:
        mov x9, 0
        br x9
    ",
    bss_test_zero = sym BSS_TEST_ZERO,
    data_test_nonzero = sym DATA_TEST_NONZERO,
    stack = sym STACK,
    stack_size = const size_of_val(&STACK),
    start = sym start,
);

/// The entry to Rust, all things must be initialized
unsafe extern "C" fn start(args_ptr: *const KernelArgs) -> ! {
    unsafe {
        let bootstrap = {
            let args = args_ptr.read();

            // Set up graphical debug
            graphical_debug::init(args.env());

            // Get hardware descriptor data
            //TODO: use env {DTB,RSDT}_{BASE,SIZE}?
            debug!("DTB: hwdesc_base=0x{:x}, hwdesc_size=0x{:x}", args.hwdesc_base, args.hwdesc_size);
            let hwdesc_data = if args.hwdesc_base != 0 {
                debug!("DTB: Creating hwdesc_data slice");
                Some(slice::from_raw_parts(
                    (crate::PHYS_OFFSET + args.hwdesc_base as usize) as *const u8,
                    args.hwdesc_size as usize,
                ))
            } else {
                debug!("DTB: No hwdesc_base, hwdesc_data is None");
                None
            };

            let dtb_res = hwdesc_data
                .ok_or(fdt::FdtError::BadPtr)
                .and_then(|data| {
                    debug!("DTB: Parsing DTB from hwdesc_data, size={}", data.len());
                    Fdt::new(data)
                });

            // Try to find serial port prior to logging
            if let Ok(dtb) = &dtb_res {
                device::serial::init_early(dtb);
            }

            info!("Redox OS starting...");
            args.print();

            // Pre-register QEMU virt device memory in case DTB parsing fails
            // This ensures device MMIO can be accessed even without DTB
            crate::startup::memory::register_memory_region(
                0x08000000, 0x08000000,
                crate::startup::memory::BootloaderMemoryKind::Device
            );
            crate::startup::memory::register_memory_region(
                0x10000000, 0x30000000,
                crate::startup::memory::BootloaderMemoryKind::Device
            );
            // QEMU virt highmem PCI ECAM at 0x4010000000 (256 MB for 256 buses)
            crate::startup::memory::register_memory_region(
                0x40_1000_0000, 0x1000_0000,
                crate::startup::memory::BootloaderMemoryKind::Device
            );

            // Initialize RMM
            crate::startup::memory::init(&args, None, None);

            // Initialize paging
            paging::init();

            crate::misc::init(crate::cpu_set::LogicalCpuId::new(0));

            // Reset AP variables
            AP_READY.store(false, Ordering::SeqCst);
            BSP_READY.store(false, Ordering::SeqCst);

            // Setup kernel heap
            allocator::init();

            // Activate memory logging
            crate::log::init();

            // Initialize devices
            //DEBUG_MARKER.store(100, AtomicOrdering::SeqCst);
            match dtb_res {
                Ok(dtb) => {
                    //DEBUG_MARKER.store(200, AtomicOrdering::SeqCst);

                    // Enumerate CPUs from DTB BEFORE dtb::init
                    let detected_cpus = enumerate_cpus_from_dtb(&dtb);
                    //DEBUG_MARKER.store(300 + detected_cpus, AtomicOrdering::SeqCst);
                    CPU_COUNT.store(detected_cpus, AtomicOrdering::SeqCst);

                    dtb::init(hwdesc_data.map(|slice| (slice.as_ptr() as usize, slice.len())));
                    device::init_devicetree(&dtb);

                    // Start secondary CPUs after DTB/GIC initialization
                    #[cfg(feature = "multi_core")]
                    {
                        crate::acpi::madt::start_secondary_cpus_dtb();
                    }
                }
                Err(err) => {
                    //DEBUG_MARKER.store(999, AtomicOrdering::SeqCst);
                    dtb::init(None);
                    warn!("failed to parse DTB: {}", err);

                    #[cfg(feature = "acpi")]
                    {
                        info!("START: About to call acpi::init");
                        crate::acpi::init(args.acpi_rsdp());
                        info!("START: acpi::init returned");
                    }

                    // Try QEMU virt machine's known PL031 RTC address as fallback
                    device::rtc::init_qemu_virt();
                }
            }

            BSP_READY.store(true, Ordering::SeqCst);

            args.bootstrap()
        };

        crate::kmain(bootstrap);
    }
}

#[repr(C, packed)]
pub struct KernelArgsAp {
    pub cpu_id: u64,
    pub page_table: u64,
    pub stack_start: u64,
    pub stack_end: u64,
}

/// Assembly entry point for Application Processors
///
/// Called by PSCI CPU_ON with:
/// - x0 = context parameter (args_ptr)
/// - MMU disabled
/// - EL1
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kstart_ap(args_ptr: *const KernelArgsAp) -> ! {
    unsafe {
        let cpu_id = {
            let args = &*args_ptr;

            let cpu_id = crate::cpu_set::LogicalCpuId::new(args.cpu_id as u32);

            // Set up exception vectors (VBAR_EL1)
            core::arch::asm!(
                "ldr x9, =exception_vector_base",
                "msr vbar_el1, x9",
                out("x9") _,
            );

            // Set up stack pointer from args
            core::arch::asm!(
                "mov sp, {}",
                in(reg) args.stack_end,
            );

            // Configure page tables for this CPU (TTBR1_EL1)
            crate::device::cpu::registers::control_regs::ttbr1_el1_write(args.page_table);

            // Flush TLB
            core::arch::asm!(
                "dsb sy",
                "tlbi vmalle1",
                "dsb sy",
                "isb",
            );

            // Initialize paging (MAIR)
            paging::init();

            // Initialize per-CPU block and set TPIDR_EL1
            crate::misc::init(cpu_id);

            // Note: GIC CPU interface initialization happens automatically
            // on GICv2, the CPU interface is banked per-CPU and accessed at
            // the same address, routed by hardware based on the accessing CPU.
            // BSP already initialized the GIC distributor and CPU interfaces.

            // Signal readiness
            AP_READY.store(true, Ordering::SeqCst);

            cpu_id
        };

        // Wait for BSP to complete initialization
        debug!("AP CPU {} waiting for BSP_READY", cpu_id.get());
        while !BSP_READY.load(Ordering::SeqCst) {
            core::hint::spin_loop();
        }
        debug!("AP CPU {} BSP ready, calling kmain_ap", cpu_id.get());

        // Call kmain_ap to enter scheduler
        crate::kmain_ap(cpu_id);
    }
}
