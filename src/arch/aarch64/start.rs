//! This function is where the kernel sets up IRQ handlers
//! It is incredibly unsafe, and should be minimal in nature
//! It must create the IDT with the correct entries, those entries are
//! defined in other files inside of the `arch` module
use core::{
    arch::global_asm,
    cell::SyncUnsafeCell,
    slice,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
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

/// Counter to track how many APs actually entered kstart_ap
pub static AP_ENTRY_COUNT: AtomicU32 = AtomicU32::new(0);

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

            // Get hardware descriptor data FIRST (needed for serial init)
            //TODO: use env {DTB,RSDT}_{BASE,SIZE}?
            let (hwdesc_data, hwdesc_is_dtb) = if args.hwdesc_base != 0 {
                // Peek at the data to determine if it's DTB or ACPI
                let header_ptr = (crate::PHYS_OFFSET + args.hwdesc_base as usize) as *const u8;
                let peek_size = core::cmp::min(args.hwdesc_size as usize, 8);
                if peek_size >= 8 {
                    let header_slice = slice::from_raw_parts(header_ptr, peek_size);

                    // Check for DTB magic (0xd00dfeed in big-endian)
                    let magic = u32::from_be_bytes([header_slice[0], header_slice[1], header_slice[2], header_slice[3]]);
                    if magic == 0xd00dfeed {
                        // This is a DTB - read actual size from header
                        let reported_size = u32::from_be_bytes([header_slice[4], header_slice[5], header_slice[6], header_slice[7]]) as usize;
                        let dtb_size = core::cmp::max(reported_size + 65536, args.hwdesc_size as usize);
                        (Some(slice::from_raw_parts(header_ptr, dtb_size)), true)
                    } else if header_slice.starts_with(b"RSD PTR ") {
                        // This is ACPI RSDP, not DTB
                        (None, false)
                    } else {
                        // Unknown format, try using it anyway
                        (Some(slice::from_raw_parts(header_ptr, args.hwdesc_size as usize)), false)
                    }
                } else {
                    // Too small to be a DTB, likely ACPI
                    (None, false)
                }
            } else {
                (None, false)
            };

            let dtb_res = if hwdesc_is_dtb {
                hwdesc_data
                    .ok_or(fdt::FdtError::BadPtr)
                    .and_then(|data| Fdt::new(data))
            } else {
                // Not DTB data, don't try to parse it
                Err(fdt::FdtError::BadPtr)
            };

            // Initialize serial FIRST so all debug output is captured
            // Try DTB-based init, fallback to hardcoded QEMU virt UART if DTB fails
            match &dtb_res {
                Ok(dtb) => {
                    device::serial::init_early(dtb);
                }
                Err(_) => {
                    // DTB failed - use hardcoded QEMU virt PL011 UART at 0x09000000
                    error!("DTB parsing failed, using hardcoded QEMU virt UART");
                    unsafe {
                        use crate::devices::uart_pl011;
                        let virt = crate::PHYS_OFFSET + 0x09000000;
                        let mut serial_port = uart_pl011::SerialPort::new(virt, false);
                        serial_port.init(false);
                        *crate::device::serial::COM1.lock() =
                            crate::devices::serial::SerialKind::Pl011(serial_port);
                    }
                }
            }

            // Set up graphical debug AFTER serial
            graphical_debug::init(args.env());

            // Now all these debug messages go to BOTH serial and framebuffer
            debug!("DTB: hwdesc_base=0x{:x}, hwdesc_size=0x{:x}", args.hwdesc_base, args.hwdesc_size);
            if let Some(data) = hwdesc_data {
                debug!("DTB: Creating hwdesc_data slice");
                info!("DTB: Parsed DTB from hwdesc_data, size={}", data.len());
            } else {
                debug!("DTB: No hwdesc_base, hwdesc_data is None");
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
                    // Only show error if we expected DTB but parsing failed
                    if hwdesc_is_dtb {
                        error!("*** FAILED to parse DTB: {:?} ***", err);
                    } else if args.hwdesc_base != 0 {
                        debug!("Hardware descriptor is not DTB (likely ACPI), using ACPI path");
                    } else {
                        debug!("No hardware descriptor provided");
                    }

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
/// - x0 = context parameter (args_phys - physical address)
/// - MMU state undefined (might be on or off)
/// - EL1
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kstart_ap(args_phys: u64) -> ! {
    // Increment entry counter FIRST - before ANY other operations
    AP_ENTRY_COUNT.fetch_add(1, Ordering::SeqCst);

    unsafe {
        let cpu_id = {
            // Convert physical address to virtual address
            let args_ptr = (args_phys as usize + crate::PHYS_OFFSET) as *const KernelArgsAp;
            let args = &*args_ptr;

            let cpu_id = crate::cpu_set::LogicalCpuId::new(args.cpu_id as u32);
            let stack_end = args.stack_end;
            let page_table = args.page_table;

            warn!("AP {}: kstart_ap entered (total entries={})", cpu_id.get(), AP_ENTRY_COUNT.load(Ordering::SeqCst));

            // Set up exception vectors (VBAR_EL1)
            core::arch::asm!(
                "ldr x9, =exception_vector_base",
                "msr vbar_el1, x9",
                out("x9") _,
            );

            warn!("AP {}: VBAR set", cpu_id.get());

            // Set up stack pointer from args
            core::arch::asm!(
                "mov sp, {}",
                in(reg) stack_end,
            );

            warn!("AP {}: Stack set to 0x{:x}", cpu_id.get(), stack_end);

            // Configure page tables for this CPU (TTBR1_EL1)
            crate::device::cpu::registers::control_regs::ttbr1_el1_write(page_table);

            warn!("AP {}: TTBR1 set to 0x{:x}", cpu_id.get(), page_table);

            // Flush TLB
            core::arch::asm!(
                "dsb sy",
                "tlbi vmalle1",
                "dsb sy",
                "isb",
            );

            warn!("AP {}: TLB flushed", cpu_id.get());

            // Initialize paging (MAIR)
            paging::init();

            warn!("AP {}: Paging initialized", cpu_id.get());

            // Initialize per-CPU block and set TPIDR_EL1
            crate::misc::init(cpu_id);

            warn!("AP {}: Percpu block initialized", cpu_id.get());

            // Note: GIC CPU interface initialization happens automatically
            // on GICv2, the CPU interface is banked per-CPU and accessed at
            // the same address, routed by hardware based on the accessing CPU.
            // BSP already initialized the GIC distributor and CPU interfaces.

            // Signal readiness
            AP_READY.store(true, Ordering::SeqCst);

            warn!("AP {}: Signaled readiness", cpu_id.get());

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
