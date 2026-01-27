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

/// Per-CPU flag to indicate if this CPU is an AP (set by assembly before jumping to start)
/// BSP never sets this, so it stays false for BSP
#[unsafe(no_mangle)]
static mut IS_AP_FLAG: bool = false;

/// Alternative counter using volatile ptr (bypasses atomic infrastructure for testing)
pub static mut AP_ENTRY_VOLATILE: u32 = 0;

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
/// This is now a SHARED entry point for both BSP and APs!
/// - BSP: x0 = KernelArgs*
/// - AP:  x0 = KernelArgsAp* (phys addr)
/// We detect which one by reading MPIDR_EL1 (BSP has MPIDR=0)
unsafe extern "C" fn start(args_ptr: *const KernelArgs) -> ! {
    // Write serial marker to confirm we're in Rust!
    unsafe {
        let serial = 0x09000000 as *mut u32;
        core::ptr::write_volatile(serial, 0x52); // 'R' = Rust entry!
    }

    // Read MPIDR_EL1 to detect if this is BSP or AP
    let mpidr: u64;
    unsafe {
        core::arch::asm!("mrs {}, mpidr_el1", out(reg) mpidr);
    }

    // Check if this is an AP (MPIDR != 0 for APs)
    // Note: We mask to get just the affinity bits, ignoring reserved bits
    let affinity = mpidr & 0xFF_00FF_FFFF;

    if affinity != 0 {
        // This is an AP! Branch to AP initialization
        unsafe {
            let serial = 0x09000000 as *mut u32;
            core::ptr::write_volatile(serial, 0x41); // 'A' = AP detected
            core::ptr::write_volatile(serial, 0x50); // 'P'
        }

        start_ap_shared(args_ptr as usize);
    }

    // This is the BSP - continue with normal initialization
    unsafe {
        let serial = 0x09000000 as *mut u32;
        core::ptr::write_volatile(serial, 0x42); // 'B' = BSP
    }

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

            // Initialize SMP sync variables in freshly-mapped shareable memory
            // This must happen after allocator::init() so we can allocate frames
            info!("Initializing SMP sync variables with explicit shareability...");
            if let Err(e) = crate::arch::smp_sync::init_smp_sync_normal() {
                error!("Failed to init SMP sync (Normal memory): {}", e);
            } else {
                info!("SMP sync initialized successfully in Normal shareable memory");
            }

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

// External declaration for assembly entry point
unsafe extern "C" {
    pub fn kstart_ap();
}

/// Assembly entry point for Application Processors (called by PSCI)
///
/// CRITICAL: PSCI starts with MMU in unknown state. Symbols are linked at KERNEL_OFFSET
/// virtual addresses, but PC is at physical/identity-mapped address.
global_asm!("
    .globl kstart_ap
    kstart_ap:
        // x0 = args_phys (physical address of KernelArgsAp struct)

        // Serial debug marker 'A' - AP entry
        mov x9, #0x09000000
        mov w11, #0x41  // 'A'
        str w11, [x9]

        // Save x0 for later - we'll need it after page table setup
        mov x10, x0

        // Load page_table (offset 8 in KernelArgsAp)
        ldr x1, [x0, #8]
        msr ttbr1_el1, x1
        msr ttbr0_el1, x1

        // Flush TLB
        dsb sy
        tlbi vmalle1
        dsb sy
        isb

        // Serial debug marker 'B' - Page tables loaded, MMU enabled
        mov w11, #0x42  // 'B'
        str w11, [x9]

        // **CRITICAL FIX**: Jump to virtual address space!
        //
        // The issue: After MMU enable, PC is at physical/identity address (0x8e0fXXXX)
        // but symbols are at virtual address (0xffffff00XXXXXXXX).
        // PC-relative addressing doesn't work across this gap.
        //
        // Solution: Build virtual address using MOV immediate instructions
        // Then branch to it explicitly.
        //
        // Get offset of next instruction from kernel base
        adr x3, .Lcontinue_virt  // x3 = physical address
        movz x4, #0x0000, lsl #16  // kernel phys base low (will adjust)
        movk x4, #0x8e0f, lsl #16  // 0x8e0f0000
        sub x3, x3, x4             // x3 = offset from kernel base

        // Build virtual address = KERNEL_OFFSET + offset
        // KERNEL_OFFSET = 0xffffff00_00000000
        movz x4, #0x0000, lsl #0   // bits 0-15
        movk x4, #0x0000, lsl #16  // bits 16-31
        movk x4, #0xff00, lsl #32  // bits 32-47
        movk x4, #0xffff, lsl #48  // bits 48-63
        add x3, x3, x4             // x3 = virtual address

        // Write marker showing we computed address
        mov w11, #0x56  // 'V'
        str w11, [x9]

        // Jump to virtual space
        br x3

    .Lcontinue_virt:
        // NOW we're in virtual address space! PC is at 0xffffff00_xxxxxxxx
        // Skip serial markers - just set up and jump to Rust!

        // Convert args_phys (x10) to virtual address
        // args_virt = PHYS_OFFSET + args_phys
        // PHYS_OFFSET = 0xFFFF_8000_0000_0000
        movz x6, #0x0000, lsl #0
        movk x6, #0x0000, lsl #16
        movk x6, #0x8000, lsl #32
        movk x6, #0xFFFF, lsl #48
        add x10, x10, x6  // x10 now points to args in virtual space

        // Setup exception handlers
        ldr x4, =exception_vector_base
        msr vbar_el1, x4
        isb

        // Load stack_end (offset 24 in KernelArgsAp)
        ldr x2, [x10, #24]
        mov sp, x2

        // Restore args pointer to x0 (start() expects it there)
        mov x0, x10

        // Clear link register
        mov lr, #0

        // Final synchronization
        dsb ish
        isb

        // Jump to start() - NOW it should work!
        b {start}

        // This should NEVER execute if branch succeeds
        mov w11, #0x58  // 'X' = branch failed!
        str w11, [x9]
    .Lap_stuck:
        wfi
        b .Lap_stuck
    ",
    start = sym start,
);

/// AP initialization - called from shared start() function
/// This runs in the proven-working Rust environment, avoiding assembly-to-Rust transition issues
#[inline(never)]
unsafe fn start_ap_shared(args_phys: usize) -> ! {
    // Write serial to confirm we're in AP init
    let serial = 0x09000000 as *mut u32;
    unsafe {
        core::ptr::write_volatile(serial, 0x49); // 'I' = Init
    }

    // Increment the shareable sync counter (now that we're in working Rust!)
    let old = crate::arch::smp_sync::increment_ap_entry();

    unsafe {
        core::ptr::write_volatile(serial, 0x2B); // '+'
        // Write count as digit
        if old < 10 {
            core::ptr::write_volatile(serial, 0x30 + old);
        } else {
            core::ptr::write_volatile(serial, 0x58); // 'X'
        }
    }

    // Convert phys to virt and read AP args
    let args_ptr = (args_phys + crate::PHYS_OFFSET) as *const KernelArgsAp;
    let args = unsafe { &*args_ptr };
    let cpu_id = crate::cpu_set::LogicalCpuId::new(args.cpu_id as u32);

    // Initialize paging (MAIR)
    paging::init();

    // Initialize per-CPU block and set TPIDR_EL1
    crate::misc::init(cpu_id);

    warn!("AP {}: Initialized via shared start() path!", cpu_id.get());

    // Signal readiness
    AP_READY.store(true, Ordering::SeqCst);

    // Wait for BSP to complete initialization
    while !BSP_READY.load(Ordering::SeqCst) {
        core::hint::spin_loop();
    }

    // Call kmain_ap to enter scheduler
    crate::kmain_ap(cpu_id);
}
