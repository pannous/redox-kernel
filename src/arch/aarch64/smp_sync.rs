//! SMP synchronization primitives with explicit cache coherency
//!
//! This module provides sync variables allocated in freshly-mapped pages
//! with explicit shareability attributes, avoiding issues with bootloader-mapped
//! .bss/.data sections that may lack proper coherency attributes.

use core::sync::atomic::{AtomicU32, Ordering};
use crate::memory::{Frame, allocate_frame};
use crate::paging::{RmmA, RmmArch};

/// Shareable sync variables allocated in properly-mapped memory
#[repr(C, align(64))] // Cache line alignment to avoid false sharing
pub struct SmpSyncBlock {
    /// Counter for APs that successfully entered start_ap
    pub ap_entry_count: AtomicU32,

    /// Alternative volatile counter for testing
    pub ap_entry_volatile: u32,

    /// AP ready flags (one per CPU)
    pub ap_ready_flags: [AtomicU32; 16],
}

impl SmpSyncBlock {
    pub const fn new() -> Self {
        const ATOMIC_ZERO: AtomicU32 = AtomicU32::new(0);
        Self {
            ap_entry_count: AtomicU32::new(0),
            ap_entry_volatile: 0,
            ap_ready_flags: [ATOMIC_ZERO; 16],
        }
    }
}

/// Global pointer to shareable sync block (allocated in init_smp_sync)
static mut SMP_SYNC_PTR: *mut SmpSyncBlock = core::ptr::null_mut();

/// Assembly-accessible storage for SMP sync pointer
/// Must have a stable address that assembly can reference
#[unsafe(no_mangle)]
static mut __smp_sync_ptr_storage: *mut SmpSyncBlock = core::ptr::null_mut();

/// Device memory sync block for fallback testing
static mut SMP_SYNC_DEVICE_PTR: *mut SmpSyncBlock = core::ptr::null_mut();

/// Initialize SMP sync variables in freshly-mapped shareable memory
///
/// This allocates a new page and maps it with explicit Inner Shareable + Write-Back
/// cacheable attributes, ensuring cache coherency across all CPUs.
pub unsafe fn init_smp_sync_normal() -> Result<(), &'static str> {
    // Allocate a fresh physical frame
    let frame = allocate_frame().ok_or("Failed to allocate frame for SMP sync")?;
    let phys_addr = frame.base();

    info!("SMP sync: Allocated frame at phys=0x{:x}", phys_addr.data());

    // Map it to virtual address space
    // RMM's phys_to_virt uses the direct-map region with proper shareable attributes
    let virt_addr = RmmA::phys_to_virt(phys_addr);

    // Zero initialize the block
    let sync_block = virt_addr.data() as *mut SmpSyncBlock;
    sync_block.write(SmpSyncBlock::new());

    // Force cache clean to PoC (Point of Coherency)
    core::arch::asm!(
        "dc cvac, {addr}",
        "dsb ish",
        "isb",
        addr = in(reg) sync_block as usize,
        options(nostack)
    );

    // Store global pointers (both for Rust and assembly access)
    SMP_SYNC_PTR = sync_block;
    __smp_sync_ptr_storage = sync_block;

    info!("SMP sync: Normal memory block initialized at virt=0x{:x}", sync_block as usize);

    // Don't let the frame be freed
    core::mem::forget(frame);

    Ok(())
}

/// Initialize SMP sync in DEVICE memory (always coherent, but slower)
///
/// Device memory guarantees coherency across all CPUs without cache maintenance,
/// but has stricter ordering and may be slower than Normal memory.
pub unsafe fn init_smp_sync_device() -> Result<(), &'static str> {
    // For device memory, we need to map a physical address with Device-nGnRnE attributes
    // QEMU typically has device memory regions around 0x0900_0000 (serial) or 0x0800_0000

    // Allocate a fresh frame but we'll remap it as Device memory
    let frame = allocate_frame().ok_or("Failed to allocate frame for SMP device sync")?;
    let phys_addr = frame.base();

    info!("SMP sync: Allocated device frame at phys=0x{:x}", phys_addr.data());

    // TODO: We need a way to map with Device memory attributes
    // For now, this is a placeholder - we'd need to extend PageMapper
    // to support explicit MAIR index selection for Device memory

    warn!("Device memory sync not fully implemented - needs MAIR configuration");
    core::mem::forget(frame);

    Err("Device memory mapping not implemented yet")
}

/// Get raw pointer to shareable sync block (for direct AP access)
pub fn get_raw_sync_ptr() -> *mut SmpSyncBlock {
    unsafe { SMP_SYNC_PTR }
}

/// Get reference to shareable sync block (Normal memory)
pub fn get_smp_sync() -> Option<&'static SmpSyncBlock> {
    unsafe {
        if SMP_SYNC_PTR.is_null() {
            None
        } else {
            Some(&*SMP_SYNC_PTR)
        }
    }
}

/// Get mutable reference to shareable sync block (use with care!)
pub fn get_smp_sync_mut() -> Option<&'static mut SmpSyncBlock> {
    unsafe {
        if SMP_SYNC_PTR.is_null() {
            None
        } else {
            Some(&mut *SMP_SYNC_PTR)
        }
    }
}

/// Helper: increment AP entry counter with explicit cache maintenance
#[inline]
pub fn increment_ap_entry() -> u32 {
    if let Some(sync) = get_smp_sync() {
        // Ensure all prior writes are visible
        unsafe {
            core::arch::asm!("dmb ish", options(nostack, nomem));
        }

        let old = sync.ap_entry_count.fetch_add(1, Ordering::SeqCst);

        // Clean cache to PoC
        unsafe {
            core::arch::asm!(
                "dc cvac, {addr}",
                "dsb ish",
                addr = in(reg) &sync.ap_entry_count as *const _ as usize,
                options(nostack)
            );
        }

        old
    } else {
        warn!("SMP sync not initialized!");
        0
    }
}

/// Helper: read AP entry count with cache invalidation
#[inline]
pub fn read_ap_entry() -> u32 {
    if let Some(sync) = get_smp_sync() {
        // Invalidate cache to force read from memory
        unsafe {
            core::arch::asm!(
                "dc ivac, {addr}",
                "dsb ish",
                addr = in(reg) &sync.ap_entry_count as *const _ as usize,
                options(nostack)
            );
        }

        let val = sync.ap_entry_count.load(Ordering::SeqCst);

        unsafe {
            core::arch::asm!("dmb ish", options(nostack, nomem));
        }

        val
    } else {
        0
    }
}
