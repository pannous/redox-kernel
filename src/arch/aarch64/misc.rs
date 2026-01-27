use crate::{
    cpu_set::LogicalCpuId,
    paging::{RmmA, RmmArch},
    percpu::PercpuBlock,
};

impl PercpuBlock {
    pub fn current() -> &'static Self {
        unsafe { &*(crate::device::cpu::registers::control_regs::tpidr_el1() as *const Self) }
    }
}

#[cold]
pub unsafe fn init(cpu_id: LogicalCpuId) {
    unsafe {
        let frame = crate::memory::allocate_frame().expect("failed to allocate percpu memory");
        let virt = RmmA::phys_to_virt(frame.base()).data() as *mut PercpuBlock;

        virt.write(PercpuBlock::init(cpu_id));

        crate::device::cpu::registers::control_regs::tpidr_el1_write(virt as u64);

        // Register this CPU's PercpuBlock so get_all_stats() can find it
        // and sys:stat can report per-CPU statistics
        crate::percpu::init_tlb_shootdown(cpu_id, virt);
    }
}
