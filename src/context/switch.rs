//! This module provides a context-switching mechanism that utilizes a simple round-robin scheduler.
//! The scheduler iterates over available contexts, selecting the next context to run, while
//! handling process states and synchronization.
use core::{
    cell::{Cell, RefCell},
    hint, mem,
    ops::Bound,
    sync::atomic::Ordering,
};

use alloc::sync::Arc;
use syscall::PtraceFlags;

use crate::{
    context::{arch, contexts, ArcContextLockWriteGuard, Context, ContextLock},
    cpu_set::LogicalCpuId,
    cpu_stats,
    percpu::PercpuBlock,
    sync::CleanLockToken,
    time,
};

use super::ContextRef;

enum UpdateResult {
    CanSwitch,
    Skip,
}

/// Determines if a given context is eligible to be scheduled on a given CPU (in
/// principle, the current CPU).
///
/// # Safety
/// This function is unsafe because it modifies the `context`'s state directly without synchronization.
///
/// # Parameters
/// - `context`: The context (process/thread) to be checked.
/// - `cpu_id`: The logical ID of the CPU on which the context is being scheduled.
///
/// # Returns
/// - `UpdateResult::CanSwitch`: If the context can be switched to.
/// - `UpdateResult::Skip`: If the context should be skipped (e.g., it's running on another CPU).
unsafe fn update_runnable(context: &mut Context, cpu_id: LogicalCpuId) -> UpdateResult {
    // Ignore contexts that are already running.
    if context.running {
        return UpdateResult::Skip;
    }

    // Ignore contexts assigned to other CPUs.
    if !context.sched_affinity.contains(cpu_id) {
        return UpdateResult::Skip;
    }

    // If context is soft-blocked and has a wake-up time, check if it should wake up.
    if context.status.is_soft_blocked() {
        if let Some(wake) = context.wake {
            let current = time::monotonic();
            if current >= wake {
                context.wake = None;
                context.unblock_no_ipi();
            }
        }
    }

    // If the context is runnable, indicate it can be switched to.
    if context.status.is_runnable() {
        UpdateResult::CanSwitch
    } else {
        UpdateResult::Skip
    }
}

struct SwitchResultInner {
    _prev_guard: ArcContextLockWriteGuard,
    _next_guard: ArcContextLockWriteGuard,
}

/// Tick function to update PIT ticks and trigger a context switch if necessary.
///
/// Called periodically, this function increments a per-CPU tick counter and performs a context
/// switch if the counter reaches a set threshold (e.g., every 3 ticks).
///
/// The function also calls the signal handler after switching contexts.
pub fn tick(token: &mut CleanLockToken) {
    let percpu = PercpuBlock::current();
    let ticks_cell = &percpu.switch_internals.pit_ticks;

    let new_ticks = ticks_cell.get() + 1;
    ticks_cell.set(new_ticks);

    // FIXME: Disabled periodic_log() - causes boot hang due to Vec allocation in interrupt context
    // crate::smp_diag::periodic_log();

    // Trigger a context switch every 3 ticks (~30ms at 100Hz).
    // IPC latency is handled by switch_pending flag set in unblock(), not by reducing this threshold.
    if new_ticks >= 3 {
        switch(token);
        crate::context::signal::signal_handler(token);
    }
}

/// Finishes the context switch by clearing any temporary data and resetting the lock.
///
/// This function is called after a context switch is completed to perform cleanup, including
/// clearing the switch result data and releasing the context switch lock.
///
/// # Safety
/// This function involves unsafe operations such as resetting state and releasing locks.
pub unsafe extern "C" fn switch_finish_hook() {
    warn!("switch_finish_hook: entered");
    unsafe {
        warn!("switch_finish_hook: about to take switch_result");
        match PercpuBlock::current().switch_internals.switch_result.take() {
            Some(switch_result) => {
                warn!("switch_finish_hook: got switch_result, dropping");
                drop(switch_result);
                warn!("switch_finish_hook: dropped switch_result");
            }
            _ => {
                warn!("switch_finish_hook: NO switch_result, calling emergency_reset");
                // TODO: unreachable_unchecked()?
                crate::arch::stop::emergency_reset();
            }
        }
        warn!("switch_finish_hook: releasing CONTEXT_SWITCH_LOCK");
        arch::CONTEXT_SWITCH_LOCK.store(false, Ordering::SeqCst);
        warn!("switch_finish_hook: calling switch_arch_hook");
        crate::percpu::switch_arch_hook();

        // CRITICAL: Enable interrupts after context switch
        // The new context inherits the DAIF register from the previous context,
        // which had interrupts disabled in the scheduler loop. We must re-enable
        // interrupts here so the new context can receive timer interrupts.
        warn!("switch_finish_hook: enabling interrupts");
        crate::interrupt::enable_and_nop();
        warn!("switch_finish_hook: interrupts enabled, returning");
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SwitchResult {
    Switched,
    AllContextsIdle,
}

/// Selects and switches to the next context using a round-robin scheduler.
///
/// This function performs the context switch, checking each context in a loop for eligibility
/// until it finds a context ready to run. If no other context is runnable, it returns to the
/// idle context.
///
/// # Warning
/// This is not memory-unsafe to call. But do NOT call this while holding locks!
///
/// # Returns
/// - `SwitchResult::Switched`: Indicates a successful switch to a new context.
/// - `SwitchResult::AllContextsIdle`: Indicates all contexts are idle, and the CPU will switch
///   to an idle context.
pub fn switch(token: &mut CleanLockToken) -> SwitchResult {
    warn!("context::switch: entered on CPU {}", crate::cpu_id().get());
    let percpu = PercpuBlock::current();
    warn!("context::switch: got percpu");
    cpu_stats::add_context_switch();
    warn!("context::switch: added cpu stats");
    percpu.stats.add_context_switch_local();
    warn!("context::switch: added percpu stats");

    let cpu_id = crate::cpu_id();
    warn!("context::switch: got cpu_id={}", cpu_id.get());

    //set PIT Interrupt counter to 0, giving each process same amount of PIT ticks
    percpu.switch_internals.pit_ticks.set(0);
    warn!("context::switch: reset pit_ticks");

    // Acquire the global lock to ensure exclusive access during context switch and avoid
    // issues that would be caused by the unsafe operations below
    // TODO: Better memory orderings?
    warn!("context::switch: about to acquire CONTEXT_SWITCH_LOCK");
    {
        use core::sync::atomic::AtomicU64;
        static SPIN_COUNTER: AtomicU64 = AtomicU64::new(0);
        let mut local_spins = 0u64;
        while arch::CONTEXT_SWITCH_LOCK
            .compare_exchange_weak(false, true, Ordering::SeqCst, Ordering::Relaxed)
            .is_err()
        {
            local_spins += 1;
            let total = SPIN_COUNTER.fetch_add(1, Ordering::Relaxed);
            if total % 1_000_000 == 0 {  // Log more frequently for debugging
                warn!("CS_LOCK spin: total={} local={} on CPU {}", total, local_spins, crate::cpu_id().get());
            }
            hint::spin_loop();
            percpu.maybe_handle_tlb_shootdown();
        }
    }
    warn!("context::switch: acquired CONTEXT_SWITCH_LOCK");

    let cpu_id = crate::cpu_id();
    warn!("context::switch: got cpu_id (2nd time)");

    let mut switch_context_opt = None;
    {
        warn!("context::switch: about to call contexts()");
        let contexts = contexts(token.token());
        warn!("context::switch: got contexts");

        // Lock the previous context.
        warn!("context::switch: about to get current context");
        let prev_context_lock = crate::context::current();
        warn!("context::switch: about to lock current context with write_arc()");
        // We are careful not to lock this context twice
        let prev_context_guard = unsafe { prev_context_lock.write_arc() };
        warn!("context::switch: locked current context");

        warn!("context::switch: checking is_preemptable()");
        if !prev_context_guard.is_preemptable() {
            warn!("context::switch: not preemptable, returning AllContextsIdle");
            // Release the lock before returning
            arch::CONTEXT_SWITCH_LOCK.store(false, Ordering::SeqCst);
            return SwitchResult::AllContextsIdle;
        }
        warn!("context::switch: is preemptable");

        warn!("context::switch: getting idle_context");
        let idle_context = percpu.switch_internals.idle_context();
        warn!("context::switch: got idle_context");

        // Stateful flag used to skip the idle process the first time it shows up.
        // After that, this flag is set to `false` so the idle process can be
        // picked up.
        let mut skip_idle = true;
        warn!("context::switch: set skip_idle={}, about to iterate contexts", skip_idle);

        // Attempt to locate the next context to switch to.
        for next_context_lock in contexts
            // Include all contexts with IDs greater than the current...
            .range((
                Bound::Excluded(ContextRef(Arc::clone(&prev_context_lock))),
                Bound::Unbounded,
            ))
            // ... and all contexts with IDs less than the current...
            .chain(contexts.range((
                Bound::Unbounded,
                Bound::Excluded(ContextRef(Arc::clone(&prev_context_lock))),
            )))
            .filter_map(ContextRef::upgrade)
            // ... and the idle context...
            .chain(Some(Arc::clone(&idle_context)))
        // ... but not the current context (note the `Bound::Excluded`),
        // which is already locked.
        {
            warn!("context::switch: for loop iteration, checking context");
            if Arc::ptr_eq(&next_context_lock, &idle_context) && skip_idle {
                warn!("context::switch: skipping idle context (first time)");
                // Skip idle process the first time it shows up, but allow it
                // to be picked up again the next time.
                skip_idle = false;
                continue;
            }

            {
                warn!("context::switch: locking next context");
                // Lock next context
                // We are careful not to lock this context twice
                let mut next_context_guard = unsafe { next_context_lock.write_arc() };
                warn!("context::switch: locked next context");

                // Check if the context is runnable and can be switched to.
                if let UpdateResult::CanSwitch =
                    unsafe { update_runnable(&mut next_context_guard, cpu_id) }
                {
                    // Store locks for previous and next context and break out from loop
                    // for the switch
                    switch_context_opt = Some((prev_context_guard, next_context_guard));
                    break;
                }
            }
        }
        warn!("context::switch: for loop completed, switch_context_opt={}", switch_context_opt.is_some());
    };

    warn!("context::switch: getting switch_time");
    // Update per-cpu times
    let switch_time = crate::time::monotonic();
    warn!("context::switch: got switch_time={}", switch_time);
    let percpu_nanos = switch_time.saturating_sub(percpu.switch_internals.switch_time.get()) as u64;
    warn!("context::switch: calculated percpu_nanos={}", percpu_nanos);
    let percpu_ms = percpu_nanos / 1_000_000;
    warn!("context::switch: calculated percpu_ms={}", percpu_ms);
    percpu.stats.add_time(percpu_ms);
    warn!("context::switch: added percpu time");
    percpu.switch_internals.switch_time.set(switch_time);
    warn!("context::switch: set switch_time");

    // Switch process states, TSS stack pointer, and store new context ID
    warn!("context::switch: matching switch_context_opt");
    match switch_context_opt {
        Some((mut prev_context_guard, mut next_context_guard)) => {
            warn!("context::switch: Some(...) branch, about to get prev/next context refs");
            // Update context states and prepare for the switch.
            let prev_context = &mut *prev_context_guard;
            let next_context = &mut *next_context_guard;
            warn!("context::switch: got prev/next context refs, prev_id={} next_id={}",
                  prev_context.debug_id, next_context.debug_id);

            // Verbose context switch logging disabled - floods output
            // debug!(
            //     "SCHED: CPU {} switch: ctx {} -> ctx {} (name: {})",
            //     cpu_id.get(),
            //     prev_context.debug_id,
            //     next_context.debug_id,
            //     next_context.name.as_str()
            // );

            // Set the previous context as "not running"
            prev_context.running = false;

            // Set the next context as "running"
            next_context.running = true;
            // Set the CPU ID for the next context
            next_context.cpu_id = Some(cpu_id);

            // Update times
            prev_context.cpu_time += switch_time.saturating_sub(prev_context.switch_time);
            next_context.switch_time = switch_time;
            if next_context.userspace {
                percpu.stats.set_state(cpu_stats::CpuState::User);
            } else {
                percpu.stats.set_state(cpu_stats::CpuState::Kernel);
            }
            unsafe {
                percpu.switch_internals.set_current_context(Arc::clone(
                    ArcContextLockWriteGuard::rwlock(&next_context_guard),
                ));
            }

            // FIXME set the switch result in arch::switch_to instead
            let prev_context = unsafe {
                mem::transmute::<&'_ mut Context, &'_ mut Context>(&mut *prev_context_guard)
            };
            let next_context = unsafe {
                mem::transmute::<&'_ mut Context, &'_ mut Context>(&mut *next_context_guard)
            };

            percpu
                .switch_internals
                .switch_result
                .set(Some(SwitchResultInner {
                    _prev_guard: prev_context_guard,
                    _next_guard: next_context_guard,
                }));

            /*let (ptrace_session, ptrace_flags) = if let Some((session, bp)) = ptrace::sessions()
                .get(&next_context.pid)
                .map(|s| (Arc::downgrade(s), s.data.lock().breakpoint))
            {
                (Some(session), bp.map_or(PtraceFlags::empty(), |f| f.flags))
            } else {
                (None, PtraceFlags::empty())
            };*/
            let ptrace_flags = PtraceFlags::empty();

            //*percpu.ptrace_session.borrow_mut() = ptrace_session;
            percpu.ptrace_flags.set(ptrace_flags);
            prev_context.inside_syscall =
                percpu.inside_syscall.replace(next_context.inside_syscall);

            #[cfg(feature = "syscall_debug")]
            {
                prev_context.syscall_debug_info = percpu
                    .syscall_debug_info
                    .replace(next_context.syscall_debug_info);
                prev_context.syscall_debug_info.on_switch_from();
                next_context.syscall_debug_info.on_switch_to();
            }

            percpu
                .switch_internals
                .being_sigkilled
                .set(next_context.being_sigkilled);

            warn!("context::switch: about to call arch::switch_to (prev={}, next={})", prev_context.debug_id, next_context.debug_id);
            unsafe {
                arch::switch_to(prev_context, next_context);
            }
            warn!("context::switch: returned from arch::switch_to");

            // NOTE: After switch_to is called, the return address can even be different from the
            // current return address, meaning that we cannot use local variables here, and that we
            // need to use the `switch_finish_hook` to be able to release the locks. Newly created
            // contexts will return directly to the function pointer passed to context::spawn, and not
            // reach this code until the next context switch back.
            SwitchResult::Switched
        }
        _ => {
            // No target was found, unset global lock and return
            arch::CONTEXT_SWITCH_LOCK.store(false, Ordering::SeqCst);

            percpu.stats.set_state(cpu_stats::CpuState::Idle);

            SwitchResult::AllContextsIdle
        }
    }
}

/// Holds per-CPU state necessary for context switching.
///
/// This struct contains information such as the idle context, current context, and PIT tick counts,
/// as well as fields required for managing ptrace sessions and signals.
pub struct ContextSwitchPercpu {
    switch_result: Cell<Option<SwitchResultInner>>,
    switch_time: Cell<u128>,
    pit_ticks: Cell<usize>,

    current_ctxt: RefCell<Option<Arc<ContextLock>>>,

    /// The idle process.
    idle_ctxt: RefCell<Option<Arc<ContextLock>>>,

    pub(crate) being_sigkilled: Cell<bool>,
}

impl ContextSwitchPercpu {
    pub const fn default() -> Self {
        Self {
            switch_result: Cell::new(None),
            switch_time: Cell::new(0),
            pit_ticks: Cell::new(0),
            current_ctxt: RefCell::new(None),
            idle_ctxt: RefCell::new(None),
            being_sigkilled: Cell::new(false),
        }
    }

    /// Applies a function to the current context, allowing controlled access.
    ///
    /// # Parameters
    /// - `f`: A closure that receives a reference to the current context and returns a value.
    ///
    /// # Returns
    /// The result of applying `f` to the current context.
    pub fn with_context<T>(&self, f: impl FnOnce(&Arc<ContextLock>) -> T) -> T {
        f(self
            .current_ctxt
            .borrow()
            .as_ref()
            .expect("not inside of context"))
    }

    /// Applies a function to the current context, allowing controlled access.
    ///
    /// # Parameters
    /// - `f`: A closure that receives a reference to the current context and returns a value.
    ///
    /// # Returns
    /// The result of applying `f` to the current context if any.
    pub fn try_with_context<T>(&self, f: impl FnOnce(Option<&Arc<ContextLock>>) -> T) -> T {
        f(self.current_ctxt.borrow().as_ref())
    }

    /// Sets the current context to a new value.
    ///
    /// # Safety
    /// This function is unsafe as it modifies the context state directly.
    ///
    /// # Parameters
    /// - `new`: The new context to be set as the current context.
    pub unsafe fn set_current_context(&self, new: Arc<ContextLock>) {
        *self.current_ctxt.borrow_mut() = Some(new);
    }

    /// Sets the idle context to a new value.
    ///
    /// # Safety
    /// This function is unsafe as it modifies the idle context state directly.
    ///
    /// # Parameters
    /// - `new`: The new context to be set as the idle context.
    pub unsafe fn set_idle_context(&self, new: Arc<ContextLock>) {
        *self.idle_ctxt.borrow_mut() = Some(new);
    }

    /// Retrieves the current idle context.
    ///
    /// # Returns
    /// A reference to the idle context.
    pub fn idle_context(&self) -> Arc<ContextLock> {
        Arc::clone(
            self.idle_ctxt
                .borrow()
                .as_ref()
                .expect("no idle context present"),
        )
    }
}
