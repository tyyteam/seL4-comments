/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2015, 2016 Hesham Almatary <heshamelmatary@gmail.com>
 * Copyright 2021, HENSOLDT Cyber
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <config.h>
#include <types.h>
#include <machine/registerset.h>
#include <machine/timer.h>
#include <arch/machine.h>
#include <arch/smp/ipi.h>

#ifndef CONFIG_KERNEL_MCS
#define RESET_CYCLES ((TIMER_CLOCK_HZ / MS_IN_S) * CONFIG_TIMER_TICK_MS)
#endif /* !CONFIG_KERNEL_MCS */

#define IS_IRQ_VALID(X) (((X)) <= maxIRQ && (X) != irqInvalid)

word_t PURE getRestartPC(tcb_t *thread)
{
    return getRegister(thread, FaultIP);
}

void setNextPC(tcb_t *thread, word_t v)
{
    setRegister(thread, NextIP, v);
}

BOOT_CODE void map_kernel_devices(void)
{
    /* If there are no kernel device frames at all, then kernel_device_frames is
     * NULL. Thus we can't use ARRAY_SIZE(kernel_device_frames) here directly,
     * but have to use NUM_KERNEL_DEVICE_FRAMES that is defined accordingly.
     */
    for (int i = 0; i < NUM_KERNEL_DEVICE_FRAMES; i++) {
        const kernel_frame_t *frame = &kernel_device_frames[i];
        map_kernel_frame(frame->paddr, frame->pptr, VMKernelOnly);
        if (!frame->userAvailable) {
            reserve_region((p_region_t) {
                .start = frame->paddr,
                .end   = frame->paddr + BIT(seL4_LargePageBits)
            });
        }
    }
}

/*
 * The following assumes familiarity with RISC-V interrupt delivery and the PLIC.
 * See the RISC-V privileged specification v1.10 and the comment in
 * include/plat/spike/plat/machine.h for more information.
 * RISC-V IRQ handling on seL4 works as follows:
 *
 * On other architectures the kernel masks interrupts between delivering them to
 * userlevel and receiving the acknowledgment invocation. This strategy doesn't
 * work on RISC-V as an IRQ is implicitly masked when it is claimed, until the
 * claim is acknowledged. If we mask and unmask the interrupt at the PLIC while
 * a claim is in progress we sometimes experience IRQ sources not being masked
 * and unmasked as expected. Because of this, we don't mask and unmask IRQs that
 * are for user level, and also call plic_complete_claim for seL4_IRQHandler_Ack.
 */

/* QT 在其他架构上，内核在将中断传递到用户级和接收确认调用之间屏蔽中断。
    此策略不适用于 RISC-V，因为 IRQ 在声明时被隐式屏蔽，直到声明被确认。
    如果我们在声明时屏蔽和取消屏蔽 PLIC 的中断，我们有时会遇到 IRQ 源未按预期屏蔽和取消屏蔽。
    因此，我们不会屏蔽和取消屏蔽用户级别的 IRQ，并且还会为 seL4_IRQHandler_Ack 调用 plic_complete_claim。
*/

static irq_t active_irq[CONFIG_MAX_NUM_NODES];//QT 如果不启用smp，CONFIG_MAX_NUM_NODES=1


/**
 * Gets the active irq. Returns the same irq if called again before ackInterrupt.
 *
 * This function is called by the kernel to get the interrupt that is currently
 * active. It not interrupt is currently active, it will try to find one and
 * put it in the active state. If no interrupt is found, irqInvalid is returned.
 * It can't be assumed that if isIRQPending() returned true, there will always
 * be an active interrupt then this is called. It may hold in mayn cases, but
 * there are corner cases with level-triggered interrupts or on multicore
 * systems.
 * This function can be called multiple times during one kernel entry. It must
 * guarantee that once one interrupt is reported as active, this interrupt is
 * always returned until ackInterrupt() is called eventually.
 *
 * @return     The active irq or irqInvalid.
 */
/*QT 该函数被内核调用，用于获取现在活跃的中断。如果没有活跃的中断，它会尝试找到一个中断，并设置为活跃状态。
如果没有找到中断，返回irqInvalid。如果isIRQPending()返回true，调用该函数时，不一定存在活跃irq。
这可能在多种情况下都成立，但是在电平触发中断或多核系统上存在一些极端情况。
在一个内核入口期间可以多次调用此函数。 它必须保证一旦一个中断被报告为活动的，
这个中断总是被返回，直到最终调用 ackInterrupt()。
*/

static inline irq_t getActiveIRQ(void)
{
    irq_t *active_irq_slot = &active_irq[CURRENT_CPU_INDEX()];

    /* If an interrupt is currently active, then return it. */
    irq_t irq = *active_irq_slot;
    if (IS_IRQ_VALID(irq)) {
        return irq;
    }

    /* No interrupt currently active, find a new one from the sources. The
     * priorities are: external -> software -> timer.
     */
    word_t sip = read_sip();
    if (sip & BIT(SIP_SEIP)) {
        /* Even if we say an external interrupt is pending, the PLIC may not
         * return any pending interrupt here in some corner cases. A level
         * triggered interrupt might have been deasserted again or another hard
         * has claimed it in a multicore system.
         */
        irq = plic_get_claim();
#ifdef ENABLE_SMP_SUPPORT
    } else if (sip & BIT(SIP_SSIP)) {
        sbi_clear_ipi();
        irq = ipi_get_irq();
#endif
    } else if (sip & BIT(SIP_STIP)) {
        irq = KERNEL_TIMER_IRQ;
    } else {
        /* Seems none of the known sources has a pending interrupt. This can
         * happen if e.g. if another hart context has claimed the interrupt
         * already.
         */
        irq = irqInvalid;
    }

    /* There is no guarantee that there is a new interrupt. */
    if (!IS_IRQ_VALID(irq)) {
        /* Sanity check: the slot can't hold an interrupt either. */
        assert(!IS_IRQ_VALID(*active_irq_slot));
        return irqInvalid;
    }

    /* A new interrupt is active, remember it. */
    *active_irq_slot = irq;
    return irq;
}

#ifdef HAVE_SET_TRIGGER
/**
 * Sets the irq trigger.
 *
 * setIRQTrigger can change the trigger between edge and level at the PLIC for
 * external interrupts. It is implementation specific as whether the PLIC has
 * support for this operation.
 *
 * @param[in]  irq             The irq
 * @param[in]  edge_triggered  edge triggered otherwise level triggered
 */
void setIRQTrigger(irq_t irq, bool_t edge_triggered)
{
    plic_irq_set_trigger(irq, edge_triggered);
}
#endif

/* isIRQPending is used to determine whether to preempt long running
 * operations at various preemption points throughout the kernel. If this
 * returns true, it means that if the Kernel were to return to user mode, it
 * would then immediately take an interrupt. We check the SIP register for if
 * either a timer interrupt (STIP) or an external interrupt (SEIP) is pending.
 * We don't check software generated interrupts. These are used to perform cross
 * core signalling which isn't currently supported.
 * TODO: Add SSIP check when SMP support is added.
 */
static inline bool_t isIRQPending(void)
{
    word_t sip = read_sip();
    return (sip & (BIT(SIP_STIP) | BIT(SIP_SEIP)));
}

/**
 * Disable or enable IRQs.
 *
 * maskInterrupt disables and enables IRQs. When an IRQ is disabled, it should
 * not raise an interrupt on the Kernel's HART context. This either masks the
 * core timer on the sie register or masks an external IRQ at the plic.
 *
 * @param[in]  disable  The disable
 * @param[in]  irq      The irq
 */
static inline void maskInterrupt(bool_t disable, irq_t irq)
{
    assert(IS_IRQ_VALID(irq));
    if (irq == KERNEL_TIMER_IRQ) {//如果是内核时钟中断
        if (disable) {//如果是屏蔽命令
            clear_sie_mask(BIT(SIE_STIE));//关闭时钟中断
        } else {
            set_sie_mask(BIT(SIE_STIE));//使能时钟中断
        }
#ifdef ENABLE_SMP_SUPPORT
    } else if (irq == irq_reschedule_ipi || irq == irq_remote_call_ipi) {
        return;
#endif
    } else {//如果是plic中断
        plic_mask_irq(disable, irq);//会提示无plic,无法关闭或使能中断
    }
}

/**
 * Kernel has dealt with the pending interrupt getActiveIRQ can return next IRQ.
 *
 * ackInterrupt is used by the kernel to indicate it has processed the interrupt
 * delivery and getActiveIRQ is now able to return a different IRQ number. Note
 * that this is called after a notification has been signalled to user level,
 * but before user level has handled the cause.
 *
 * @param[in]  irq   The irq
 */
static inline void ackInterrupt(irq_t irq)
{
    assert(IS_IRQ_VALID(irq));
    active_irq[CURRENT_CPU_INDEX()] = irqInvalid;

    if (irq == KERNEL_TIMER_IRQ) {
        /* Reprogramming the timer has cleared the interrupt. */
        return;
    }
#ifdef ENABLE_SMP_SUPPORT
    if (irq == irq_reschedule_ipi || irq == irq_remote_call_ipi) {
        ipi_clear_irq(irq);
    }
#endif
}

#ifndef CONFIG_KERNEL_MCS
void resetTimer(void)
{
    uint64_t target;
    // repeatedly try and set the timer in a loop as otherwise there is a race and we
    // may set a timeout in the past, resulting in it never getting triggered
    do {
        target = riscv_read_time() + RESET_CYCLES;
        sbi_set_timer(target);
    } while (riscv_read_time() > target);
}

/**
   DONT_TRANSLATE
 */
BOOT_CODE void initTimer(void)
{
    sbi_set_timer(riscv_read_time() + RESET_CYCLES);
}
#endif /* !CONFIG_KERNEL_MCS */

BOOT_CODE void initLocalIRQController(void)
{
    printf("Init local IRQ\n");

    /* Init per-hart PLIC */
    /*QT 从调试信息来看，只是在include/drivers/irq/riscv_plic_dummy.h里打印一句话。整个文件是dummy处理，因为spike没有实现。*/
    plic_init_hart();

    /* Enable timer and external interrupt. If SMP is enabled, then enable the
     * software interrupt also, it is used as IPI between cores. */
    /*CY sie是当前的中断使能位 */
    /*QT 使能中断位，中断号见手册图10.3：s模式外部中断9、s模式时间中断5。未开smp,s模式software中断==0*/
    set_sie_mask(BIT(SIE_SEIE) | BIT(SIE_STIE) | SMP_TERNARY(BIT(SIE_SSIE), 0));
}

BOOT_CODE void initIRQController(void)
{
    printf("Initializing PLIC...\n");

    /* Initialize active_irq[] properly to stick to the semantics and play safe.
     * Effectively this is not needed if irqInvalid is zero (which is currently
     * the case) and the array is in the BSS, that is filled with zeros (which
     * the a kernel loader is supposed to do and which the ELF-Loader does).
     */
    /*QT 正确初始化active_irq[],遵守语义，确保安全。
         通常情况下，active_irq数组保存在bss段中，irqInvalid初值就是0，此处因此通常不需要这样初始化。
         irqInvalid在build文件夹下定义是0，见build-spike/kernel/gen_headers/plat/platform_gen.h
         此处应该是将活跃irq初始化为非法值。
    */
    for (word_t i = 0; i < ARRAY_SIZE(active_irq); i++) {
        active_irq[i] = irqInvalid;
    }

    plic_init_controller();/*QT include/drivers/irq/riscv_plic_dummy.h 无plic*/
}

static inline void handleSpuriousIRQ(void)
{
    /* Do nothing */
    printf("Superior IRQ!! SIP %lx\n", read_sip());
}