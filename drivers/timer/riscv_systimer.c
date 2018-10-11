#include <nanokernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <drivers/system_timer.h>
#include <soc.h>
#include <memaccess.h>

/* running total of timer count */
static uint32_t __noinit cycles_per_tick;
static uint32_t accumulated_cycle_count;

#define GPTIMER_CONTROL_COUNT_ENA   0x1
#define GPTIMER_CONTROL_IRQ_ENA     0x2

/**
 *
 * @brief Read the platform's timer hardware
 *
 * This routine returns the current time in terms of timer hardware clock
 * cycles.
 *
 * @return up counter of elapsed clock cycles
 *
 * \INTERNAL WARNING
 * If this routine is ever enhanced to return all 64 bits of the counter
 * it will need to call _hpetMainCounterAtomic().
 */

uint32_t k_cycle_get_32(void)
{
    //return (uint32_t)READ64(&__TIMERS->tmr[CFG_SYS_TIMER_IDX].cur_value);
	return (uint32_t)READ64(&__TIMERS->highcnt);
}

uint64_t k_cycle_get_64(void)
{
	return READ64(&__TIMERS->highcnt);
}

#if defined(CONFIG_SYSTEM_CLOCK_DISABLE)
/**
 *
 * @brief Stop announcing ticks into the kernel
 *
 * This routine disables timer interrupt generation and delivery.
 * Note that the timer's counting cannot be stopped by software.
 *
 * @return N/A
 */
void sys_clock_disable(void)
{
	unsigned int key;  /* interrupt lock level */
	uint32_t control; /* timer control register value */

	key = irq_lock();

	/* disable interrupt generation */

	control = timer0_control_register_get();
	timer0_control_register_set(control & ~_ARC_V2_TMR_CTRL_IE);

	irq_unlock(key);

	/* disable interrupt in the interrupt controller */

	irq_disable(CONFIG_ARCV2_TIMER0_INT_LVL);
}
#endif /* CONFIG_SYSTEM_CLOCK_DISABLE */


#ifdef CONFIG_TICKLESS_IDLE
static INLINE void update_accumulated_count(void)
{
	accumulated_cycle_count += (_sys_idle_elapsed_ticks * cycles_per_tick);
}
#else /* CONFIG_TICKLESS_IDLE */
static INLINE void update_accumulated_count(void)
{
	accumulated_cycle_count += cycles_per_tick;
}
#endif /* CONFIG_TICKLESS_IDLE */

/**
 *
 * @brief System clock periodic tick handler
 *
 * This routine handles the system clock periodic tick interrupt. It always
 * announces one tick.
 *
 * @return N/A
 */
void _timer_int_handler(void *unused)
{
	ARG_UNUSED(unused);

	/* clear the interrupt by writing 0 to IP bit of the control register */
    WRITE32(&__IRQCTRL->irq_clear, (1u << CFG_IRQ_SYS_TIMER));
    WRITE32(&__TIMERS->pending, 0);

#if defined(CONFIG_TICKLESS_IDLE)
	timer0_limit_register_set(cycles_per_tick - 1);

	_sys_idle_elapsed_ticks = 1;
#endif

	update_accumulated_count();
	_sys_clock_tick_announce();
}

/*
 * @brief initialize the tickless idle feature
 *
 * This routine initializes the tickless idle feature.
 *
 * @return N/A
 */
static void tickless_idle_init(void) {}

int _sys_clock_driver_init(struct device *device) {
	ARG_UNUSED(device);

	/* ensure that the timer will not generate interrupts */
    WRITE32(&__TIMERS->tmr[CFG_SYS_TIMER_IDX].control, 0);

    sys_clock_hw_cycles_per_tick = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC /
                                    sys_clock_ticks_per_sec;
	cycles_per_tick = sys_clock_hw_cycles_per_tick;
    accumulated_cycle_count = 0;

	IRQ_CONNECT(CFG_IRQ_SYS_TIMER, 0, _timer_int_handler, 0, 0);

	/*
	 * Set the reload value to achieve the configured tick rate, enable the
	 * counter and interrupt generation.
	 */

	tickless_idle_init();

    WRITE64(&__TIMERS->tmr[CFG_SYS_TIMER_IDX].init_value, cycles_per_tick - 1);
    WRITE32(&__TIMERS->tmr[CFG_SYS_TIMER_IDX].control, 
        GPTIMER_CONTROL_COUNT_ENA | GPTIMER_CONTROL_IRQ_ENA);

	/* everything has been configured: safe to enable the interrupt */

	irq_enable(CFG_IRQ_SYS_TIMER);

    return 0;
}
