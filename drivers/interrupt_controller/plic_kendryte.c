/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 * Contributors: 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Platform Level Interrupt Controller (PLIC) driver
 *        for RISC-V processors
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <init.h>
#include "plic_kendryte.h"
#include <sw_isr_table.h>

static volatile plic_t* const plic = (volatile plic_t*)PLIC_BASE_ADDRESS;

static int save_irq;

/**
 *
 * @brief Enable a riscv PLIC-specific interrupt line
 *
 * This routine enables a RISCV PLIC-specific interrupt line.
 * riscv_plic_irq_enable is called by SOC_FAMILY_RISCV_PRIVILEGE
 * _arch_irq_enable function to enable external interrupts for
 * IRQS > RISCV_MAX_GENERIC_IRQ, whenever CONFIG_RISCV_HAS_PLIC
 * variable is set.
 * @param irq IRQ number to enable
 *
 * @return N/A
 */
void riscv_plic_irq_enable(u32_t irq)
{
	u32_t key;
	u32_t plic_irq = irq - RISCV_MAX_GENERIC_IRQ;

	key = irq_lock();
	
	unsigned long core_id = current_coreid();
	/* Get current enable bit array by IRQ number */
	uint32_t current = plic->target_enables.target[core_id].enable[plic_irq / 32];
	/* Set enable bit in enable bit array */
	current |= (uint32_t)1 << (plic_irq % 32);
	/* Write back the enable bit array */
	plic->target_enables.target[core_id].enable[plic_irq / 32] = current;
	
	irq_unlock(key);
}

/**
 *
 * @brief Disable a riscv PLIC-specific interrupt line
 *
 * This routine disables a RISCV PLIC-specific interrupt line.
 * riscv_plic_irq_disable is called by SOC_FAMILY_RISCV_PRIVILEGE
 * _arch_irq_disable function to disable external interrupts, for
 * IRQS > RISCV_MAX_GENERIC_IRQ, whenever CONFIG_RISCV_HAS_PLIC
 * variable is set.
 * @param irq IRQ number to disable
 *
 * @return N/A
 */
void riscv_plic_irq_disable(u32_t irq)
{
	u32_t key;
	u32_t plic_irq = irq - RISCV_MAX_GENERIC_IRQ;

	key = irq_lock();

	unsigned long core_id = current_coreid();
	/* Get current enable bit array by IRQ number */
	uint32_t current = plic->target_enables.target[core_id].enable[plic_irq / 32];
	/* Clear enable bit in enable bit array */
	current &= ~((uint32_t)1 << (plic_irq % 32));
	/* Write back the enable bit array */
	plic->target_enables.target[core_id].enable[plic_irq / 32] = current;
	
	irq_unlock(key);
}

/**
 *
 * @brief Check if a riscv PLIC-specific interrupt line is enabled
 *
 * This routine checks if a RISCV PLIC-specific interrupt line is enabled.
 * @param irq IRQ number to check
 *
 * @return 1 or 0
 */
int riscv_plic_irq_is_enabled(u32_t irq)
{
	u32_t plic_irq = irq - RISCV_MAX_GENERIC_IRQ;

	unsigned long core_id = current_coreid();
	/* Get current enable bit array by IRQ number */
	uint32_t current = plic->target_enables.target[core_id].enable[plic_irq / 32];
	return !!(current & ((uint32_t)1 << (plic_irq % 32)));
}

/**
 *
 * @brief Set priority of a riscv PLIC-specific interrupt line
 *
 * This routine set the priority of a RISCV PLIC-specific interrupt line.
 * riscv_plic_irq_set_prio is called by riscv32 _ARCH_IRQ_CONNECT to set
 * the priority of an interrupt whenever CONFIG_RISCV_HAS_PLIC variable is set.
 * @param irq IRQ number for which to set priority
 *
 * @return N/A
 */
void riscv_plic_set_priority(u32_t irq, u32_t priority)
{
	/* Can set priority only for PLIC-specific interrupt line */
	if (irq <= RISCV_MAX_GENERIC_IRQ)
		return;

	if (priority > PLIC_MAX_PRIORITY)
		priority = PLIC_MAX_PRIORITY;

	/* Set interrupt priority by IRQ number */
	plic->source_priorities.priority[irq] = priority;
}

/**
 *
 * @brief Get riscv PLIC-specific interrupt line causing an interrupt
 *
 * This routine returns the RISCV PLIC-specific interrupt line causing an
 * interrupt.
 * @param irq IRQ number for which to set priority
 *
 * @return N/A
 */
int riscv_plic_get_irq(void)
{
	return save_irq;
}

static void plic_irq_handler(void *arg)
{
	struct _isr_table_entry *ite;

        /* Get current core id */
        uint64_t core_id = current_coreid();
        /* Get primitive interrupt enable flag */
        uint64_t ie_flag = read_csr(mie);
        /* Get current IRQ num */
        uint32_t irq = plic->targets.target[core_id].claim_complete;

	/*
	 * If the IRQ is out of range, call _irq_spurious.
	 * A call to _irq_spurious will not return.
	 */
	if (irq == 0 || irq >= PLIC_IRQS)
		_irq_spurious(NULL);

        /* Get primitive IRQ threshold */
        uint32_t int_threshold = plic->targets.target[core_id].priority_threshold;
        /* Set new IRQ threshold = current IRQ threshold */
        plic->targets.target[core_id].priority_threshold =
		plic->source_priorities.priority[irq];
        /* Disable software interrupt and timer interrupt */
        clear_csr(mie, MIP_MTIP | MIP_MSIP);
        /* Enable global interrupt */
        set_csr(mstatus, MSTATUS_MIE);

	/* Call the corresponding IRQ handler in _sw_isr_table */
	ite = (struct _isr_table_entry *)&_sw_isr_table[irq + RISCV_MAX_GENERIC_IRQ];
	ite->isr(ite->arg);

	save_irq = irq;

        /* Perform IRQ complete */
        plic->targets.target[core_id].claim_complete = irq;
        /* Disable global interrupt */
        clear_csr(mstatus, MSTATUS_MIE);
        /* Set MPIE and MPP flag used to MRET instructions restore MIE flag */
        set_csr(mstatus, MSTATUS_MPIE | MSTATUS_MPP);
        /* Restore primitive interrupt enable flag */
        write_csr(mie, ie_flag);
        /* Restore primitive IRQ threshold */
        plic->targets.target[core_id].priority_threshold = int_threshold;
}

/**
 *
 * @brief Initialize the Platform Level Interrupt Controller
 * @return N/A
 */
static int plic_init(struct device *dev)
{
	ARG_UNUSED(dev);
	int i;

	/* Get current core id */
	unsigned long core_id = current_coreid();

	/* Disable all interrupts for the current core. */
	for (i = 0; i < ((PLIC_NUM_SOURCES + 32u) / 32u); i++)
		plic->target_enables.target[core_id].enable[i] = 0;

	/* Set priorities to zero. */
	for (i = 0; i < PLIC_NUM_SOURCES; i++)
		plic->source_priorities.priority[i] = 0;

	/* Set the threshold to zero. */
	plic->targets.target[core_id].priority_threshold = 0;

	/*
	* A successful claim will also atomically clear the corresponding
	* * pending bit on the interrupt source. A target can perform a claim
	* at any time, even if the EIP is not set.
	*/
	i = 0;
	while (plic->targets.target[core_id].claim_complete > 0 && i < 100) {
		/* This loop will clear pending bit on the interrupt source */
		i++;
	}

	/* Setup IRQ handler for PLIC driver */
	IRQ_CONNECT(RISCV_MACHINE_EXT_IRQ,
		    0,
		    plic_irq_handler,
		    NULL,
		    0);

	/* Enable IRQ for PLIC driver */
	irq_enable(RISCV_MACHINE_EXT_IRQ);

	return 0;
}

SYS_INIT(plic_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
