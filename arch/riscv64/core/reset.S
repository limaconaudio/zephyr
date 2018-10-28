/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 * Contributors: 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel_structs.h>

/* exports */
GTEXT(__initialize)
GTEXT(__reset)

/* imports */
GTEXT(_PrepC)
GDATA(__global_pointer$)

#if CONFIG_INCLUDE_RESET_VECTOR
SECTION_FUNC(reset, __reset)
	/*
	 * jump to __initialize
	 * use call opcode in case __initialize is far away.
	 * This will be dependent on linker.ld configuration.
	 */
	call __initialize
#endif /* CONFIG_INCLUDE_RESET_VECTOR */

/* use ABI name of registers for the sake of simplicity */

/*
 * Remainder of asm-land initialization code before we can jump into
 * the C domain
 */
SECTION_FUNC(TEXT, __initialize)
	li x1, 0
	li x2, 0
	li x3, 0
	li x4, 0
	li x5, 0
	li x6, 0
	li x7, 0
	li x8, 0
	li x9, 0
	li x10, 0
	li x11, 0
	li x12, 0
	li x13, 0
	li x14, 0
	li x15, 0
	li x16, 0
	li x17, 0
	li x18, 0
	li x19, 0
	li x20, 0
	li x21, 0
	li x22, 0
	li x23, 0
	li x24, 0
	li x25, 0
	li x26, 0
	li x27, 0
	li x28, 0
	li x29, 0
	li x30, 0
	li x31, 0
	csrw mscratch, x0

	/* Reload the global pointer */
	la gp, __global_pointer$
	/*
	 * Use HART with ID zero to execute the init code and put rest
	 * of the HART's to spin until we handle SMP
	 */
	csrr t0, mhartid
	bnez t0, secondary_boot

#ifdef CONFIG_INIT_STACKS
	/* Pre-populate all bytes in _interrupt_stack with 0xAA */
	la t0, _interrupt_stack
	li t1, CONFIG_ISR_STACK_SIZE
	add t1, t1, t0

	/* Populate _interrupt_stack with 0xaaaaaaaaaaaaaaaa */
	li t2, 0xaaaaaaaaaaaaaaaa
aa_loop:
	sw t2, 0x00(t0)
	addi t0, t0, 8
	blt t0, t1, aa_loop
#endif

	/*
	 * Initially, setup stack pointer to
	 * _interrupt_stack + CONFIG_ISR_STACK_SIZE
	 */
	la sp, _interrupt_stack
	li t0, CONFIG_ISR_STACK_SIZE
	add sp, sp, t0

	/*
	 * Jump into C domain. _PrepC zeroes BSS, copies rw data into RAM,
	 * and then enters kernel _Cstart
	 */
	call _PrepC

secondary_boot:
	addi t0, x0, 0
	beqz t0, secondary_boot
