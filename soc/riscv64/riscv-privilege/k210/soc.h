/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the SiFive Freedom processor
 */

#ifndef __RISCV64_K210_SOC_H_
#define __RISCV64_K210_SOC_H_

#include <soc_common.h>

/* Platform Level Interrupt Controller Configuration */
#define PLIC_PRIO_BASE_ADDR    PLIC_BASE_ADDRESS
#define PLIC_IRQ_EN_BASE_ADDR  (PLIC_BASE_ADDRESS + 0x2000)
#define PLIC_REG_BASE_ADDR     (PLIC_BASE_ADDRESS + 0x200000)

#define PLIC_MAX_PRIORITY      PLIC_RISCV_MAX_PRIORITY

/* Clock controller. */
#define SYSCTL_BASE_ADDR	0x50440000

/* Timer configuration */
#define RISCV_MTIME_BASE	0x0200BFF8
#define RISCV_MTIMECMP_BASE	0x02004000

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               CONFIG_RISCV_RAM_BASE_ADDR
#define RISCV_RAM_SIZE               CONFIG_RISCV_RAM_SIZE

#endif /* __RISCV64_K210_SOC_H_ */
