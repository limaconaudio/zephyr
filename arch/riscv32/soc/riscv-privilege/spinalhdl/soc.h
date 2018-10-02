/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the SPINALHDL SoC
 */

#ifndef __RISCV32_SPINALHDL_SOC_H_
#define __RISCV32_SPINALHDL_SOC_H_

#include <soc_common.h>

#define SPINALHDL_FCLK_RATE	         50000000

/* Platform Level Interrupt Controller (PLIC) interrupt sources */

/* PLIC Interrupt Sources */
#define SPINALHDL_UART_IRQ           (RISCV_MAX_GENERIC_IRQ + 1)
#define SPINALHDL_GPIO_IRQ           (RISCV_MAX_GENERIC_IRQ + 2)
#define SPINALHDL_SPI_IRQ            (RISCV_MAX_GENERIC_IRQ + 3)
#define SPINALHDL_I2C_IRQ            (RISCV_MAX_GENERIC_IRQ + 4)

/* UART Configuration */
#define SPINALHDL_UART_BASE_ADDR      0x00030000
#define SPINALHDL_UART_BAUDRATE       115200

/* GPIO Configuration */
#define SPINALHDL_GPIO_0_BASE_ADDR    0x00000000

/* Platform Level Interrupt Controller Configuration */
#define SPINALHDL_PLIC_BASE_ADDR		  0x00060000
#define SPINALHDL_PLIC_REG_PRI	(SPINALHDL_PLIC_BASE_ADDR + 0x0C)
#define SPINALHDL_PLIC_REG_IRQ_EN	(SPINALHDL_PLIC_BASE_ADDR + 0x10)
#define SPINALHDL_PLIC_REG_THRES	(SPINALHDL_PLIC_BASE_ADDR + 0x14)
#define SPINALHDL_PLIC_REG_ID	(SPINALHDL_PLIC_BASE_ADDR + 0x18)

#define SPINALHDL_PLIC_MAX_PRIORITY	7

/* Timer configuration */
#define RISCV_MTIME_BASE               0x0005000C
#define RISCV_MTIMECMP_BASE            0x00050004

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE                 CONFIG_RISCV_RAM_BASE_ADDR
#define RISCV_RAM_SIZE                 CONFIG_RISCV_RAM_SIZE

#endif /* __RISCV32_SPINALHDL_SOC_H_ */
