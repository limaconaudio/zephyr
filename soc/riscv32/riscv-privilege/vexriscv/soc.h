#ifndef __RISCV32_VEXRISCV_H_
#define __RISCV32_VEXRISCV_H_

#include <soc_common.h>


/* Timer configuration */
#define RISCV_MTIME_BASE             CONFIG_RISCV_MTIME_BASE
#define RISCV_MTIMECMP_BASE          CONFIG_RISCV_MTIMECMP_BASE


/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               CONFIG_RISCV_RAM_BASE_ADDR
#define RISCV_RAM_SIZE               CONFIG_RISCV_RAM_SIZE



#if CONFIG_PLIC == 1
/* PLIC */
#define DT_PLIC_MAX_PRIORITY	\
	DT_RISCV_PLIC0_0_RISCV_MAX_PRIORITY
#define DT_PLIC_PRIO_BASE_ADDR	\
	DT_RISCV_PLIC0_0_PRIO_BASE_ADDRESS
#define DT_PLIC_IRQ_EN_BASE_ADDR \
	DT_RISCV_PLIC0_0_IRQ_EN_BASE_ADDRESS
#define DT_PLIC_REG_BASE_ADDR	\
	DT_RISCV_PLIC0_0_REG_BASE_ADDRESS
#endif

#ifdef CONFIG_SOC_VEXRISCV_SAXON_SOC
#include "soc_saxon.h"
#endif


#endif /* __RISCV32_VEXRISCV_H_ */
