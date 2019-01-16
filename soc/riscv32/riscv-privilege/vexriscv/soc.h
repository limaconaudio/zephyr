#ifndef __RISCV32_VEXRISCV_H_
#define __RISCV32_VEXRISCV_H_

#include <soc_common.h>


/* Timer configuration */
#define RISCV_MTIME_BASE             CONFIG_RISCV_MTIME_BASE
#define RISCV_MTIMECMP_BASE          CONFIG_RISCV_MTIMECMP_BASE


/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               CONFIG_RISCV_RAM_BASE_ADDR
#define RISCV_RAM_SIZE               CONFIG_RISCV_RAM_SIZE

#endif /* __RISCV32_VEXRISCV_H_ */
