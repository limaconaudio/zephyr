#include <autoconf.h>

#ifdef CONFIG_SOC_VEXRISCV_SAXON_SOC
#include "saxon.ld"
#else
#include <arch/riscv32/common/linker.ld>
#endif