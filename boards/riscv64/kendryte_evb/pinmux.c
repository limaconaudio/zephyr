/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <pinmux.h>
#include <board.h>

#include <pinmux/pinmux_kendryte.h>

static int kendryte_pinmux_init(struct device *dev)
{
	ARG_UNUSED(dev);

	struct device *p = device_get_binding(CONFIG_KENDRYTE_PINMUX_NAME);

	/* UART3 RX */
	pinmux_pin_set(p, 4, FUNC_UART3_RX);
	/* UART3 TX */
	pinmux_pin_set(p, 5, FUNC_UART3_TX);
	pinmux_pin_set(p, 24, FUNC_GPIOHS3);


	return 0;
}

SYS_INIT(kendryte_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
