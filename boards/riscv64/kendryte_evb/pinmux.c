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
	pinmux_pin_set(p, 24, FUNC_SPI0_SS3);
	pinmux_pin_set(p, 25, FUNC_GPIO4);
	/* SPI0 */
	pinmux_pin_set(p, 29, FUNC_SPI0_SCLK);
	pinmux_pin_set(p, 30, FUNC_SPI0_D0);
	pinmux_pin_set(p, 31, FUNC_SPI0_D1);
	pinmux_pin_set(p, 32, FUNC_GPIOHS7);

	/* I2S0 */
	pinmux_pin_set(p, 33, FUNC_I2S0_OUT_D1);
	pinmux_pin_set(p, 35, FUNC_I2S0_SCLK);
	pinmux_pin_set(p, 34, FUNC_I2S0_WS);

	return 0;
}

SYS_INIT(kendryte_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
