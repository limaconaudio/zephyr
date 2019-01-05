/*
 * Copyright (c) 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>

static void uart_spinal_lib_com_uart_uartCtrl_poll_out(struct device *dev, unsigned char c)
{
	volatile int *uart = (int*)CONFIG_UART_BASE;
    while((uart[1] & 0xFFFF0000) == 0);
    uart[0] = c;
}

static int uart_spinal_lib_com_uart_uartCtrl_poll_in(struct device *dev, unsigned char *c)
{
	return -1;
}

static int uart_spinal_lib_com_uart_uartCtrl_init(struct device *dev)
{
	return 0;
}


static const struct uart_driver_api uart_spinal_lib_com_uart_uartCtrl_driver_api = {
	.poll_in = uart_spinal_lib_com_uart_uartCtrl_poll_in,
	.poll_out = uart_spinal_lib_com_uart_uartCtrl_poll_out,
	.err_check = NULL,
};



DEVICE_AND_API_INIT(uart_spinal_lib_com_uart_uartCtrl_0, "uart_0",
		    uart_spinal_lib_com_uart_uartCtrl_init, NULL,
		    NULL,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_spinal_lib_com_uart_uartCtrl_driver_api);


