/*
 * Copyright (c) 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>
#include <serial/serial_spinal_lib_io.h>

#define DEV_CFG(dev)						\
	((const struct uart_spinal_lib_com_device_config * const)	\
	 (dev)->config->config_info)

#define DEV_UART(dev)						\
	((struct uart_spinal_lib_com_regs *)(DEV_CFG(dev))->base_address)

static void uart_spinal_lib_com_poll_out(struct device *dev, unsigned char c)
{
	volatile struct uart_spinal_lib_com_regs *uart = DEV_UART(dev);
    while((uart->status & 0xFFFF0000) == 0);
    uart->data = c;
}

static int uart_spinal_lib_com_poll_in(struct device *dev, unsigned char *c)
{
	return -1;
}

int uart_spinal_lib_com_init(struct device *dev)
{
	return 0;
}


const struct uart_driver_api uart_spinal_lib_com_driver_api = {
	.poll_in = uart_spinal_lib_com_poll_in,
	.poll_out = uart_spinal_lib_com_poll_out,
	.err_check = NULL,
};


