/*
 * Copyright (c) 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>

struct uart_spinal_lib_com_uart_uartCtrl_regs{
	u32_t data;
	u32_t status;
};


struct uart_spinal_lib_com_uart_uartCtrl_device_config {
	u32_t base_address;
};


#define DEV_CFG(dev)						\
	((const struct uart_spinal_lib_com_uart_uartCtrl_device_config * const)	\
	 (dev)->config->config_info)

#define DEV_UART(dev)						\
	((struct uart_spinal_lib_com_uart_uartCtrl_regs *)(DEV_CFG(dev))->base_address)

static void uart_spinal_lib_com_uart_uartCtrl_poll_out(struct device *dev, unsigned char c)
{
	volatile struct uart_spinal_lib_com_uart_uartCtrl_regs *uart = DEV_UART(dev);
    while((uart->status & 0xFFFF0000) == 0);
    uart->data = c;
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


static const struct uart_spinal_lib_com_uart_uartCtrl_device_config uart_spinal_lib_com_uart_uartCtrl_dev_cfg_0 = {
	.base_address = DT_SPINAL_LIB_COM_UART_UARTCTRL_0_BASE_ADDRESS
};

DEVICE_AND_API_INIT(uart_spinal_lib_com_uart_uartCtrl_0, "uart0",
		    uart_spinal_lib_com_uart_uartCtrl_init, NULL,
		    &uart_spinal_lib_com_uart_uartCtrl_dev_cfg_0,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_spinal_lib_com_uart_uartCtrl_driver_api);


