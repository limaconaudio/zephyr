/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief UART driver for the SpinalHDL SoC
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>
#include <board.h>

#define RXDATA_MASK    	0xFF       /* Receive Data Mask */
#define RX_FULL    	(1 << 8)   /* Receive FIFO Full */
#define RX_EMPTY	(1 << 9)   /* Receive FIFO Empty */
#define RX_ENB    	(1 << 10)  /* Receive Enable */

#define TXDATA_MASK    	0xFF       /* Transmit Data Mask */
#define TX_FULL    	(1 << 8)   /* Transmit FIFO Full */
#define TX_EMPTY	(1 << 9)   /* Transmit FIFO Empty */
#define TX_ENB    	(1 << 10)  /* Transmit Enable */

struct uart_spinalhdl_regs_t {
	u32_t tx;
	u32_t rx;
};

#define DEV_CFG(dev)                                            \
        ((const struct uart_device_config * const)	        \
         (dev)->config->config_info)
#define DEV_UART(dev)                                           \
        ((struct uart_spinalhdl_regs_t *)(DEV_CFG(dev))->base)

/**
 * @brief Output a character in polled mode.
 *
 * Writes data to tx register if transmitter is not full.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static unsigned char uart_spinalhdl_poll_out(struct device *dev,
					 unsigned char c)
{
	volatile struct uart_spinalhdl_regs_t *uart = DEV_UART(dev);
	
	/* Wait till TX_FIFO gets empty */
	while (uart->tx & TX_FULL);

	uart->tx = (int)c;

	return c;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_spinalhdl_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct uart_spinalhdl_regs_t *uart = DEV_UART(dev);
	
	u32_t val = uart->rx;

	if (val & RX_EMPTY)
		return -1;

	*c = (unsigned char)(val & RXDATA_MASK);

	return 0;
}

static int uart_spinalhdl_init(struct device *dev)
{
	volatile struct uart_spinalhdl_regs_t *uart = DEV_UART(dev);

	/* Enable TX and RX channels */
	uart->tx |= TX_ENB;
	uart->rx |= RX_ENB;

	/* Set baud rate */
//	uart->div = cfg->sys_clk_freq / cfg->baud_rate - 1;

	return 0;
}

static const struct uart_driver_api uart_spinalhdl_driver_api = {
	.poll_in          = uart_spinalhdl_poll_in,
	.poll_out         = uart_spinalhdl_poll_out,
	.err_check        = NULL,
};

static const struct uart_device_config uart_spinalhdl_dev_cfg = {
	.base = (void *)SPINALHDL_UART_BASE_ADDR,
	.sys_clk_freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
};

DEVICE_AND_API_INIT(uart_spinalhdl, "uart0",
		    uart_spinalhdl_init,
		    NULL, &uart_spinalhdl_dev_cfg,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_spinalhdl_driver_api);
