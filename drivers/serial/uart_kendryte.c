/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief UART driver for the SiFive Freedom Processor
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>
#include <board.h>
#include <clock_control.h>

#include <clock_control/kendryte_clock.h>

#define RXDATA_MASK    0xFF        /* Receive Data Mask */

#define TXDATA_FULL    (1 << 6)   /* Transmit FIFO Full */

struct uart_kendryte_regs_t
{
    union
    {
        volatile uint32_t RBR;
        volatile uint32_t DLL;
        volatile uint32_t THR;
    };

    union
    {
        volatile uint32_t DLH;
        volatile uint32_t IER;
    };

    union
    {
        volatile uint32_t FCR;
        volatile uint32_t IIR;
    };

    volatile uint32_t LCR;
    volatile uint32_t MCR;
    volatile uint32_t LSR;
    volatile uint32_t MSR;
    volatile uint32_t SCR;
    volatile uint32_t LPDLL;
    volatile uint32_t LPDLH;
    volatile uint32_t reserve[18];
    volatile uint32_t FAR;
    volatile uint32_t TFR;
    volatile uint32_t RFW;
    volatile uint32_t USR;
    volatile uint32_t TFL;
    volatile uint32_t RFL;
    volatile uint32_t SRR;
    volatile uint32_t SRTS;
    volatile uint32_t SBCR;
    volatile uint32_t SDMAM;
    volatile uint32_t SFE;
    volatile uint32_t SRT;
    volatile uint32_t STET;
    volatile uint32_t HTX;
    volatile uint32_t DMASA;
    volatile uint32_t TCR;
    volatile uint32_t DE_EN;
    volatile uint32_t RE_EN;
    volatile uint32_t DET;
    volatile uint32_t TAT;
    volatile uint32_t DLF;
    volatile uint32_t RAR;
    volatile uint32_t TAR;
    volatile uint32_t LCR_EXT;
    volatile uint32_t R[5];
    volatile uint32_t CPR;
    volatile uint32_t UCV;
    volatile uint32_t CTR;
};

struct uart_kendryte_device_config {
	u64_t	port;
	u32_t	sys_clk_freq;
	u32_t	baud_rate;
	u32_t	clock_id;
};

#define DEV_CFG(dev)						\
	((const struct uart_kendryte_device_config * const)	\
	 (dev)->config->config_info)
#define DEV_UART(dev)						\
	((struct uart_kendryte_regs_t *)(DEV_CFG(dev))->port)

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
static unsigned char uart_kendryte_poll_out(struct device *dev,
					 unsigned char c)
{
	volatile struct uart_kendryte_regs_t *uart = DEV_UART(dev);

    	while (uart->LSR & (1u << 5));
	/* Wait while TX FIFO is full */
//	while (!(uart->LSR & TXDATA_FULL));

	uart->THR = (char)c;

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
static int uart_kendryte_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct uart_kendryte_regs_t *uart = DEV_UART(dev);
	u32_t val = uart->RBR;

	*c = (uint8_t)(val & RXDATA_MASK);

	return 0;
}

static int uart_kendryte_init(struct device *dev)
{
	const struct uart_kendryte_device_config * const cfg = DEV_CFG(dev);
	volatile struct uart_kendryte_regs_t *uart = DEV_UART(dev);
	struct device *clk =
		device_get_binding(CONFIG_KENDRYTE_SYSCTL_NAME);
	u32_t rate = 195000000;
	uint8_t databits, stopbit_val, parity_val;

//	clock_control_get_rate(clk, (void *)KENDRYTE_CLOCK_APB0, &rate);
	uint32_t u16_div = (rate + 16  * cfg->baud_rate / 2) /
				(16 * cfg->baud_rate);

	/* enable clock */
	clock_control_on(clk, (void *)cfg->clock_id);

	databits = 8;
	stopbit_val = 0;
	parity_val = 0;

        /* Set UART registers */
        uart->TCR &= ~(1u);
        uart->TCR &= ~(1u << 3);
        uart->TCR &= ~(1u << 4);
        uart->TCR |= (1u << 2);
        uart->TCR &= ~(1u << 1);
        uart->DE_EN &= ~(1u);

        uart->LCR |= 1u << 7;
        uart->DLL = u16_div & 0xFF;
        uart->DLH = u16_div >> 8;
        uart->LCR = 0;
        uart->LCR = (databits - 5) | (stopbit_val << 2) | (parity_val << 3);
        uart->LCR &= ~(1u << 7);
        uart->MCR &= ~3;
	uart->FCR = 0 << 6 | 3 << 4 | 0x1 << 3 | 0x1;
	uart->IER = 0x80;

	return 0;
}

static const struct uart_driver_api uart_kendryte_driver_api = {
	.poll_in          = uart_kendryte_poll_in,
	.poll_out         = uart_kendryte_poll_out,
	.err_check        = NULL,
};

#ifdef CONFIG_UART_KENDRYTE_PORT_1

static const struct uart_kendryte_device_config uart_kendryte_dev_cfg_1 = {
	.port		= CONFIG_KENDRYTE_UART_1_BASE_ADDR,
	.sys_clk_freq	= CONFIG_KENDRYTE_UART_1_CLK_FREQ,
	.baud_rate	= CONFIG_KENDRYTE_UART_1_CURRENT_SPEED,
	.clock_id	= KENDRYTE_CLOCK_UART1,
};

DEVICE_AND_API_INIT(uart_kendryte_1, CONFIG_KENDRYTE_UART_1_LABEL,
		    uart_kendryte_init,
		    NULL, &uart_kendryte_dev_cfg_1,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_kendryte_driver_api);

#endif /* CONFIG_UART_KENDRYTE_PORT_1 */

#ifdef CONFIG_UART_KENDRYTE_PORT_2

static const struct uart_kendryte_device_config uart_kendryte_dev_cfg_2 = {
	.port		= CONFIG_KENDRYTE_UART_2_BASE_ADDR,
	.sys_clk_freq	= CONFIG_KENDRYTE_UART_2_CLK_FREQ,
	.baud_rate	= CONFIG_KENDRYTE_UART_2_CURRENT_SPEED,
	.clock_id	= KENDRYTE_CLOCK_UART2,
};

DEVICE_AND_API_INIT(uart_kendryte_2, CONFIG_KENDRYTE_UART_2_LABEL,
		    uart_kendryte_init,
		    NULL, &uart_kendryte_dev_cfg_2,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_kendryte_driver_api);

#endif /* CONFIG_UART_KENDRYTE_PORT_2 */

#ifdef CONFIG_UART_KENDRYTE_PORT_3

static const struct uart_kendryte_device_config uart_kendryte_dev_cfg_3 = {
	.port		= CONFIG_KENDRYTE_UART_3_BASE_ADDR,
	.sys_clk_freq	= CONFIG_KENDRYTE_UART_3_CLK_FREQ,
	.baud_rate	= CONFIG_KENDRYTE_UART_3_CURRENT_SPEED,
	.clock_id	= KENDRYTE_CLOCK_UART3,
};

DEVICE_AND_API_INIT(uart_kendryte_3, CONFIG_KENDRYTE_UART_3_LABEL,
		    uart_kendryte_init,
		    NULL, &uart_kendryte_dev_cfg_3,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_kendryte_driver_api);

#endif /* CONFIG_UART_KENDRYTE_PORT_3 */
