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

#define RXDATA_MASK    0xFF        /* Receive Data Mask */

#define TXDATA_FULL    (1 << 6)   /* Transmit FIFO Full */

struct uart_kendryte_regs_t
{
    union
    {
        uint32_t RBR;
        uint32_t DLL;
        uint32_t THR;
    };

    union
    {
        uint32_t DLH;
        uint32_t IER;
    };

    union
    {
        uint32_t FCR;
        uint32_t IIR;
    };

    uint32_t LCR;
    uint32_t MCR;
    uint32_t LSR;
    uint32_t MSR;
    uint32_t SCR;
    uint32_t LPDLL;
    uint32_t LPDLH;
    uint32_t reserve[18];
    uint32_t FAR;
    uint32_t TFR;
    uint32_t RFW;
    uint32_t USR;
    uint32_t TFL;
    uint32_t RFL;
    uint32_t SRR;
    uint32_t SRTS;
    uint32_t SBCR;
    uint32_t SDMAM;
    uint32_t SFE;
    uint32_t SRT;
    uint32_t STET;
    uint32_t HTX;
    uint32_t DMASA;
    uint32_t TCR;
    uint32_t DE_EN;
    uint32_t RE_EN;
    uint32_t DET;
    uint32_t TAT;
    uint32_t DLF;
    uint32_t RAR;
    uint32_t TAR;
    uint32_t LCR_EXT;
    uint32_t R[5];
    uint32_t CPR;
    uint32_t UCV;
    uint32_t CTR;
};

struct uart_kendryte_device_config {
	u32_t       port;
	u32_t       sys_clk_freq;
	u32_t       baud_rate;
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

	/* Wait while TX FIFO is full */
	while (!(uart->LSR & TXDATA_FULL));

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

	uint8_t databits, stopbit_val, parity_val;
	uint32_t u16_div = (cfg->sys_clk_freq + 16  * cfg->baud_rate / 2) /
				(16 * cfg->baud_rate);

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
//        uart->IER = 1; Interrupt Enable?

	return 0;
}

static const struct uart_driver_api uart_kendryte_driver_api = {
	.poll_in          = uart_kendryte_poll_in,
	.poll_out         = uart_kendryte_poll_out,
	.err_check        = NULL,
};

#ifdef CONFIG_UART_KENDRYTE_PORT_0

static const struct uart_kendryte_device_config uart_kendryte_dev_cfg_0 = {
	.port         = CONFIG_KENDRYTE_UART_0_BASE_ADDR,
	.sys_clk_freq = CONFIG_KENDRYTE_UART_0_CLK_FREQ,
	.baud_rate    = CONFIG_KENDRYTE_UART_0_CURRENT_SPEED,
};

DEVICE_AND_API_INIT(uart_kendryte_0, CONFIG_KENDRYTE_UART_0_LABEL,
		    uart_kendryte_init,
		    NULL, &uart_kendryte_dev_cfg_0,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_kendryte_driver_api);

#endif /* CONFIG_UART_KENDRYTE_PORT_0 */
