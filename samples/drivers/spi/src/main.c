/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <spi.h>

#define BUF_SIZE 5
u8_t buffer_tx[BUF_SIZE] = {0x12, 0xBB, 0xBB, 0x11, 0x11};
u8_t buffer_rx[BUF_SIZE] = {0};

struct spi_config spi_conf = {
	.frequency = 1000000,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
	.slave = 0,
	.cs = NULL,
};

void main(void)
{
	struct device *dev;
	int ret,i;

	const struct spi_buf tx_bufs[] = {
		{
			.buf = buffer_tx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs)
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs)
	};
	dev = device_get_binding("spi_0");
	if (!dev) {
		printk("Cannot find spi_0!\n");
		return;
	}

	ret = spi_transceive(dev, &spi_conf, &tx, &rx);
	if (ret) {
		printk("Code %d\n", ret);
		printk("SPI transceive failed\n");
		return;
	}

	for (i=0;i<5;i++)
		printk("TX Buf[%d]: %d\n", i, buffer_tx[i]);

	for (i=0;i<5;i++)
		printk("RX Buf[%d]: %d\n", i, buffer_rx[i]);
}
