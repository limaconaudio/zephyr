/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/util.h>
#include <kernel.h>
#include <board.h>
#include <errno.h>
#include <spi.h>
#include <toolchain.h>

#include <clock_control/kendryte_clock.h>
#include <clock_control.h>

#include "spi_context.h"
#include "spi_kendryte.h"

#define DEV_CFG(dev)						\
(const struct spi_kendryte_cfg * const)(dev->config->config_info)

#define DEV_SPI(dev)					\
((volatile spi_t *)(DEV_CFG(dev))->base)

#define DEV_DATA(dev)					\
(struct spi_kendryte_data * const)(dev->driver_data)

struct spi_kendryte_cfg {
	u64_t base;
	u32_t clock_id;
	u32_t thres_id;
};

struct spi_kendryte_data {
	struct device *clk;
	struct spi_context ctx;
} kendryte_spi_data;

void kendryte_spi_set_clk(struct device *dev, u32_t spi_clk)
{
	const struct spi_kendryte_cfg *cfg = DEV_CFG(dev);
	struct spi_kendryte_data *data = DEV_DATA(dev);
	volatile spi_t *spi = DEV_SPI(dev);
	u32_t rate;
	u32_t spi_baudr;

	// FIXME: Get clock rate from API
//	clock_control_get_rate(data->clk, (clock_control_subsys_t)cfg->clock_id,
//				&rate);
	rate = 390000000;

	printk("SPI0 Clk rate: %d\n", rate);
	spi_baudr = rate / spi_clk;

	if (spi_baudr < 2 ) {
		spi_baudr = 2;
	} else if(spi_baudr > 65534) {
		spi_baudr = 65534;
	}

	spi->baudr = spi_baudr;
}

static int spi_kendryte_configure(struct device *dev,
			       const struct spi_config *config)
{
	struct spi_kendryte_data *data = DEV_DATA(dev);
	volatile spi_t *spi = DEV_SPI(dev);
	spi_frame_format_t frame_format = SPI_FF_STANDARD;
	spi_work_mode_t work_mode;
	uint8_t dfs_offset = 16, frf_offset = 21, work_mode_offset = 6;
	uint8_t cpol = SPI_MODE_GET(config->operation) & SPI_MODE_CPOL;
	uint8_t cpha = SPI_MODE_GET(config->operation) & SPI_MODE_CPHA;

	if (spi_context_configured(&data->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	kendryte_spi_set_clk(dev, config->frequency);

	if (cpol & cpha)
		work_mode = SPI_WORK_MODE_3;
	else if (cpol)
		work_mode = SPI_WORK_MODE_2;
	else if (cpha)
		work_mode = SPI_WORK_MODE_1;
	else
		work_mode = SPI_WORK_MODE_0;

	printk("SPI Work mode: %d\n", work_mode);
	spi->imr = 0x00;
	spi->dmacr = 0x00;
	spi->dmatdlr = 0x10;
	spi->dmardlr = 0x00;
	spi->ser = 0x00;
	spi->ssienr = 0x00;
	spi->ctrlr0 = (work_mode << work_mode_offset) |
			(frame_format << frf_offset) |
			((SPI_WORD_SIZE_GET(config->operation) - 1) << dfs_offset);
	spi->spi_ctrlr0 = 0;
	spi->endian = 0;

	return 0;
}

static int spi_kendryte_release(struct device *dev,
			     const struct spi_config *config)
{
	struct spi_kendryte_data *data = DEV_DATA(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static inline u32_t spi_kendryte_next_tx(struct spi_kendryte_data *data,
					 u8_t word_size)
{
	u32_t tx_frame = 0;
	if (spi_context_tx_buf_on(&data->ctx)) {
		switch (word_size) {
		case 32:
			tx_frame = UNALIGNED_GET((u32_t *)(data->ctx.tx_buf));
			break;
		case 16:
			tx_frame = UNALIGNED_GET((u16_t *)(data->ctx.tx_buf));
			break;
		default:
			tx_frame = UNALIGNED_GET((u8_t *)(data->ctx.tx_buf));
			break;
		}
	}

	return tx_frame;
}

static bool spi_kendryte_transfer_ongoing(struct spi_kendryte_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

/* Shift a SPI frame as master. */
static void spi_kendryte_shift_frames(struct device *dev,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs)
{
	struct spi_kendryte_data *data = DEV_DATA(dev);
	volatile spi_t *spi = DEV_SPI(dev);
	u32_t tx_frame, rx_frame;
	u32_t reg;
	u8_t word_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
	int i;

	/* Set TMOD register value */
	reg = spi->ctrlr0 & ~(3 << 8);
	reg |= SPI_TMOD_TRANS << 8;
	spi->ctrlr0 = reg;

	spi->ssienr = 0x01;
	spi->ser = 1 << SPI_CHIP_SELECT_3;
	printk("SPI CTRLR0 %d\n", spi->ctrlr0);

	while (spi_context_tx_on(&data->ctx)) {
		tx_frame = spi_kendryte_next_tx(data, word_size);
		printk("Sending frame: %x\t", tx_frame);
	
		switch (word_size) {
		case 32:
			spi->dr[0] = tx_frame;
			spi_context_update_tx(&data->ctx, 4, 1);
			break;
		case 16:
			spi->dr[0] = (u16_t) tx_frame;
			spi_context_update_tx(&data->ctx, 2, 1);
			break;
		default:
			spi->dr[0] = (u8_t) tx_frame;
			spi_context_update_tx(&data->ctx, 1, 1);
			break;
		}

	}

	/* Wait until data is pushed out */
	while ((spi->sr & 0x05) != 0x04);

	spi->ssienr = 0x00;
	spi->ser = 0x00;

	/* Set TMOD register value */
	reg = spi->ctrlr0 & ~(3 << 8);
	reg |= SPI_TMOD_RECV << 8;
	spi->ctrlr0 = reg;

	spi->ssienr = 0x01;
	spi->ser = 1 << SPI_CHIP_SELECT_3;

	spi->ctrlr1 = word_size - 1;

	spi->dr[0] = 0xffffffff;
	while (spi_context_rx_on(&data->ctx)) {
		printk("Rx FIFO Len: %d\n", spi->rxflr);
		switch (word_size) {
		case 32:
			rx_frame = spi->dr[0];
			if (spi_context_rx_buf_on(&data->ctx)) {
				UNALIGNED_PUT(rx_frame, (u32_t *)data->ctx.rx_buf);
			}
			spi_context_update_rx(&data->ctx, 4, 1);
			break;
		case 16:
			rx_frame = (u16_t) spi->dr[0];
			if (spi_context_rx_buf_on(&data->ctx)) {
				UNALIGNED_PUT(rx_frame, (u16_t *)data->ctx.rx_buf);
			}
			spi_context_update_rx(&data->ctx, 2, 1);
			break;
		default:
			rx_frame = (u8_t) spi->dr[0];
			if (spi_context_rx_buf_on(&data->ctx)) {
				UNALIGNED_PUT(rx_frame, (u8_t *)data->ctx.rx_buf);
			}
			spi_context_update_rx(&data->ctx, 1, 1);
			break;
		}
		printk("Received frame: %x\n", rx_frame);
	}
	
	spi->ssienr = 0x00;
	spi->ser = 0x00;

}

static int transceive(struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous, struct k_poll_signal *signal)
{
	struct spi_kendryte_data *data = DEV_DATA(dev);
	volatile spi_t *spi = DEV_SPI(dev);
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	spi_context_lock(&data->ctx, asynchronous, signal);

	ret = spi_kendryte_configure(dev, config);
	if (ret) {
		return ret;
	}

	/* Set buffers info */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	/* This is turned off in spi_kendryte_complete(). */
//	spi_context_cs_control(&data->ctx, true);

	do {
		spi_kendryte_shift_frames(dev, tx_bufs, rx_bufs);
	} while (spi_kendryte_transfer_ongoing(data));

	spi->ser = 0x00;
	spi->ssienr = 0x00;

	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_kendryte_transceive(struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

static const struct spi_driver_api api_funcs = {
	.transceive = spi_kendryte_transceive,
	.release = spi_kendryte_release,
};

static int spi_kendryte_init(struct device *dev)
{
	const struct spi_kendryte_cfg *cfg = DEV_CFG(dev);
	struct spi_kendryte_data *data = DEV_DATA(dev);

	/* Enable SPI clock */
	data->clk = device_get_binding(CONFIG_KENDRYTE_SYSCTL_NAME);

	__ASSERT_NO_MSG(data->clk);

	clock_control_on(data->clk, (clock_control_subsys_t) cfg->clock_id);
	kendryte_clock_set_threshold(data->clk,
				     (kendryte_threshold_t) cfg->thres_id, 0);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_0

static const struct spi_kendryte_cfg spi_kendryte_cfg_0 = {
	.base = CONFIG_KENDRYTE_SPI_0_BASE_ADDR,
	.clock_id = KENDRYTE_CLOCK_SPI0,
	.thres_id = KENDRYTE_THRESHOLD_SPI0,
};

static struct spi_kendryte_data spi_kendryte_dev_data_0 = {
	SPI_CONTEXT_INIT_LOCK(spi_kendryte_dev_data_0, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_kendryte_dev_data_0, ctx),
};

DEVICE_AND_API_INIT(spi_kendryte_0, CONFIG_KENDRYTE_SPI_0_LABEL, &spi_kendryte_init,
		    &spi_kendryte_dev_data_0, &spi_kendryte_cfg_0,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#endif /* CONFIG_SPI_0 */

#ifdef CONFIG_SPI_1

static const struct spi_kendryte_cfg spi_kendryte_cfg_1 = {
	.base = CONFIG_KENDRYTE_SPI_1_BASE_ADDR,
	.clock_id = KENDRYTE_CLOCK_SPI1,
	.thres_id = KENDRYTE_THRESHOLD_SPI1,
};

static struct spi_kendryte_data spi_kendryte_dev_data_1 = {
	SPI_CONTEXT_INIT_LOCK(spi_kendryte_dev_data_1, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_kendryte_dev_data_1, ctx),
};

DEVICE_AND_API_INIT(spi_kendryte_1, CONFIG_KENDRYTE_SPI_1_LABEL, &spi_kendryte_init,
		    &spi_kendryte_dev_data_1, &spi_kendryte_cfg_1,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#endif /* CONFIG_SPI_1 */
