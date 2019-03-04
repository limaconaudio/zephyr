/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief I2S driver for Kendryte K210.
 *
 */

#include <errno.h>
#include <string.h>
#include <misc/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <i2s.h>
#include <soc.h>
#include <sys_io.h>

#include <clock_control.h>
#include <clock_control/kendryte_clock.h>
#include <i2s/i2s_kendryte.h>

struct i2s_kendryte_cfg {
	u64_t base;
	u32_t clock_id;
	u32_t thres_id;
	u32_t reset_id;
	u32_t dev_id;
};

struct stream {
	struct i2s_config cfg;
	void *mem_block;
	bool last_block;
};

/* Device run time data */
struct i2s_kendryte_data {
	struct device *clk;
	struct stream tx;
};

static u8_t *pcm_data;
static u32_t pcm_size;

#define DEV_CFG(dev)					\
	((const struct i2s_kendryte_cfg * const)	\
	 (dev)->config->config_info)

#define DEV_I2S(dev)					\
	((volatile i2s_t *)(DEV_CFG(dev))->base)

#define DEV_DATA(dev)					\
	(struct i2s_kendryte_data * const)(dev->driver_data)

#define MODULO_INC(val, max) { val = (++val < max) ? val : 0; }

static int i2s_set_mask_interrupt(volatile i2s_t *i2s,
		i2s_device_number_t device_num,
		i2s_channel_num_t channel_num,
		uint32_t rx_available_int, uint32_t rx_overrun_int,
		uint32_t tx_empty_int, uint32_t tx_overrun_int)
{
	imr_t u_imr;

	if (channel_num < I2S_CHANNEL_1 || channel_num > I2S_CHANNEL_3)
		return -1;

	u_imr.reg_data = sys_read32(&i2s->channel[channel_num].imr);

	if (rx_available_int == 1)
		u_imr.imr.rxdam = 1;
	else
		u_imr.imr.rxdam = 0;

	if (rx_overrun_int == 1)
		u_imr.imr.rxfom = 1;
	else
		u_imr.imr.rxfom = 0;

	if (tx_empty_int == 1)
		u_imr.imr.txfem = 1;
	else
		u_imr.imr.txfem = 0;

	if (tx_overrun_int == 1)
		u_imr.imr.txfom = 1;
	else
		u_imr.imr.txfom = 0;

	sys_write32(u_imr.reg_data, &i2s->channel[channel_num].imr);

	return 0;
}

static int i2s_transmit_channel_enable(volatile i2s_t *i2s,
				i2s_device_number_t device_num,
				i2s_channel_num_t channel_num, uint32_t enable)
{
	ter_t u_ter;

	if (channel_num < I2S_CHANNEL_1 || channel_num > I2S_CHANNEL_3)
		return -1;

	u_ter.reg_data = sys_read32(&i2s->channel[channel_num].ter);
	u_ter.ter.txchenx = enable;
	sys_write32(u_ter.reg_data, &i2s->channel[channel_num].ter);

	return 0;
}

static void i2s_transimit_enable(volatile i2s_t *i2s,
		i2s_device_number_t device_num, i2s_channel_num_t channel_num)
{
	iter_t u_iter;

	u_iter.reg_data = sys_read32(&i2s->iter);
	u_iter.iter.txen = 1;
	sys_write32(u_iter.reg_data, &i2s->iter);

	/* Transmitter block enable */
	i2s_transmit_channel_enable(i2s, device_num, channel_num, 1);
}

static void i2s_set_enable(volatile i2s_t *i2s,
		i2s_device_number_t device_num, uint32_t enable)
{
	ier_t u_ier;

	u_ier.reg_data = sys_read32(&i2s->ier);
	u_ier.ier.ien = enable;
	sys_write32(u_ier.reg_data, &i2s->ier);
}

static void i2s_disable_block(volatile i2s_t *i2s,
		i2s_device_number_t device_num, i2s_transmit_t rxtx_mode)
{
	irer_t u_irer;
	iter_t u_iter;

	if (rxtx_mode == I2S_RECEIVER)
	{
		/* Receiver block disable */
		u_irer.reg_data = sys_read32(&i2s->irer);
		u_irer.irer.rxen = 0;
		sys_write32(u_irer.reg_data, &i2s->irer);
	} else {
        	/* Transmitter block disable */
		u_iter.reg_data = sys_read32(&i2s->iter);
		u_iter.iter.txen = 0;
		sys_write32(u_iter.reg_data, &i2s->iter);
	}
}

static int i2s_set_tx_word_length(volatile i2s_t *i2s,
			i2s_device_number_t device_num,
			i2s_word_length_t word_length,
			i2s_channel_num_t channel_num)
{
	rcr_tcr_t u_tcr;

	if (word_length > RESOLUTION_32_BIT || word_length < IGNORE_WORD_LENGTH)
		return -1;

	if (channel_num < I2S_CHANNEL_1 || channel_num > I2S_CHANNEL_3)
		return -1;

	u_tcr.reg_data = sys_read32(&i2s->channel[channel_num].tcr);
	u_tcr.rcr_tcr.wlen = word_length;

	sys_write32(u_tcr.reg_data, &i2s->channel[channel_num].tcr);

	return 0;
}

static void i2s_master_configure(volatile i2s_t *i2s,
			i2s_device_number_t device_num,
			i2s_word_select_cycles_t word_select_size,
			i2s_sclk_gating_cycles_t gating_cycles,
			i2s_work_mode_t word_mode)
{
	ccr_t u_ccr;
	cer_t u_cer;

	u_ccr.reg_data = sys_read32(&i2s->ccr);
	u_ccr.ccr.clk_word_size = word_select_size;
	u_ccr.ccr.clk_gate = gating_cycles;
	u_ccr.ccr.align_mode = word_mode;
	sys_write32(u_ccr.reg_data, &i2s->ccr);

	/* Clock generation enable */
	u_cer.reg_data = sys_read32(&i2s->cer);
	u_cer.cer.clken = 1;
	sys_write32(u_cer.reg_data, &i2s->cer);
}

static int i2s_set_tx_threshold(volatile i2s_t *i2s,
			i2s_device_number_t device_num,
			i2s_fifo_threshold_t threshold,
			i2s_channel_num_t channel_num)
{
	tfcr_t u_tfcr;

	if (threshold < TRIGGER_LEVEL_1 || threshold > TRIGGER_LEVEL_16)
		return -1;

	if (channel_num < I2S_CHANNEL_1 || channel_num > I2S_CHANNEL_3)
		return -1;

	u_tfcr.reg_data = sys_read32(&i2s->channel[channel_num].tfcr);
	u_tfcr.tfcr.txchet = threshold;
	sys_write32(u_tfcr.reg_data, &i2s->channel[channel_num].tfcr);

	return 0;
}

static int i2s_kendryte_configure(struct device *dev, enum i2s_dir dir,
			      struct i2s_config *i2s_cfg)
{
	const struct i2s_kendryte_cfg *const cfg = DEV_CFG(dev);
	struct i2s_kendryte_data *const data = DEV_DATA(dev);
	volatile i2s_t *i2s = DEV_I2S(dev);
	u8_t word_size_bits = i2s_cfg->word_size;
	u8_t word_size_bytes;
	i2s_work_mode_t word_mode;
	u32_t word_select_size = i2s_cfg->frame_clk_freq;
	u32_t channel_mask, trigger_level;
	u32_t channel_num = I2S_CHANNEL_1; /* use I2S_CHANNEL_1 for now */
	int i;

	if (dir != I2S_DIR_TX) {
		printk("TX direction must be selected");
		return -EINVAL;
	}

	channel_mask = 0x0C;
	trigger_level = 4;

	for (i = 0; i < 4; i++) {
		if ((channel_mask & 0x3) == 0x3) {
			i2s_set_mask_interrupt(i2s, cfg->dev_id, I2S_CHANNEL_1 + i, 1, 1, 1, 1);
			i2s_transimit_enable(i2s, cfg->dev_id, I2S_CHANNEL_1 + i);
		} else {
			i2s_transmit_channel_enable(i2s, cfg->dev_id, I2S_CHANNEL_1 + i, 0);
		}
		channel_mask >>= 2;
	}

	/* disable rx */
	sys_write32(0, &i2s->channel[channel_num].rer);

	/* Transmit channel disable */
	i2s_transmit_channel_enable(i2s, cfg->dev_id, channel_num, 0);

	/* flush tx fifo */
	sys_write32(1, &i2s->txffr);

	/* flush individual fifo */
	sys_write32(1, &i2s->channel[channel_num].tff);

	/* Word buf_len is RESOLUTION_16_BIT */
	i2s_set_tx_word_length(i2s, cfg->dev_id, word_size_bits, channel_num);

	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		word_mode = STANDARD_MODE;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		word_mode = RIGHT_JUSTIFYING_MODE;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		word_mode = LEFT_JUSTIFYING_MODE;
		break;
	default:
		return -EINVAL;
	}

	/* word select size is 16 bits,gating after 16 bit */
	i2s_master_configure(i2s, cfg->dev_id, word_select_size, NO_CLOCK_GATING, word_mode);

	/* Interrupt trigger when FIFO level is 8 */
	i2s_set_tx_threshold(i2s, cfg->dev_id, trigger_level, channel_num);

	i2s_transmit_channel_enable(i2s, cfg->dev_id, channel_num, 1);

	return 0;
}

static int i2s_kendryte_write(struct device *dev, void *mem_block, size_t size)
{
	const struct i2s_kendryte_cfg *const cfg = DEV_CFG(dev);
	struct i2s_kendryte_data *data = DEV_DATA(dev);

	/* Add data to the global ptr */
	pcm_data = (u8_t *)mem_block;
	pcm_size = size;

	return 0;
}

static int tx_stream_start(struct device *dev)
{
	const struct i2s_kendryte_cfg *const cfg = DEV_CFG(dev);
	struct i2s_kendryte_data *data = DEV_DATA(dev);
	volatile i2s_t *i2s = DEV_I2S(dev);
	isr_t u_isr;
	u32_t left_buffer = 0;
	u32_t right_buffer = 0;
	u32_t i = 0, j = 0;
	u32_t channel_num = I2S_CHANNEL_1; /* use I2S_CHANNEL_1 for now */
	int ret;
	u8_t single_length = 16;
	u8_t track_num = 2;

	pcm_size = pcm_size / (single_length / 8) / 2; /* sample num */
	/* Clear overrun flag */
	sys_read32(&i2s->channel[channel_num].tor);

	for (j = 0; j < pcm_size;) {
		u_isr.reg_data = sys_read32(&i2s->channel[channel_num].isr);
		if (u_isr.isr.txfe == 1) {
			switch(single_length) {
			case 16:
				left_buffer = ((uint16_t *)pcm_data)[i++];
				right_buffer = ((uint16_t *)pcm_data)[i++];
				break;
			case 32:
				left_buffer = ((uint32_t *)pcm_data)[i++];
				right_buffer = ((uint32_t *)pcm_data)[i++];
				break;
			default:
				return -EINVAL;
			}
			sys_write32(left_buffer, &i2s->channel[channel_num].left_rxtx);
			sys_write32(right_buffer, &i2s->channel[channel_num].right_rxtx);
			j++;
		}
	}

	return 0;
}

static int i2s_kendryte_trigger(struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
	int ret;

	switch (cmd) {
	case I2S_TRIGGER_START:
		ret = tx_stream_start(dev);
		if (ret < 0) {
			printk("START trigger failed %d\n", ret);
			return ret;
		}
		break;

	case I2S_TRIGGER_STOP:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int i2s_kendryte_initialize(struct device *dev)
{
	const struct i2s_kendryte_cfg *const cfg = DEV_CFG(dev);
	struct i2s_kendryte_data *data = DEV_DATA(dev);
	volatile i2s_t *i2s = DEV_I2S(dev);

	/* Enable I2S clock */
	data->clk = device_get_binding(CONFIG_KENDRYTE_SYSCTL_NAME);
	__ASSERT_NO_MSG(data->clk);
	clock_control_on(data->clk, (clock_control_subsys_t) cfg->clock_id +
			cfg->dev_id);

	sysctl_reset(data->clk, cfg->reset_id + cfg->dev_id);
	kendryte_clock_set_threshold(data->clk,
				     (kendryte_threshold_t) cfg->thres_id +
				     cfg->dev_id, 7);

	i2s_set_enable(i2s, cfg->dev_id, 1);
	i2s_disable_block(i2s, cfg->dev_id, I2S_TRANSMITTER);
	i2s_disable_block(i2s, cfg->dev_id, I2S_RECEIVER);

	return 0;
}

static const struct i2s_driver_api i2s_kendryte_driver_api = {
	.configure = i2s_kendryte_configure,
	.write = i2s_kendryte_write,
	.trigger = i2s_kendryte_trigger,
};

/* I2S1 */

struct i2s_kendryte_data i2s_kendryte_data1;

static const struct i2s_kendryte_cfg i2s_kendryte_cfg1 = {
	.base = CONFIG_KENDRYTE_I2S_0_BASE_ADDR,
	.clock_id = KENDRYTE_CLOCK_I2S0,
	.thres_id = KENDRYTE_THRESHOLD_I2S0,
	.reset_id = SYSCTL_RESET_I2S0,
	.dev_id = I2S_DEVICE_0,
};

DEVICE_AND_API_INIT(i2s_kendryte, CONFIG_KENDRYTE_I2S_0_LABEL, &i2s_kendryte_initialize,
		    &i2s_kendryte_data1, &i2s_kendryte_cfg1, POST_KERNEL,
		    CONFIG_I2S_INIT_PRIORITY, &i2s_kendryte_driver_api);
