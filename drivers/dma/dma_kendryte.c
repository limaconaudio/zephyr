/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <clock_control.h>
#include <device.h>
#include <dma.h>
#include <errno.h>
#include <init.h>
#include <stdio.h>
#include <string.h>
#include <misc/util.h>

#include <clock_control/kendryte_clock.h>
#include <dma/kendryte_dma.h>

#define writeq(v, addr)	((*(volatile u64_t *)(addr)) = (v))
#define readq(addr) (*(volatile u64_t *)(addr))

struct dmac_kendryte_cfg {
	u64_t base;
	u32_t clock_id;
};

struct dmac_kendryte_data {
	struct device *clk;
} kendryte_dmac_data;

#define DEV_CFG(dev)					\
	((const struct dmac_kendryte_cfg * const)	\
	 (dev)->config->config_info)

#define DEV_DMA(dev)					\
	((volatile dmac_t *)(DEV_CFG(dev))->base)

#define DEV_DATA(dev)					\
	(struct dmac_kendryte_data * const)(dev->driver_data)

static void dmac_channel_interrupt_clear(volatile dmac_t *dmac,
					 dmac_channel_number_t channel_num)
{
	writeq(0xffffffff, &dmac->channel[channel_num].intclear);
}

static void dmac_enable(volatile dmac_t *dmac)
{
	dmac_cfg_u_t dmac_cfg;

	dmac_cfg.data = readq(&dmac->cfg);
	dmac_cfg.cfg.dmac_en = 1;
	dmac_cfg.cfg.int_en = 1;
	writeq(dmac_cfg.data, &dmac->cfg);
}

static void dmac_disable(volatile dmac_t *dmac)
{
	dmac_cfg_u_t dmac_cfg;

	dmac_cfg.data = readq(&dmac->cfg);
	dmac_cfg.cfg.dmac_en = 0;
	dmac_cfg.cfg.int_en = 0;
	writeq(dmac_cfg.data, &dmac->cfg);
}

static int dmac_is_idle(volatile dmac_t *dmac,
			dmac_channel_number_t channel_num)
{
	dmac_chen_u_t chen;
	chen.data = readq(&dmac->chen);

	if ((chen.data >> channel_num) & 0x1UL)
		return 0;
	else
		return 1;
}

void dmac_wait_idle(volatile dmac_t *dmac,
		    dmac_channel_number_t channel_num)
{
	while (!dmac_is_idle(dmac, channel_num));
	dmac_channel_interrupt_clear(dmac, channel_num); /* clear interrupt */
}

void dmac_wait_done(struct device *dev, dmac_channel_number_t channel_num)
{
    volatile dmac_t *dmac = DEV_DMA(dev);

    while (!(readq(&dmac->channel[channel_num].intstatus) & 0x2))
        ;
    dmac_channel_interrupt_clear(dmac, channel_num); /* clear interrupt */
}

void dmac_channel_enable(volatile dmac_t *dmac,
			 dmac_channel_number_t channel_num)
{
	dmac_chen_u_t chen;

	chen.data = readq(&dmac->chen);

	switch (channel_num) {
	case DMAC_CHANNEL0:
		chen.dmac_chen.ch1_en = 1;
		chen.dmac_chen.ch1_en_we = 1;
		break;
	case DMAC_CHANNEL1:
		chen.dmac_chen.ch2_en = 1;
		chen.dmac_chen.ch2_en_we = 1;
		break;
	case DMAC_CHANNEL2:
		chen.dmac_chen.ch3_en = 1;
		chen.dmac_chen.ch3_en_we = 1;
		break;
	case DMAC_CHANNEL3:
		chen.dmac_chen.ch4_en = 1;
		chen.dmac_chen.ch4_en_we = 1;
		break;
	case DMAC_CHANNEL4:
		chen.dmac_chen.ch5_en = 1;
		chen.dmac_chen.ch5_en_we = 1;
		break;
	case DMAC_CHANNEL5:
		chen.dmac_chen.ch6_en = 1;
		chen.dmac_chen.ch6_en_we = 1;
		break;
	default:
		break;
	}

	writeq(chen.data, &dmac->chen);
}

void dmac_channel_disable(volatile dmac_t *dmac,
			  dmac_channel_number_t channel_num)
{
	dmac_chen_u_t chen;

	chen.data = readq(&dmac->chen);

	switch (channel_num) {
	case DMAC_CHANNEL0:
		chen.dmac_chen.ch1_en = 0;
		chen.dmac_chen.ch1_en_we = 1;
		break;
	case DMAC_CHANNEL1:
		chen.dmac_chen.ch2_en = 0;
		chen.dmac_chen.ch2_en_we = 1;
		break;
	case DMAC_CHANNEL2:
		chen.dmac_chen.ch3_en = 0;
		chen.dmac_chen.ch3_en_we = 1;
		break;
	case DMAC_CHANNEL3:
		chen.dmac_chen.ch4_en = 0;
		chen.dmac_chen.ch4_en_we = 1;
		break;
	case DMAC_CHANNEL4:
		chen.dmac_chen.ch5_en = 0;
		chen.dmac_chen.ch5_en_we = 1;
		break;
	case DMAC_CHANNEL5:
		chen.dmac_chen.ch6_en = 0;
		chen.dmac_chen.ch6_en_we = 1;
		break;
	default:
		break;
	}

	writeq(chen.data, &dmac->chen);
}

int32_t dmac_check_channel_busy(volatile dmac_t *dmac,
				dmac_channel_number_t channel_num)
{
	int32_t ret = 0;
	dmac_chen_u_t chen_u;

	chen_u.data = readq(&dmac->chen);
	switch (channel_num) {
	case DMAC_CHANNEL0:
		if (chen_u.dmac_chen.ch1_en == 1)
			ret = 1;
		break;
	case DMAC_CHANNEL1:
		if (chen_u.dmac_chen.ch2_en == 1)
			ret = 1;
		break;
	case DMAC_CHANNEL2:
		if (chen_u.dmac_chen.ch3_en == 1)
			ret = 1;
		break;
	case DMAC_CHANNEL3:
		if (chen_u.dmac_chen.ch4_en == 1)
			ret = 1;
		break;
	case DMAC_CHANNEL4:
		if (chen_u.dmac_chen.ch5_en == 1)
			ret = 1;
		break;
	case DMAC_CHANNEL5:
		if (chen_u.dmac_chen.ch6_en == 1)
			ret = 1;
		break;
	default:
		break;
	}

	writeq(chen_u.data, &dmac->chen);

	return ret;
}

static int dmac_kendryte_config(struct device *dev, u32_t id,
			    struct dma_config *config)
{
	volatile dmac_t *dmac = DEV_DMA(dev);
	dmac_transfer_flow_t flow_control;
	dmac_sw_hw_hs_select_t src_type, dest_type;
	dmac_ch_cfg_u_t cfg_u;
	dmac_ch_ctl_u_t ctl;
	int ret;

	if (id >= DMAC_CHANNEL_MAX) {
		printk("Invalid DMA channel\n");
		return -EINVAL;
	}

	dmac_channel_interrupt_clear(dmac, id);
	dmac_channel_disable(dmac, id);
	dmac_wait_idle(dmac, id);

	if (config->channel_direction == MEMORY_TO_PERIPHERAL) {
		flow_control = DMAC_MEM2PRF_DMA;
		src_type = DMAC_HS_SOFTWARE;
		dest_type = DMAC_HS_HARDWARE;
	} else if (config->channel_direction == PERIPHERAL_TO_MEMORY) {
		flow_control = DMAC_PRF2MEM_DMA;
		src_type = DMAC_HS_HARDWARE;
		dest_type = DMAC_HS_SOFTWARE;
	} else {
		flow_control = DMAC_MEM2MEM_DMA;
		src_type = DMAC_HS_SOFTWARE;
		dest_type = DMAC_HS_SOFTWARE;
	}

	/*
	 * cfg register must be configured before ts_block and
	 * sar dar register
	 */
	cfg_u.data = readq(&dmac->channel[id].cfg);

	cfg_u.ch_cfg.tt_fc = flow_control;
	cfg_u.ch_cfg.hs_sel_src = src_type;
	cfg_u.ch_cfg.hs_sel_dst = dest_type;
	cfg_u.ch_cfg.src_per = id;
	cfg_u.ch_cfg.dst_per = id;
	cfg_u.ch_cfg.src_multblk_type = 0;
	cfg_u.ch_cfg.dst_multblk_type = 0;

	writeq(cfg_u.data, &dmac->channel[id].cfg);

	dmac->channel[id].sar = (uint64_t)config->head_block->source_address;
	dmac->channel[id].dar = (uint64_t)config->head_block->dest_address;

	ctl.data = readq(&dmac->channel[id].ctl);
	ctl.ch_ctl.sms = DMAC_MASTER1;
	ctl.ch_ctl.dms = DMAC_MASTER2;

	/* Master select */
	ctl.ch_ctl.sinc = config->head_block->source_addr_adj;
	ctl.ch_ctl.dinc = config->head_block->dest_addr_adj;

	/* Address incrememt */
	ctl.ch_ctl.src_tr_width = config->source_data_size;
	ctl.ch_ctl.dst_tr_width = config->dest_data_size;

	/* Transfer width */
	ctl.ch_ctl.src_msize = config->source_burst_length;
	ctl.ch_ctl.dst_msize = config->dest_burst_length;

	writeq(ctl.data, &dmac->channel[id].ctl);

	writeq(config->head_block->block_size - 1, &dmac->channel[id].block_ts);

	return 0;
}

static int dmac_kendryte_start(struct device *dev, u32_t id)
{
	volatile dmac_t *dmac = DEV_DMA(dev);

	if (id >= DMAC_CHANNEL_MAX) {
		return -EINVAL;
	}

	dmac_enable(dmac);
	dmac_channel_enable(dmac, id);

	return 0;
}

static int dmac_kendryte_stop(struct device *dev, u32_t id)
{
	volatile dmac_t *dmac = DEV_DMA(dev);

	if (id >= DMAC_CHANNEL_MAX) {
		return -EINVAL;
	}

	dmac_disable(dmac);
	dmac_channel_disable(dmac, id);

	return 0;
}

static int dmac_kendryte_init(struct device *dev)
{
	volatile dmac_t *dmac = DEV_DMA(dev);
	const struct dmac_kendryte_cfg *const cfg = DEV_CFG(dev);
	struct dmac_kendryte_data *data = DEV_DATA(dev);
	dmac_commonreg_intclear_u_t intclear;
	dmac_cfg_u_t dmac_cfg;
	dmac_reset_u_t dmac_reset;
	u64_t tmp;

	/* Enable DMA clock */
	data->clk = device_get_binding(CONFIG_KENDRYTE_SYSCTL_NAME);

	__ASSERT_NO_MSG(data->clk);

	clock_control_on(data->clk, (void *)cfg->clock_id);

	dmac_reset.data = readq(&dmac->reset);
	dmac_reset.reset.rst = 1;
	writeq(dmac_reset.data, &dmac->reset);
	while (dmac_reset.reset.rst)
		dmac_reset.data = readq(&dmac->reset);

	/* Reset dmac */
	intclear.data = readq(&dmac->com_intclear);
	intclear.com_intclear.cear_slvif_dec_err_intstat = 1;
	intclear.com_intclear.clear_slvif_wr2ro_err_intstat = 1;
	intclear.com_intclear.clear_slvif_rd2wo_err_intstat = 1;
	intclear.com_intclear.clear_slvif_wronhold_err_intstat = 1;
	intclear.com_intclear.clear_slvif_undefinedreg_dec_err_intstat = 1;
	writeq(intclear.data, &dmac->com_intclear);

	/* Clear common register interrupt */
	dmac_cfg.data = readq(&dmac->cfg);
	dmac_cfg.cfg.dmac_en = 0;
	dmac_cfg.cfg.int_en = 0;
	writeq(dmac_cfg.data, &dmac->cfg);

	/* Disable dmac and disable interrupt */
	while (readq(&dmac->cfg));

	tmp = readq(&dmac->chen);
	tmp &= ~0xf;
	writeq(tmp, &dmac->chen);

	/* Disable all channel before configure */
	dmac_enable(dmac);
	
	return 0;
}

static const struct dma_driver_api dmac_funcs = {
	.config		 = dmac_kendryte_config,
	.start		 = dmac_kendryte_start,
	.stop		 = dmac_kendryte_stop,
};

static const struct dmac_kendryte_cfg kendryte_dmac_cfg = {
	.base = CONFIG_KENDRYTE_DMA_BASE_ADDR,
	.clock_id = KENDRYTE_CLOCK_DMA,
};

DEVICE_AND_API_INIT(dmac_kendryte, CONFIG_KENDRYTE_DMA_NAME, &dmac_kendryte_init,
		    &kendryte_dmac_data, &kendryte_dmac_cfg,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&dmac_funcs);
