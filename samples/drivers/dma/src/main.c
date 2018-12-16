/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <device.h>
#include <dma.h>
#include <kernel.h>
#include <misc/printk.h>
#include <string.h>

#define DMA_CHANNEL 1
#define DMA_SLOT 2

K_MEM_SLAB_DEFINE(dma_src_slab, 8, 4, 1);
K_MEM_SLAB_DEFINE(dma_dest_slab, 8, 4, 1);

static void dma_callback(void *arg, u32_t channel, int status)
{
	struct dma_block_config *blk_config = arg;
	u32_t *buf;

	if (status != 0) {
		printk("DMA error: %d\n", status);
		return;
	}

	printk("Transferred 1 block using channel: %d\n", channel);

	buf = (u32_t *)blk_config->dest_address;
	printk("Dest buffer: %x\n", *buf++);
}

void main(void)
{
	struct dma_config config;
	struct dma_block_config blk_config;
	struct device *dma_dev;
	void *src_buf, *dest_buf;
	u32_t *buf;
	int ret, i;

	printk("Testing DMA driver\n");
	dma_dev = device_get_binding(CONFIG_KENDRYTE_DMA_NAME);

	config.block_count = 1,
	config.complete_callback_en = 1,
	config.dma_slot = DMA_SLOT,
	config.channel_direction = MEMORY_TO_MEMORY,
	config.source_data_size = 2,  /* 32bit */
	config.dest_data_size = 2,    /* 32bit */
	config.source_burst_length = 0,
	config.dest_burst_length = 0, /* SINGLE transfer */
	config.callback_arg = &blk_config;
	config.dma_callback = dma_callback,

	ret = k_mem_slab_alloc(&dma_src_slab, &src_buf, K_NO_WAIT);
	if (ret < 0) {
		printk("Mem alloc error\n");
		return;
	}

	ret = k_mem_slab_alloc(&dma_dest_slab, &dest_buf, K_NO_WAIT);
	if (ret < 0) {
		printk("Mem alloc error\n");
		return;
	}

	/* Fill the src buffer with known values */
	printk("Filling the buffer with known values... 0xAABBEEFF\n");
	buf = (u32_t *)src_buf;
	for (i = 0; i < 8; i++) {
		buf[i] = 0xAABBEEFF;
	}

	buf = (u32_t *)dest_buf;
	for (i = 0; i < 8; i++) {
		buf[i] = 0x000000AA;
	}

	memset(&blk_config, 0, sizeof(blk_config));
	blk_config.source_address = (u32_t)src_buf;
	blk_config.dest_address = (u32_t)dest_buf;
	blk_config.block_size = 8;
	blk_config.next_block = NULL;

	config.head_block = &blk_config;

	ret = dma_config(dma_dev, DMA_CHANNEL, &config);
	if (ret < 0) {
		printk("DMA config error\n");
		return;
	}

	ret = dma_start(dma_dev, DMA_CHANNEL);

	k_sleep(100000);

	printk("Dest buffer: %x\n", *buf++);
	printk("Dest buffer: %x\n", *buf++);
	printk("Dest buffer: %x\n", *buf++);
	printk("Dest buffer: %x\n", *buf++);
	printk("Dest buffer: %x\n", *buf++);
	printk("Dest buffer: %x\n", *buf++);
	printk("Dest buffer: %x\n", *buf++);
	printk("Dest buffer: %x\n", *buf++);
}
