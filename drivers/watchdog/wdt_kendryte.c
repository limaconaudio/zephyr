/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Watchdog (WDT) Driver for Kendryte SoC
 */

#include <watchdog.h>
#include <soc.h>
#include <clock_control.h>

#include <clock_control/kendryte_clock.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wdt_kendryte);

#define WDT_TORR_TOP(x)	((x) << 4 | (x) << 0)

/* WDT Control Register */
#define WDT_CR_ENABLE                                       0x00000001
#define WDT_CR_RMOD_MASK                                    0x00000002
#define WDT_CR_RMOD_RESET                                   0x00000000
#define WDT_CR_RMOD_INTERRUPT                               0x00000002
#define WDT_CR_RPL_MASK                                     0x0000001C
#define WDT_CR_RPL(x)                                       ((x) << 2)
/* WDT Timeout Range Register */
#define WDT_TORR_TOP_MASK                                   0x000000FF
#define WDT_TORR_TOP(x)                                     ((x) << 4 | (x) << 0)
/* WDT Current Counter Value Register */
#define WDT_CCVR_MASK                                       0xFFFFFFFF
/* WDT Counter Restart Register */
#define WDT_CRR_MASK                                        0x00000076
/* WDT Interrupt Status Register */
#define WDT_STAT_MASK                                       0x00000001
/* WDT Interrupt Clear Register */
#define WDT_EOI_MASK                                        0x00000001
/* WDT Protection level Register */
#define WDT_PROT_LEVEL_MASK                                 0x00000007

struct __attribute__((packed, aligned(4))) wdt_kendryte_regs_t
{
    /* WDT Control Register                     (0x00) */
    volatile uint32_t cr;
    /* WDT Timeout Range Register               (0x04) */
    volatile uint32_t torr;
    /* WDT Current Counter Value Register       (0x08) */
    volatile uint32_t ccvr;
    /* WDT Counter Restart Register             (0x0c) */
    volatile uint32_t crr;
    /* WDT Interrupt Status Register            (0x10) */
    volatile uint32_t stat;
    /* WDT Interrupt Clear Register             (0x14) */
    volatile uint32_t eoi;
    /* reserverd                                (0x18) */
    volatile uint32_t resv1;
    /* WDT Protection level Register            (0x1c) */
    volatile uint32_t prot_level;
    /* reserved                                 (0x20-0xe0) */
    volatile uint32_t resv4[49];
    /* WDT Component Parameters Register 5      (0xe4) */
    volatile uint32_t comp_param_5;
    /* WDT Component Parameters Register 4      (0xe8) */
    volatile uint32_t comp_param_4;
    /* WDT Component Parameters Register 3      (0xec) */
    volatile uint32_t comp_param_3;
    /* WDT Component Parameters Register 2      (0xf0) */
    volatile uint32_t comp_param_2;
    /* WDT Component Parameters Register 1      (0xf4) */
    volatile uint32_t comp_param_1;
    /* WDT Component Version Register           (0xf8) */
    volatile uint32_t comp_version;
    /* WDT Component Type Register              (0xfc) */
    volatile uint32_t comp_type;
};

/* Device constant configuration parameters */
struct wdt_kendryte_dev_cfg {
	u32_t *base;
	u32_t clock_id;
};

struct wdt_kendryte_data {
	struct device *clk_dev;
};

#define DEV_CFG(dev) \
	((const struct wdt_kendryte_dev_cfg *const)(dev)->config->config_info)
#define DEV_WDT(dev)						\
	((struct wdt_kendryte_regs_t *)(DEV_CFG(dev))->base)
#define DEV_DATA(dev)						\
	((struct wdt_kendryte_data * const)(dev)->driver_data)

static int wdt_kendryte_setup(struct device *dev, u8_t options)
{
	volatile struct wdt_kendryte_regs_t *wdt = DEV_WDT(dev);

	wdt->crr = WDT_CRR_MASK;
	wdt->cr |= WDT_CR_ENABLE;

	return 0;
}

static int wdt_kendryte_disable(struct device *dev)
{
	volatile struct wdt_kendryte_regs_t *wdt = DEV_WDT(dev);

	wdt->crr = WDT_CRR_MASK;
	wdt->cr &= (~WDT_CR_ENABLE);

	return 0;
}

static uint8_t wdt_get_top(struct device *dev, u64_t timeout_ms)
{
	const struct wdt_kendryte_dev_cfg * const dev_cfg = DEV_CFG(dev);
	struct wdt_kendryte_data *dev_data = DEV_DATA(dev);
	u32_t *rate = NULL;
	u64_t val;

	clock_control_get_rate(dev_data->clk_dev, (void *)dev_cfg->clock_id, rate);
	val = (timeout_ms * (*rate) / 1000) >> 16;

	if (val)
		val = (uint32_t)(31 - __builtin_clz(val));
	if (val > 0xf)
		val = 0xf;

	return (u8_t)val;
}


static int wdt_kendryte_install_timeout(struct device *dev,
				   const struct wdt_timeout_cfg *cfg)
{
	volatile struct wdt_kendryte_regs_t *wdt = DEV_WDT(dev);
	u8_t top_val;

	if (cfg->window.max == 0)
		return -EINVAL;

	top_val = wdt_get_top(dev, cfg->window.max);
	wdt->torr = WDT_TORR_TOP(top_val);

	/* 
	 * Api expects to return channel ID. But since there is no channel
	 * concept in this SoC, returning 1.
	 */
	return 1;
}

static int wdt_kendryte_feed(struct device *dev, int channel_id)
{
	volatile struct wdt_kendryte_regs_t *wdt = DEV_WDT(dev);

	wdt->crr = WDT_CRR_MASK;

	return 0;
}

static int wdt_kendryte_init(struct device *dev)
{
	const struct wdt_kendryte_dev_cfg * const dev_cfg = DEV_CFG(dev);
	struct wdt_kendryte_data *dev_data = DEV_DATA(dev);
	struct device *clk =
		device_get_binding(KENDRYTE_CLOCK_CONTROL_NAME);

	dev_data->clk_dev = clk;
	/* enable clock */
	clock_control_on(dev_data->clk_dev, (void *)dev_cfg->clock_id);

	/* TODO: reset */

	return 0;
}

static const struct wdt_driver_api wdt_kendryte_api = {
	.setup = wdt_kendryte_setup,
	.disable = wdt_kendryte_disable,
	.install_timeout = wdt_kendryte_install_timeout,
	.feed = wdt_kendryte_feed,
};

#ifdef CONFIG_WDT_KENDRYTE_0

struct wdt_kendryte_data wdt_kendryte_data0;

static const struct wdt_kendryte_dev_cfg wdt_kendryte_config_0 = {
	.base = (u32_t *) CONFIG_KENDRYTE_WDT_0_BASE_ADDR,
	.clock_id = KENDRYTE_CLOCK_WDT0,
};

DEVICE_AND_API_INIT(wdt_kendryte_0, CONFIG_KENDRYTE_WDT_0_LABEL, wdt_kendryte_init,
		    &wdt_kendryte_data0, &wdt_kendryte_config_0,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &wdt_kendryte_api);
#endif

#ifdef CONFIG_WDT_KENDRYTE_1

struct wdt_kendryte_data wdt_kendryte_data1;

static const struct wdt_kendryte_dev_cfg wdt_kendryte_config_1 = {
	.base = (u32_t *) CONFIG_KENDRYTE_WDT_1_BASE_ADDR,
	.clock_id = KENDRYTE_CLOCK_WDT1,
};

DEVICE_AND_API_INIT(wdt_kendryte_1, CONFIG_KENDRYTE_WDT_0_LABEL, wdt_kendryte_init,
		    &wdt_kendryte_data1, &wdt_kendryte_config_1,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &wdt_kendryte_api);
#endif
