/* kendryte_clock_control.c - Clock controller driver for Kendryte SoC */

/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>

#include <misc/__assert.h>
#include <board.h>
#include <device.h>
#include <init.h>
#include <math.h>

#include <sys_io.h>

#include <clock_control.h>
#include <clock_control/kendryte_clock.h>

struct kendryte_clock_control_config {
	u64_t base;
};

u32_t kendryte_clock_source_get_freq(volatile kendryte_sysctl *sysctl,
				     kendryte_clock_source_t input);
u32_t kendryte_pll_get_freq(volatile kendryte_sysctl *sysctl, kendryte_pll_t pll);
u32_t kendryte_pll_set_freq(volatile kendryte_sysctl *sysctl,
				kendryte_pll_t pll, u32_t pll_freq);
u32_t kendryte_clock_get_freq(volatile kendryte_sysctl *sysctl,
			      kendryte_peripheral_clocks_t clock);

const u8_t get_select_pll2[] = {
    [KENDRYTE_SOURCE_IN0] = 0,
    [KENDRYTE_SOURCE_PLL0] = 1,
    [KENDRYTE_SOURCE_PLL1] = 2,
};

const u8_t get_source_pll2[] = {
    [0] = KENDRYTE_SOURCE_IN0,
    [1] = KENDRYTE_SOURCE_PLL0,
    [2] = KENDRYTE_SOURCE_PLL1,
};

const u8_t get_select_aclk[] = {
    [KENDRYTE_SOURCE_IN0] = 0,
    [KENDRYTE_SOURCE_PLL0] = 1,
};

const u8_t get_source_aclk[] = {
    [0] = KENDRYTE_SOURCE_IN0,
    [1] = KENDRYTE_SOURCE_PLL0,
};

static void sysctl_reset_ctl(volatile kendryte_sysctl *sysctl,
		sysctl_reset_t reset, uint8_t rst_value)
{
    switch (reset)
    {
        case SYSCTL_RESET_SOC:
            sysctl->soft_reset.soft_reset = rst_value;
            break;
        case SYSCTL_RESET_ROM:
            sysctl->peri_reset.rom_reset = rst_value;
            break;
        case SYSCTL_RESET_DMA:
            sysctl->peri_reset.dma_reset = rst_value;
            break;
        case SYSCTL_RESET_AI:
            sysctl->peri_reset.ai_reset = rst_value;
            break;
        case SYSCTL_RESET_DVP:
            sysctl->peri_reset.dvp_reset = rst_value;
            break;
        case SYSCTL_RESET_FFT:
            sysctl->peri_reset.fft_reset = rst_value;
            break;
        case SYSCTL_RESET_GPIO:
            sysctl->peri_reset.gpio_reset = rst_value;
            break;
        case SYSCTL_RESET_SPI0:
            sysctl->peri_reset.spi0_reset = rst_value;
            break;
        case SYSCTL_RESET_SPI1:
            sysctl->peri_reset.spi1_reset = rst_value;
            break;
        case SYSCTL_RESET_SPI2:
            sysctl->peri_reset.spi2_reset = rst_value;
            break;
        case SYSCTL_RESET_SPI3:
            sysctl->peri_reset.spi3_reset = rst_value;
            break;
        case SYSCTL_RESET_I2S0:
            sysctl->peri_reset.i2s0_reset = rst_value;
            break;
        case SYSCTL_RESET_I2S1:
            sysctl->peri_reset.i2s1_reset = rst_value;
            break;
        case SYSCTL_RESET_I2S2:
            sysctl->peri_reset.i2s2_reset = rst_value;
            break;
        case SYSCTL_RESET_I2C0:
            sysctl->peri_reset.i2c0_reset = rst_value;
            break;
        case SYSCTL_RESET_I2C1:
            sysctl->peri_reset.i2c1_reset = rst_value;
            break;
        case SYSCTL_RESET_I2C2:
            sysctl->peri_reset.i2c2_reset = rst_value;
            break;
        case SYSCTL_RESET_UART1:
            sysctl->peri_reset.uart1_reset = rst_value;
            break;
        case SYSCTL_RESET_UART2:
            sysctl->peri_reset.uart2_reset = rst_value;
            break;
        case SYSCTL_RESET_UART3:
            sysctl->peri_reset.uart3_reset = rst_value;
            break;
        case SYSCTL_RESET_AES:
            sysctl->peri_reset.aes_reset = rst_value;
            break;
        case SYSCTL_RESET_FPIOA:
            sysctl->peri_reset.fpioa_reset = rst_value;
            break;
        case SYSCTL_RESET_TIMER0:
            sysctl->peri_reset.timer0_reset = rst_value;
            break;
        case SYSCTL_RESET_TIMER1:
            sysctl->peri_reset.timer1_reset = rst_value;
            break;
        case SYSCTL_RESET_TIMER2:
            sysctl->peri_reset.timer2_reset = rst_value;
            break;
        case SYSCTL_RESET_WDT0:
            sysctl->peri_reset.wdt0_reset = rst_value;
            break;
        case SYSCTL_RESET_WDT1:
            sysctl->peri_reset.wdt1_reset = rst_value;
            break;
        case SYSCTL_RESET_SHA:
            sysctl->peri_reset.sha_reset = rst_value;
            break;
        case SYSCTL_RESET_RTC:
            sysctl->peri_reset.rtc_reset = rst_value;
            break;

        default:
            break;
    }
}

void sysctl_reset(struct device *dev, sysctl_reset_t reset)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	volatile kendryte_sysctl *sysctl = (volatile kendryte_sysctl *)info->base;

	sysctl_reset_ctl(sysctl, reset, 1);
	sysctl_reset_ctl(sysctl, reset, 0);
}

int sysctl_dma_select(struct device *dev, sysctl_dma_channel_t channel,
		sysctl_dma_select_t select)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	volatile kendryte_sysctl *sysctl = (volatile kendryte_sysctl *)info->base;

	sysctl_dma_sel0_t dma_sel0;
	sysctl_dma_sel1_t dma_sel1;

	/* Read register from bus */
	dma_sel0 = sysctl->dma_sel0;
	dma_sel1 = sysctl->dma_sel1;
	switch (channel)
	{
        case SYSCTL_DMA_CHANNEL_0:
            dma_sel0.dma_sel0 = select;
            break;

        case SYSCTL_DMA_CHANNEL_1:
            dma_sel0.dma_sel1 = select;
            break;

        case SYSCTL_DMA_CHANNEL_2:
            dma_sel0.dma_sel2 = select;
            break;

        case SYSCTL_DMA_CHANNEL_3:
            dma_sel0.dma_sel3 = select;
            break;

        case SYSCTL_DMA_CHANNEL_4:
            dma_sel0.dma_sel4 = select;
            break;

        case SYSCTL_DMA_CHANNEL_5:
            dma_sel1.dma_sel5 = select;
            break;

        default:
            return -1;
	}

	/* Write register back to bus */
	sysctl->dma_sel0 = dma_sel0;
	sysctl->dma_sel1 = dma_sel1;

	return 0;
}


static int kendryte_clock_bus_enable(struct device *dev,
				   clock_control_subsys_t sub_system,
				   u8_t enable)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	volatile kendryte_sysctl *sysctl = (volatile kendryte_sysctl *)info->base;
	u32_t subsys = POINTER_TO_UINT(sub_system);

        switch (subsys) {
            /*
             * These peripheral devices are under APB0
             * GPIO, UART1, UART2, UART3, SPI_SLAVE, I2S0, I2S1,
             * I2S2, I2C0, I2C1, I2C2, FPIOA, SHA256, TIMER0,
             * TIMER1, TIMER2
             */
        case KENDRYTE_CLOCK_GPIO:
        case KENDRYTE_CLOCK_SPI2:
        case KENDRYTE_CLOCK_I2S0:
        case KENDRYTE_CLOCK_I2S1:
        case KENDRYTE_CLOCK_I2S2:
        case KENDRYTE_CLOCK_I2C0:
        case KENDRYTE_CLOCK_I2C1:
        case KENDRYTE_CLOCK_I2C2:
        case KENDRYTE_CLOCK_UART1:
        case KENDRYTE_CLOCK_UART2:
        case KENDRYTE_CLOCK_UART3:
        case KENDRYTE_CLOCK_FPIOA:
        case KENDRYTE_CLOCK_TIMER0:
        case KENDRYTE_CLOCK_TIMER1:
        case KENDRYTE_CLOCK_TIMER2:
        case KENDRYTE_CLOCK_SHA:
            sysctl->clk_en_cent.apb0_clk_en = enable;
            break;

            /*
             * These peripheral devices are under APB1
             * WDT, AES, OTP, DVP, KENDRYTE
             */
        case KENDRYTE_CLOCK_AES:
        case KENDRYTE_CLOCK_WDT0:
        case KENDRYTE_CLOCK_WDT1:
        case KENDRYTE_CLOCK_OTP:
        case KENDRYTE_CLOCK_RTC:
            sysctl->clk_en_cent.apb1_clk_en = enable;
            break;

            /*
             * These peripheral devices are under APB2
             * SPI0, SPI1
             */
        case KENDRYTE_CLOCK_SPI0:
        case KENDRYTE_CLOCK_SPI1:
            sysctl->clk_en_cent.apb2_clk_en = enable;
            break;

        default:
            return -EINVAL;
        }

	return 0;
}

static int kendryte_device_bus_enable(struct device *dev,
				    clock_control_subsys_t sub_system,
				    u8_t enable)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	volatile kendryte_sysctl *sysctl = (volatile kendryte_sysctl *)info->base;
	u32_t subsys = POINTER_TO_UINT(sub_system);

        switch (subsys) {
        /*
         * These devices are PLL
         */
	case KENDRYTE_CLOCK_PLL0:
        	sysctl->pll0.pll_out_en0 = enable;
        	break;
	case KENDRYTE_CLOCK_PLL1:
       		sysctl->pll1.pll_out_en1 = enable;
        	break;
	case KENDRYTE_CLOCK_PLL2:
        	sysctl->pll2.pll_out_en2 = enable;
        	break;

        /*
         * These devices are CPU, SRAM, APB bus, ROM, DMA, AI
         */
	case KENDRYTE_CLOCK_CPU:
        	sysctl->clk_en_cent.cpu_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_SRAM0:
        	sysctl->clk_en_cent.sram0_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_SRAM1:
        	sysctl->clk_en_cent.sram1_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_APB0:
        	sysctl->clk_en_cent.apb0_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_APB1:
        	sysctl->clk_en_cent.apb1_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_APB2:
        	sysctl->clk_en_cent.apb2_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_ROM:
        	sysctl->clk_en_peri.rom_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_DMA:
        	sysctl->clk_en_peri.dma_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_AI:
        	sysctl->clk_en_peri.ai_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_DVP:
        	sysctl->clk_en_peri.dvp_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_FFT:
        	sysctl->clk_en_peri.fft_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_SPI3:
        	sysctl->clk_en_peri.spi3_clk_en = enable;
        	break;

        /*
         * These peripheral devices are under APB0
         * GPIO, UART1, UART2, UART3, SPI_SLAVE, I2S0, I2S1,
         * I2S2, I2C0, I2C1, I2C2, FPIOA, SHA256, TIMER0,
         * TIMER1, TIMER2
         */
	case KENDRYTE_CLOCK_GPIO:
        	sysctl->clk_en_peri.gpio_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_SPI2:
        	sysctl->clk_en_peri.spi2_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_I2S0:
        	sysctl->clk_en_peri.i2s0_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_I2S1:
        	sysctl->clk_en_peri.i2s1_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_I2S2:
        	sysctl->clk_en_peri.i2s2_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_I2C0:
        	sysctl->clk_en_peri.i2c0_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_I2C1:
        	sysctl->clk_en_peri.i2c1_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_I2C2:
        	sysctl->clk_en_peri.i2c2_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_UART1:
        	sysctl->clk_en_peri.uart1_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_UART2:
        	sysctl->clk_en_peri.uart2_clk_en = enable;
       		break;
	case KENDRYTE_CLOCK_UART3:
        	sysctl->clk_en_peri.uart3_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_FPIOA:
        	sysctl->clk_en_peri.fpioa_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_TIMER0:
        	sysctl->clk_en_peri.timer0_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_TIMER1:
        	sysctl->clk_en_peri.timer1_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_TIMER2:
        	sysctl->clk_en_peri.timer2_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_SHA:
        	sysctl->clk_en_peri.sha_clk_en = enable;
        	break;

        /*
         * These peripheral devices are under APB1
         * WDT, AES, OTP, DVP, KENDRYTE
         */
	case KENDRYTE_CLOCK_AES:
        	sysctl->clk_en_peri.aes_clk_en = enable;
       		break;
	case KENDRYTE_CLOCK_WDT0:
        	sysctl->clk_en_peri.wdt0_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_WDT1:
        	sysctl->clk_en_peri.wdt1_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_OTP:
        	sysctl->clk_en_peri.otp_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_RTC:
        	sysctl->clk_en_peri.rtc_clk_en = enable;
        	break;

        /*
         * These peripheral devices are under APB2
         * SPI0, SPI1
         */
	case KENDRYTE_CLOCK_SPI0:
        	sysctl->clk_en_peri.spi0_clk_en = enable;
        	break;
	case KENDRYTE_CLOCK_SPI1:
        	sysctl->clk_en_peri.spi1_clk_en = enable;
        	break;

	default:
        	return -EINVAL;
	}

	return 0;
}

static int kendryte_clock_control_on(struct device *dev,
				     clock_control_subsys_t sub_system)
{
	int ret;

	ret = kendryte_clock_bus_enable(dev, sub_system, 1);
	if (ret < 0)
		return ret;

	ret = kendryte_device_bus_enable(dev, sub_system, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int kendryte_clock_control_off(struct device *dev,
				     clock_control_subsys_t sub_system)
{
	int ret;

	ret = kendryte_clock_bus_enable(dev, sub_system, 0);
	if (ret < 0)
		return ret;

	ret = kendryte_device_bus_enable(dev, sub_system, 0);
	if (ret < 0)
		return ret;

	return 0;
}


int kendryte_clock_get_clock_select(volatile kendryte_sysctl *sysctl,
				    kendryte_clock_select_t sel)
{
    int clock_select = 0;

    switch (sel)
    {
        /*
         * Select and get clock select value
         */
    case KENDRYTE_CLOCK_SELECT_PLL0_BYPASS:
        clock_select = (int)sysctl->pll0.pll_bypass0;
        break;
    case KENDRYTE_CLOCK_SELECT_PLL1_BYPASS:
        clock_select = (int)sysctl->pll1.pll_bypass1;
        break;
    case KENDRYTE_CLOCK_SELECT_PLL2_BYPASS:
        clock_select = (int)sysctl->pll2.pll_bypass2;
        break;
    case KENDRYTE_CLOCK_SELECT_PLL2:
        clock_select = (int)sysctl->pll2.pll_ckin_sel2;
        break;
    case KENDRYTE_CLOCK_SELECT_ACLK:
        clock_select = (int)sysctl->clk_sel0.aclk_sel;
        break;
    case KENDRYTE_CLOCK_SELECT_SPI3:
        clock_select = (int)sysctl->clk_sel0.spi3_clk_sel;
        break;
    case KENDRYTE_CLOCK_SELECT_TIMER0:
        clock_select = (int)sysctl->clk_sel0.timer0_clk_sel;
        break;
    case KENDRYTE_CLOCK_SELECT_TIMER1:
        clock_select = (int)sysctl->clk_sel0.timer1_clk_sel;
        break;
    case KENDRYTE_CLOCK_SELECT_TIMER2:
        clock_select = (int)sysctl->clk_sel0.timer2_clk_sel;
        break;
    case KENDRYTE_CLOCK_SELECT_SPI3_SAMPLE:
        clock_select = (int)sysctl->clk_sel1.spi3_sample_clk_sel;
        break;

    default:
        break;
    }

    return clock_select;
}


int sysctl_clock_set_clock_select(volatile kendryte_sysctl *sysctl,
				kendryte_clock_select_t which, int select)
{
    int result = 0;
    switch (which)
    {
        /*
         * This clock select is 1 bit width
         */
        case KENDRYTE_CLOCK_SELECT_PLL0_BYPASS:
            sysctl->pll0.pll_bypass0 = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_PLL1_BYPASS:
            sysctl->pll1.pll_bypass1 = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_PLL2_BYPASS:
            sysctl->pll2.pll_bypass2 = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_ACLK:
            sysctl->clk_sel0.aclk_sel = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_SPI3:
            sysctl->clk_sel0.spi3_clk_sel = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_TIMER0:
            sysctl->clk_sel0.timer0_clk_sel = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_TIMER1:
            sysctl->clk_sel0.timer1_clk_sel = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_TIMER2:
            sysctl->clk_sel0.timer2_clk_sel = select & 0x01;
            break;
        case KENDRYTE_CLOCK_SELECT_SPI3_SAMPLE:
            sysctl->clk_sel1.spi3_sample_clk_sel = select & 0x01;
            break;

        /*
         * These clock select is 2 bit width
         */
        case KENDRYTE_CLOCK_SELECT_PLL2:
            sysctl->pll2.pll_ckin_sel2 = select & 0x03;
            break;

        default:
            result = -1;
            break;
    }

    return result;
}

static int sysctl_pll_is_lock(volatile kendryte_sysctl *sysctl, kendryte_pll_t pll)
{
    /*
     * All bit enable means PLL lock
     *
     * struct pll_lock_t
     * {
     *         uint8_t overflow : 1;
     *         uint8_t rfslip : 1;
     *         uint8_t fbslip : 1;
     * };
     *
     */

    if (pll >= KENDRYTE_PLL_MAX)
        return 0;

    switch (pll)
    {
        case KENDRYTE_PLL0:
            return sysctl->pll_lock.pll_lock0 == 3;

        case KENDRYTE_PLL1:
            return sysctl->pll_lock.pll_lock1 & 1;

        case KENDRYTE_PLL2:
            return sysctl->pll_lock.pll_lock2 & 1;

        default:
            break;
    }

    return 0;
}

static int sysctl_pll_clear_slip(volatile kendryte_sysctl *sysctl, kendryte_pll_t pll)
{
    if (pll >= KENDRYTE_PLL_MAX)
        return -1;

    switch (pll)
    {
        case KENDRYTE_PLL0:
            sysctl->pll_lock.pll_slip_clear0 = 1;
            break;

        case KENDRYTE_PLL1:
            sysctl->pll_lock.pll_slip_clear1 = 1;
            break;

        case KENDRYTE_PLL2:
            sysctl->pll_lock.pll_slip_clear2 = 1;
            break;

        default:
            break;
    }

    return sysctl_pll_is_lock(sysctl, pll) ? 0 : -1;
}

static uint32_t sysctl_pll_source_set_freq(volatile kendryte_sysctl *sysctl,
			kendryte_pll_t pll, kendryte_clock_source_t source, uint32_t freq)
{
    uint32_t freq_in = 0;

    if (pll >= KENDRYTE_PLL_MAX)
        return 0;

    if (source >= KENDRYTE_SOURCE_MAX)
        return 0;

    switch (pll)
    {
        case KENDRYTE_PLL0:
        case KENDRYTE_PLL1:
            /*
             * Check input clock source
             */
            if (source != KENDRYTE_SOURCE_IN0)
                return 0;
            freq_in = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            /*
             * Check input clock freq
             */
            if (freq_in == 0)
                return 0;
            break;

        case KENDRYTE_PLL2:
            /*
             * Check input clock source
             */
            if (source < sizeof(get_select_pll2))
                freq_in = kendryte_clock_source_get_freq(sysctl, source);
            /*
             * Check input clock freq
             */
            if (freq_in == 0)
                return 0;
            break;

        default:
            return 0;
    }

    /*
     * Begin calculate PLL registers' value
     */

    /* constants */
    const double vco_min = 3.5e+08;
    const double vco_max = 1.75e+09;
    const double ref_min = 1.36719e+07;
    const double ref_max = 1.75e+09;
    const int nr_min     = 1;
    const int nr_max     = 16;
    const int nf_min     = 1;
    const int nf_max     = 64;
    const int no_min     = 1;
    const int no_max     = 16;
    const int nb_min     = 1;
    const int nb_max     = 64;
    const int max_vco    = 1;
    const int ref_rng    = 1;

    /* variables */
    int nr     = 0;
    int nrx    = 0;
    int nf     = 0;
    int nfi    = 0;
    int no     = 0;
    int noe    = 0;
    int not    = 0;
    int nor    = 0;
    int nore   = 0;
    int nb     = 0;
    int first  = 0;
    int firstx = 0;
    int found  = 0;

    long long nfx = 0;
    double fin = 0, fout = 0, fvco = 0;
    double val = 0, nval = 0, err = 0, merr = 0, terr = 0;
    int x_nrx = 0, x_no = 0, x_nb = 0;
    long long x_nfx = 0;
    double x_fvco = 0, x_err = 0;

    fin   = freq_in;
    fout  = freq;
    val   = fout / fin;
    terr  = 0.5 / ((double)(nf_max / 2));
    first = firstx = 1;
    if (terr != -2)
    {
        first = 0;
        if (terr == 0)
            terr = 1e-16;
        merr = fabs(terr);
    }
    found = 0;
    for (nfi = val; nfi < nf_max; ++nfi)
    {
        nr = rint(((double)nfi) / val);
        if (nr == 0)
            continue;
        if ((ref_rng) && (nr < nr_min))
            continue;
        if (fin / ((double)nr) > ref_max)
            continue;
        nrx = nr;
        nf = nfx = nfi;
        nval = ((double)nfx) / ((double)nr);
        if (nf == 0)
            nf = 1;
        err = 1 - nval / val;

        if ((first) || (fabs(err) < merr * (1 + 1e-6)) || (fabs(err) < 1e-16))
        {
            not = floor(vco_max / fout);
            for (no = (not > no_max) ? no_max : not; no > no_min; --no)
            {
                if ((ref_rng) && ((nr / no) < nr_min))
                    continue;
                if ((nr % no) == 0)
                    break;
            }
            if ((nr % no) != 0)
                continue;
            nor  = ((not > no_max) ? no_max : not) / no;
            nore = nf_max / nf;
            if (nor > nore)
                nor = nore;
            noe  = ceil(vco_min / fout);
            if (!max_vco)
            {
                nore = (noe - 1) / no + 1;
                nor  = nore;
                not  = 0; /* force next if to fail */
            }
            if ((((no * nor) < (not >> 1)) || ((no * nor) < noe)) && ((no * nor) < (nf_max / nf)))
            {
                no = nf_max / nf;
                if (no > no_max)
                    no = no_max;
                if (no > not)
                    no = not;
                nfx *= no;
                nf *= no;
                if ((no > 1) && (!firstx))
                    continue;
                /* wait for larger nf in later iterations */
            }
            else
            {
                nrx /= no;
                nfx *= nor;
                nf *= nor;
                no *= nor;
                if (no > no_max)
                    continue;
                if ((nor > 1) && (!firstx))
                    continue;
                /* wait for larger nf in later iterations */
            }

            nb = nfx;
            if (nb < nb_min)
                nb = nb_min;
            if (nb > nb_max)
                continue;

            fvco = fin / ((double)nrx) * ((double)nfx);
            if (fvco < vco_min)
                continue;
            if (fvco > vco_max)
                continue;
            if (nf < nf_min)
                continue;
            if ((ref_rng) && (fin / ((double)nrx) < ref_min))
                continue;
            if ((ref_rng) && (nrx > nr_max))
                continue;
            if (!(((firstx) && (terr < 0)) || (fabs(err) < merr * (1 - 1e-6)) || ((max_vco) && (no > x_no))))
                continue;
            if ((!firstx) && (terr >= 0) && (nrx > x_nrx))
                continue;

            found  = 1;
            x_no   = no;
            x_nrx  = nrx;
            x_nfx  = nfx;
            x_nb   = nb;
            x_fvco = fvco;
            x_err  = err;
            first  = firstx = 0;
            merr   = fabs(err);
            if (terr != -1)
                continue;
        }
    }
    if (!found)
    {
        return 0;
    }

    nrx  = x_nrx;
    nfx  = x_nfx;
    no   = x_no;
    nb   = x_nb;
    fvco = x_fvco;
    err  = x_err;
    if ((terr != -2) && (fabs(err) >= terr * (1 - 1e-6)))
    {
        return 0;
    }

    /*
     * Begin write PLL registers' value,
     * Using atomic write method.
     */
    sysctl_pll0_t pll0;
    sysctl_pll1_t pll1;
    sysctl_pll2_t pll2;

    switch (pll)
    {
        case KENDRYTE_PLL0:
            /* Read register from bus */
            pll0 = sysctl->pll0;
            /* Set register temporary value */
            pll0.clkr0  = nrx - 1;
            pll0.clkf0  = nfx - 1;
            pll0.clkod0 = no - 1;
            pll0.bwadj0 = nb - 1;
            /* Write register back to bus */
            sysctl->pll0 = pll0;
            break;

        case KENDRYTE_PLL1:
            /* Read register from bus */
            pll1 = sysctl->pll1;
            /* Set register temporary value */
            pll1.clkr1  = nrx - 1;
            pll1.clkf1  = nfx - 1;
            pll1.clkod1 = no - 1;
            pll1.bwadj1 = nb - 1;
            /* Write register back to bus */
            sysctl->pll1 = pll1;
            break;

        case KENDRYTE_PLL2:
            /* Read register from bus */
            pll2 = sysctl->pll2;
            /* Set register temporary value */
            if (source < sizeof(get_select_pll2))
                pll2.pll_ckin_sel2 = get_select_pll2[source];

            pll2.clkr2  = nrx - 1;
            pll2.clkf2  = nfx - 1;
            pll2.clkod2 = no - 1;
            pll2.bwadj2 = nb - 1;
            /* Write register back to bus */
            sysctl->pll2 = pll2;
            break;

        default:
            return 0;
    }

    return kendryte_pll_get_freq(sysctl, pll);
}

u32_t kendryte_pll_set_freq(volatile kendryte_sysctl *sysctl, kendryte_pll_t pll,
				u32_t pll_freq)
{
    if(pll_freq == 0)
        return 0;

    volatile sysctl_general_pll_t *v_pll_t;
    switch(pll)
    {
        case KENDRYTE_PLL0:
            v_pll_t = (sysctl_general_pll_t *)(&sysctl->pll0);
            break;
        case KENDRYTE_PLL1:
            v_pll_t = (sysctl_general_pll_t *)(&sysctl->pll1);
            break;
        case KENDRYTE_PLL2:
            v_pll_t = (sysctl_general_pll_t *)(&sysctl->pll2);
            break;
        default:
            return 0;
            break;
    }

    /* 1. Change CPU CLK to XTAL */
    if(pll == KENDRYTE_PLL0)
        sysctl_clock_set_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_ACLK, KENDRYTE_SOURCE_IN0);

    /* 2. Disable PLL output */
    v_pll_t->pll_out_en = 0;

    /* 3. Turn off PLL */
    v_pll_t->pll_pwrd = 0;

    /* 4. Set PLL new value */
    u32_t result;
    if(pll == KENDRYTE_PLL2)
        result = sysctl_pll_source_set_freq(sysctl, pll, v_pll_t->pll_ckin_sel, pll_freq);
    else
        result = sysctl_pll_source_set_freq(sysctl, pll, KENDRYTE_SOURCE_IN0, pll_freq);

    /* 5. Power on PLL */
    v_pll_t->pll_pwrd = 1;
    /* wait >100ns */
    k_sleep(1);

    /* 6. Reset PLL then Release Reset*/
    v_pll_t->pll_reset = 0;
    v_pll_t->pll_reset = 1;
    /* wait >100ns */
    k_sleep(1);
    v_pll_t->pll_reset = 0;

    /* 7. Get lock status, wait PLL stable */
    while (sysctl_pll_is_lock(sysctl, pll) == 0)
        sysctl_pll_clear_slip(sysctl, pll);

    /* 8. Enable PLL output */
    v_pll_t->pll_out_en = 1;

    /* 9. Change CPU CLK to PLL */
    if(pll == KENDRYTE_PLL0)
        sysctl_clock_set_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_ACLK, KENDRYTE_SOURCE_PLL0);

    return result;
}

u32_t kendryte_pll_get_freq(volatile kendryte_sysctl *sysctl, kendryte_pll_t pll)
{
    u32_t freq_in = 0, freq_out = 0;
    u32_t nr = 0, nf = 0, od = 0;
    u8_t select = 0;

    if (pll >= KENDRYTE_PLL_MAX)
        return 0;

    switch (pll)
    {
    case KENDRYTE_PLL0:
        freq_in = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
        nr = sysctl->pll0.clkr0 + 1;
        nf = sysctl->pll0.clkf0 + 1;
        od = sysctl->pll0.clkod0 + 1;
        break;

    case KENDRYTE_PLL1:
        freq_in = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
        nr = sysctl->pll1.clkr1 + 1;
        nf = sysctl->pll1.clkf1 + 1;
        od = sysctl->pll1.clkod1 + 1;
        break;

    case KENDRYTE_PLL2:
        /*
         * Get input freq accroding select register
         */
        select = sysctl->pll2.pll_ckin_sel2;
        if (select < sizeof(get_source_pll2))
            freq_in = kendryte_clock_source_get_freq(sysctl, get_source_pll2[select]);
        else
            return 0;

        nr = sysctl->pll2.clkr2 + 1;
        nf = sysctl->pll2.clkf2 + 1;
        od = sysctl->pll2.clkod2 + 1;
        break;

    default:
        break;
    }

    /*
     * Get final PLL output freq
     * FOUT = FIN / NR * NF / OD
     */
    freq_out = freq_in /( nr * nf) / od;
    return freq_out;
}

u32_t kendryte_clock_source_get_freq(volatile kendryte_sysctl *sysctl,
				     kendryte_clock_source_t input)
{
    u32_t result;

    switch (input)
    {
    case KENDRYTE_SOURCE_IN0:
        result = KENDRYTE_CLOCK_FREQ_IN0;
        break;
    case KENDRYTE_SOURCE_PLL0:
        result = kendryte_pll_get_freq(sysctl, KENDRYTE_PLL0);
        break;
    case KENDRYTE_SOURCE_PLL1:
        result = kendryte_pll_get_freq(sysctl, KENDRYTE_PLL1);
        break;
    case KENDRYTE_SOURCE_PLL2:
        result = kendryte_pll_get_freq(sysctl, KENDRYTE_PLL2);
        break;
    case KENDRYTE_SOURCE_ACLK:
        result = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_ACLK);
        break;
    default:
        result = 0;
        break;
    }
    return result;
}

int kendryte_clock_set_threshold(struct device *dev,
			       kendryte_threshold_t thres, int threshold)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	volatile kendryte_sysctl *sysctl = (volatile kendryte_sysctl *)info->base;

	switch (thres) {
	/*
	 * These threshold is 2 bit width
	 */
	case KENDRYTE_THRESHOLD_ACLK:
	    sysctl->clk_sel0.aclk_divider_sel = (uint8_t)threshold & 0x03;
	    break;

	/*
	 * These threshold is 3 bit width
	 */
        case KENDRYTE_THRESHOLD_APB0:
            sysctl->clk_sel0.apb0_clk_sel = (uint8_t)threshold & 0x07;
            break;
        case KENDRYTE_THRESHOLD_APB1:
            sysctl->clk_sel0.apb1_clk_sel = (uint8_t)threshold & 0x07;
            break;
        case KENDRYTE_THRESHOLD_APB2:
            sysctl->clk_sel0.apb2_clk_sel = (uint8_t)threshold & 0x07;
            break;

        /*
         * These threshold is 4 bit width
         */
        case KENDRYTE_THRESHOLD_SRAM0:
            sysctl->clk_th0.sram0_gclk_threshold = (uint8_t)threshold & 0x0F;
            break;
        case KENDRYTE_THRESHOLD_SRAM1:
            sysctl->clk_th0.sram1_gclk_threshold = (uint8_t)threshold & 0x0F;
            break;
        case KENDRYTE_THRESHOLD_AI:
            sysctl->clk_th0.ai_gclk_threshold = (uint8_t)threshold & 0x0F;
            break;
        case KENDRYTE_THRESHOLD_DVP:
            sysctl->clk_th0.dvp_gclk_threshold = (uint8_t)threshold & 0x0F;
            break;
        case KENDRYTE_THRESHOLD_ROM:
            sysctl->clk_th0.rom_gclk_threshold = (uint8_t)threshold & 0x0F;
            break;

        /*
         * These threshold is 8 bit width
         */
        case KENDRYTE_THRESHOLD_SPI0:
            sysctl->clk_th1.spi0_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_SPI1:
            sysctl->clk_th1.spi1_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_SPI2:
            sysctl->clk_th1.spi2_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_SPI3:
            sysctl->clk_th1.spi3_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_TIMER0:
            sysctl->clk_th2.timer0_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_TIMER1:
            sysctl->clk_th2.timer1_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_TIMER2:
            sysctl->clk_th2.timer2_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2S0_M:
            sysctl->clk_th4.i2s0_mclk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2S1_M:
            sysctl->clk_th4.i2s1_mclk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2S2_M:
            sysctl->clk_th5.i2s2_mclk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2C0:
            sysctl->clk_th5.i2c0_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2C1:
            sysctl->clk_th5.i2c1_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2C2:
            sysctl->clk_th5.i2c2_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_WDT0:
            sysctl->clk_th6.wdt0_clk_threshold = (uint8_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_WDT1:
            sysctl->clk_th6.wdt1_clk_threshold = (uint8_t)threshold;
            break;

        /*
         * These threshold is 16 bit width
         */
        case KENDRYTE_THRESHOLD_I2S0:
            sysctl->clk_th3.i2s0_clk_threshold = (uint16_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2S1:
            sysctl->clk_th3.i2s1_clk_threshold = (uint16_t)threshold;
            break;
        case KENDRYTE_THRESHOLD_I2S2:
            sysctl->clk_th4.i2s2_clk_threshold = (uint16_t)threshold;
            break;

        default:
            return -1;
    }

    return 0;
}

int kendryte_clock_get_threshold(volatile kendryte_sysctl *sysctl,
				 kendryte_threshold_t thres)
{
    int threshold = 0;

    switch (thres)
    {
        /*
         * Select and get threshold value
         */
    case KENDRYTE_THRESHOLD_ACLK:
        threshold = (int)sysctl->clk_sel0.aclk_divider_sel;
        break;
    case KENDRYTE_THRESHOLD_APB0:
        threshold = (int)sysctl->clk_sel0.apb0_clk_sel;
        break;
    case KENDRYTE_THRESHOLD_APB1:
        threshold = (int)sysctl->clk_sel0.apb1_clk_sel;
        break;
    case KENDRYTE_THRESHOLD_APB2:
        threshold = (int)sysctl->clk_sel0.apb2_clk_sel;
        break;
    case KENDRYTE_THRESHOLD_SRAM0:
        threshold = (int)sysctl->clk_th0.sram0_gclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_SRAM1:
        threshold = (int)sysctl->clk_th0.sram1_gclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_AI:
        threshold = (int)sysctl->clk_th0.ai_gclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_DVP:
        threshold = (int)sysctl->clk_th0.dvp_gclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_ROM:
        threshold = (int)sysctl->clk_th0.rom_gclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_SPI0:
        threshold = (int)sysctl->clk_th1.spi0_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_SPI1:
        threshold = (int)sysctl->clk_th1.spi1_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_SPI2:
        threshold = (int)sysctl->clk_th1.spi2_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_SPI3:
        threshold = (int)sysctl->clk_th1.spi3_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_TIMER0:
        threshold = (int)sysctl->clk_th2.timer0_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_TIMER1:
        threshold = (int)sysctl->clk_th2.timer1_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_TIMER2:
        threshold = (int)sysctl->clk_th2.timer2_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2S0:
        threshold = (int)sysctl->clk_th3.i2s0_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2S1:
        threshold = (int)sysctl->clk_th3.i2s1_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2S2:
        threshold = (int)sysctl->clk_th4.i2s2_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2S0_M:
        threshold = (int)sysctl->clk_th4.i2s0_mclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2S1_M:
        threshold = (int)sysctl->clk_th4.i2s1_mclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2S2_M:
        threshold = (int)sysctl->clk_th5.i2s2_mclk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2C0:
        threshold = (int)sysctl->clk_th5.i2c0_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2C1:
        threshold = (int)sysctl->clk_th5.i2c1_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_I2C2:
        threshold = (int)sysctl->clk_th5.i2c2_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_WDT0:
        threshold = (int)sysctl->clk_th6.wdt0_clk_threshold;
        break;
    case KENDRYTE_THRESHOLD_WDT1:
        threshold = (int)sysctl->clk_th6.wdt1_clk_threshold;
        break;

    default:
        break;
    }

    return threshold;
}

u32_t kendryte_clock_get_freq(volatile kendryte_sysctl *sysctl,
				 kendryte_peripheral_clocks_t clock)
{
    u32_t source = 0;
    u32_t result = 0;

    switch (clock)
    {
        /*
         * The clock IN0
         */
    case KENDRYTE_CLOCK_IN0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
        result = source;
        break;

        /*
         * These clock directly under PLL clock domain
         * They are using gated divider.
         */
    case KENDRYTE_CLOCK_PLL0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
        result = source;
        break;
    case KENDRYTE_CLOCK_PLL1:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL1);
        result = source;
        break;
    case KENDRYTE_CLOCK_PLL2:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL2);
        result = source;
        break;

        /*
         * These clock directly under ACLK clock domain
         */
    case KENDRYTE_CLOCK_CPU:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_ACLK))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0) / (2ULL << kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_ACLK));
            break;
        default:
            break;
        }
        result = source;
        break;
    case KENDRYTE_CLOCK_DMA:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_ACLK))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0) / (2ULL << kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_ACLK));
            break;
        default:
            break;
        }
        result = source;
        break;
    case KENDRYTE_CLOCK_FFT:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_ACLK))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0) / (2ULL << kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_ACLK));
            break;
        default:
            break;
        }
        result = source;
        break;
    case KENDRYTE_CLOCK_ACLK:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_ACLK))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0) / (2ULL << kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_ACLK));
            break;
        default:
            break;
        }
        result = source;
        break;
    case KENDRYTE_CLOCK_HCLK:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_ACLK))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0) / (2ULL << kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_ACLK));
            break;
        default:
            break;
        }
        result = source;
        break;

        /*
         * These clock under ACLK clock domain.
         * They are using gated divider.
         */
    case KENDRYTE_CLOCK_SRAM0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_ACLK);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_SRAM0) + 1);
        break;
    case KENDRYTE_CLOCK_SRAM1:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_ACLK);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_SRAM1) + 1);
        break;
    case KENDRYTE_CLOCK_ROM:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_ACLK);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_ROM) + 1);
        break;
    case KENDRYTE_CLOCK_DVP:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_ACLK);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_DVP) + 1);
        break;

        /*
         * These clock under ACLK clock domain.
         * They are using even divider.
         */
    case KENDRYTE_CLOCK_APB0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_ACLK);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_APB0) + 1);
        break;
    case KENDRYTE_CLOCK_APB1:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_ACLK);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_APB1) + 1);
        break;
    case KENDRYTE_CLOCK_APB2:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_ACLK);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_APB2) + 1);
        break;

        /*
         * These clock under AI clock domain.
         * They are using gated divider.
         */
    case KENDRYTE_CLOCK_AI:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL1);
        result = source / (kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_AI) + 1);
        break;

        /*
         * These clock under I2S clock domain.
         * They are using even divider.
         */
    case KENDRYTE_CLOCK_I2S0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL2);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_I2S0) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_I2S1:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL2);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_I2S1) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_I2S2:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL2);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_I2S2) + 1) * 2);
        break;

        /*
         * These clock under WDT clock domain.
         * They are using even divider.
         */
    case KENDRYTE_CLOCK_WDT0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_WDT0) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_WDT1:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_WDT1) + 1) * 2);
        break;

        /*
         * These clock under PLL0 clock domain.
         * They are using even divider.
         */
    case KENDRYTE_CLOCK_SPI0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_SPI0) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_SPI1:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_SPI1) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_SPI2:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_SPI2) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_I2C0:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_I2C0) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_I2C1:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_I2C1) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_I2C2:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_I2C2) + 1) * 2);
        break;

        /*
         * These clock under PLL0_SEL clock domain.
         * They are using even divider.
         */
    case KENDRYTE_CLOCK_SPI3:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_SPI3))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
            break;
        default:
            break;
        }

        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_SPI3) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_TIMER0:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_TIMER0))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
            break;
        default:
            break;
        }

        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_TIMER0) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_TIMER1:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_TIMER1))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
            break;
        default:
            break;
        }

        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_TIMER1) + 1) * 2);
        break;
    case KENDRYTE_CLOCK_TIMER2:
        switch (kendryte_clock_get_clock_select(sysctl, KENDRYTE_CLOCK_SELECT_TIMER2))
        {
        case 0:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
            break;
        case 1:
            source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_PLL0);
            break;
        default:
            break;
        }

        result = source / ((kendryte_clock_get_threshold(sysctl, KENDRYTE_THRESHOLD_TIMER2) + 1) * 2);
        break;

        /*
         * These clock under MISC clock domain.
         * They are using even divider.
         */

        /*
          * These clock under APB0 clock domain.
          * They are using even divider.
          */
    case KENDRYTE_CLOCK_GPIO:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB0);
        result = source;
        break;
    case KENDRYTE_CLOCK_UART1:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB0);
        result = source;
        break;
    case KENDRYTE_CLOCK_UART2:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB0);
        result = source;
        break;
    case KENDRYTE_CLOCK_UART3:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB0);
        result = source;
        break;
    case KENDRYTE_CLOCK_FPIOA:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB0);
        result = source;
        break;
    case KENDRYTE_CLOCK_SHA:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB0);
        result = source;
        break;

        /*
         * These clock under APB1 clock domain.
         * They are using even divider.
         */
    case KENDRYTE_CLOCK_AES:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB1);
        result = source;
        break;
    case KENDRYTE_CLOCK_OTP:
        source = kendryte_clock_get_freq(sysctl, KENDRYTE_CLOCK_APB1);
        result = source;
        break;
    case KENDRYTE_CLOCK_RTC:
        source = kendryte_clock_source_get_freq(sysctl, KENDRYTE_SOURCE_IN0);
        result = source;
        break;

        /*
         * These clock under APB2 clock domain.
         * They are using even divider.
         */
        /*
          * Do nothing.
          */
    default:
        break;
    }
    return result;
}

static int kendryte_clock_control_get_rate(struct device *dev,
					 clock_control_subsys_t sub_system,
					 u32_t *rate)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	volatile kendryte_sysctl *sysctl = (volatile kendryte_sysctl *)info->base;
	u32_t subsys = POINTER_TO_UINT(sub_system);

	*rate = kendryte_clock_get_freq(sysctl, subsys);

	return 0;
}
static const struct clock_control_driver_api kendryte_clock_control_api = {
	.on = kendryte_clock_control_on,
	.off = kendryte_clock_control_off,
	.get_rate = kendryte_clock_control_get_rate,
};

int kendryte_clock_control_init(struct device *dev)
{
	return 0;
}

static struct kendryte_clock_control_config clock_kendryte_config = {
	.base = CONFIG_KENDRYTE_SYSCTL_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(clock_kendryte,
		    CONFIG_KENDRYTE_SYSCTL_NAME,
		    &kendryte_clock_control_init,
		    NULL, &clock_kendryte_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &kendryte_clock_control_api);
