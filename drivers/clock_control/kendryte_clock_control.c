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

#include <sys_io.h>

#include <clock_control.h>
#include <clock_control/kendryte_clock.h>

struct kendryte_clock_control_config {
	u64_t base;
};

u32_t kendryte_clock_source_get_freq(volatile kendryte_sysctl *sysctl,
				     kendryte_clock_source_t input);
u32_t kendryte_pll_get_freq(volatile kendryte_sysctl *sysctl, kendryte_pll_t pll);
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
