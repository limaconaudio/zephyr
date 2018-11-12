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
	u32_t *base;
};

static int sysctl_clock_bus_enable(struct device *dev,
				   clock_control_subsys_t sub_system,
				   bool enable)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	kendryte_sysctl *sysctl = (struct _kendryte_sysctl *)info->base;
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

static int sysctl_device_bus_enable(struct device *dev,
				    clock_control_subsys_t sub_system,
				    bool enable)
{
	const struct kendryte_clock_control_config *info =
					dev->config->config_info;
	kendryte_sysctl *sysctl = (struct _kendryte_sysctl *)info->base;
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

	ret = sysctl_clock_bus_enable(dev, sub_system, true);
	if (ret < 0)
		return ret;

	ret = sysctl_device_bus_enable(dev, sub_system, true);
	if (ret < 0)
		return ret;

	return 0;
}

static int kendryte_clock_control_off(struct device *dev,
				     clock_control_subsys_t sub_system)
{
	int ret;

	ret = sysctl_clock_bus_enable(dev, sub_system, false);
	if (ret < 0)
		return ret;

	ret = sysctl_device_bus_enable(dev, sub_system, false);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct clock_control_driver_api kendryte_clock_control_api = {
	.on = kendryte_clock_control_on,
	.off = kendryte_clock_control_off,
	.get_rate = NULL,
};

int kendryte_clock_control_init(struct device *dev)
{
	return 0;
}

static struct kendryte_clock_control_config clock_kendryte_config = {
	.base = (u32_t *)SYSCTL_BASE_ADDR
};

DEVICE_AND_API_INIT(clock_kendryte,
		    KENDRYTE_CLOCK_CONTROL_NAME,
		    &kendryte_clock_control_init,
		    NULL, &clock_kendryte_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &kendryte_clock_control_api);
