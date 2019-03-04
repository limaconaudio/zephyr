/* kendryte_clock.h - Clock controller header for Kendryte SoC */

/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_KENDRYTE_CLOCK_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_KENDRYTE_CLOCK_H_

#define KENDRYTE_CLOCK_CONTROL_NAME "kendryte-clock"

#define KENDRYTE_CLOCK_FREQ_IN0 (26000000UL)

typedef enum _kendryte_peripheral_clocks_t {
    	KENDRYTE_CLOCK_PLL0 = 0,
    	KENDRYTE_CLOCK_PLL1,
    	KENDRYTE_CLOCK_PLL2,
    	KENDRYTE_CLOCK_CPU,
    	KENDRYTE_CLOCK_SRAM0,
    	KENDRYTE_CLOCK_SRAM1,
    	KENDRYTE_CLOCK_APB0,
    	KENDRYTE_CLOCK_APB1,
    	KENDRYTE_CLOCK_APB2,
    	KENDRYTE_CLOCK_ROM,
    	KENDRYTE_CLOCK_DMA,
    	KENDRYTE_CLOCK_AI,
    	KENDRYTE_CLOCK_DVP,
    	KENDRYTE_CLOCK_FFT,
    	KENDRYTE_CLOCK_GPIO,
    	KENDRYTE_CLOCK_SPI0,
    	KENDRYTE_CLOCK_SPI1,
    	KENDRYTE_CLOCK_SPI2,
    	KENDRYTE_CLOCK_SPI3,
    	KENDRYTE_CLOCK_I2S0,
    	KENDRYTE_CLOCK_I2S1,
    	KENDRYTE_CLOCK_I2S2,
    	KENDRYTE_CLOCK_I2C0,
    	KENDRYTE_CLOCK_I2C1,
    	KENDRYTE_CLOCK_I2C2,
    	KENDRYTE_CLOCK_UART1,
    	KENDRYTE_CLOCK_UART2,
    	KENDRYTE_CLOCK_UART3,
    	KENDRYTE_CLOCK_AES,
    	KENDRYTE_CLOCK_FPIOA,
    	KENDRYTE_CLOCK_TIMER0,
    	KENDRYTE_CLOCK_TIMER1,
    	KENDRYTE_CLOCK_TIMER2,
    	KENDRYTE_CLOCK_WDT0,
    	KENDRYTE_CLOCK_WDT1,
    	KENDRYTE_CLOCK_SHA,
    	KENDRYTE_CLOCK_OTP,
    	KENDRYTE_CLOCK_RTC,
    	KENDRYTE_CLOCK_ACLK = 40,
    	KENDRYTE_CLOCK_HCLK,
    	KENDRYTE_CLOCK_IN0,
    	KENDRYTE_CLOCK_MAX
} kendryte_peripheral_clocks_t;

typedef enum _kendryte_clock_select_t
{
    KENDRYTE_CLOCK_SELECT_PLL0_BYPASS,
    KENDRYTE_CLOCK_SELECT_PLL1_BYPASS,
    KENDRYTE_CLOCK_SELECT_PLL2_BYPASS,
    KENDRYTE_CLOCK_SELECT_PLL2,
    KENDRYTE_CLOCK_SELECT_ACLK,
    KENDRYTE_CLOCK_SELECT_SPI3,
    KENDRYTE_CLOCK_SELECT_TIMER0,
    KENDRYTE_CLOCK_SELECT_TIMER1,
    KENDRYTE_CLOCK_SELECT_TIMER2,
    KENDRYTE_CLOCK_SELECT_SPI3_SAMPLE,
    KENDRYTE_CLOCK_SELECT_MAX = 11
} kendryte_clock_select_t;

typedef enum _kendryte_pll_t
{
    KENDRYTE_PLL0,
    KENDRYTE_PLL1,
    KENDRYTE_PLL2,
    KENDRYTE_PLL_MAX
} kendryte_pll_t;

typedef enum _kendryte_clock_source_t
{
    KENDRYTE_SOURCE_IN0,
    KENDRYTE_SOURCE_PLL0,
    KENDRYTE_SOURCE_PLL1,
    KENDRYTE_SOURCE_PLL2,
    KENDRYTE_SOURCE_ACLK,
    KENDRYTE_SOURCE_MAX
} kendryte_clock_source_t;

typedef enum _kendryte_threshold_t
{
    KENDRYTE_THRESHOLD_ACLK,
    KENDRYTE_THRESHOLD_APB0,
    KENDRYTE_THRESHOLD_APB1,
    KENDRYTE_THRESHOLD_APB2,
    KENDRYTE_THRESHOLD_SRAM0,
    KENDRYTE_THRESHOLD_SRAM1,
    KENDRYTE_THRESHOLD_AI,
    KENDRYTE_THRESHOLD_DVP,
    KENDRYTE_THRESHOLD_ROM,
    KENDRYTE_THRESHOLD_SPI0,
    KENDRYTE_THRESHOLD_SPI1,
    KENDRYTE_THRESHOLD_SPI2,
    KENDRYTE_THRESHOLD_SPI3,
    KENDRYTE_THRESHOLD_TIMER0,
    KENDRYTE_THRESHOLD_TIMER1,
    KENDRYTE_THRESHOLD_TIMER2,
    KENDRYTE_THRESHOLD_I2S0,
    KENDRYTE_THRESHOLD_I2S1,
    KENDRYTE_THRESHOLD_I2S2,
    KENDRYTE_THRESHOLD_I2S0_M,
    KENDRYTE_THRESHOLD_I2S1_M,
    KENDRYTE_THRESHOLD_I2S2_M,
    KENDRYTE_THRESHOLD_I2C0,
    KENDRYTE_THRESHOLD_I2C1,
    KENDRYTE_THRESHOLD_I2C2,
    KENDRYTE_THRESHOLD_WDT0,
    KENDRYTE_THRESHOLD_WDT1,
    KENDRYTE_THRESHOLD_MAX = 28
} kendryte_threshold_t;

/**
 * @brief      System controller reset control id
 */
typedef enum _sysctl_reset_t
{
    SYSCTL_RESET_SOC,
    SYSCTL_RESET_ROM,
    SYSCTL_RESET_DMA,
    SYSCTL_RESET_AI,
    SYSCTL_RESET_DVP,
    SYSCTL_RESET_FFT,
    SYSCTL_RESET_GPIO,
    SYSCTL_RESET_SPI0,
    SYSCTL_RESET_SPI1,
    SYSCTL_RESET_SPI2,
    SYSCTL_RESET_SPI3,
    SYSCTL_RESET_I2S0,
    SYSCTL_RESET_I2S1,
    SYSCTL_RESET_I2S2,
    SYSCTL_RESET_I2C0,
    SYSCTL_RESET_I2C1,
    SYSCTL_RESET_I2C2,
    SYSCTL_RESET_UART1,
    SYSCTL_RESET_UART2,
    SYSCTL_RESET_UART3,
    SYSCTL_RESET_AES,
    SYSCTL_RESET_FPIOA,
    SYSCTL_RESET_TIMER0,
    SYSCTL_RESET_TIMER1,
    SYSCTL_RESET_TIMER2,
    SYSCTL_RESET_WDT0,
    SYSCTL_RESET_WDT1,
    SYSCTL_RESET_SHA,
    SYSCTL_RESET_RTC,
    SYSCTL_RESET_MAX = 31
} sysctl_reset_t;

typedef enum _sysctl_dma_channel_t
{
    SYSCTL_DMA_CHANNEL_0,
    SYSCTL_DMA_CHANNEL_1,
    SYSCTL_DMA_CHANNEL_2,
    SYSCTL_DMA_CHANNEL_3,
    SYSCTL_DMA_CHANNEL_4,
    SYSCTL_DMA_CHANNEL_5,
    SYSCTL_DMA_CHANNEL_MAX
} sysctl_dma_channel_t;

typedef enum _sysctl_dma_select_t
{
    SYSCTL_DMA_SELECT_SSI0_RX_REQ,
    SYSCTL_DMA_SELECT_SSI0_TX_REQ,
    SYSCTL_DMA_SELECT_SSI1_RX_REQ,
    SYSCTL_DMA_SELECT_SSI1_TX_REQ,
    SYSCTL_DMA_SELECT_SSI2_RX_REQ,
    SYSCTL_DMA_SELECT_SSI2_TX_REQ,
    SYSCTL_DMA_SELECT_SSI3_RX_REQ,
    SYSCTL_DMA_SELECT_SSI3_TX_REQ,
    SYSCTL_DMA_SELECT_I2C0_RX_REQ,
    SYSCTL_DMA_SELECT_I2C0_TX_REQ,
    SYSCTL_DMA_SELECT_I2C1_RX_REQ,
    SYSCTL_DMA_SELECT_I2C1_TX_REQ,
    SYSCTL_DMA_SELECT_I2C2_RX_REQ,
    SYSCTL_DMA_SELECT_I2C2_TX_REQ,
    SYSCTL_DMA_SELECT_UART1_RX_REQ,
    SYSCTL_DMA_SELECT_UART1_TX_REQ,
    SYSCTL_DMA_SELECT_UART2_RX_REQ,
    SYSCTL_DMA_SELECT_UART2_TX_REQ,
    SYSCTL_DMA_SELECT_UART3_RX_REQ,
    SYSCTL_DMA_SELECT_UART3_TX_REQ,
    SYSCTL_DMA_SELECT_AES_REQ,
    SYSCTL_DMA_SELECT_SHA_RX_REQ,
    SYSCTL_DMA_SELECT_AI_RX_REQ,
    SYSCTL_DMA_SELECT_FFT_RX_REQ,
    SYSCTL_DMA_SELECT_FFT_TX_REQ,
    SYSCTL_DMA_SELECT_I2S0_TX_REQ,
    SYSCTL_DMA_SELECT_I2S0_RX_REQ,
    SYSCTL_DMA_SELECT_I2S1_TX_REQ,
    SYSCTL_DMA_SELECT_I2S1_RX_REQ,
    SYSCTL_DMA_SELECT_I2S2_TX_REQ,
    SYSCTL_DMA_SELECT_I2S2_RX_REQ,
    SYSCTL_DMA_SELECT_I2S0_BF_DIR_REQ,
    SYSCTL_DMA_SELECT_I2S0_BF_VOICE_REQ,
    SYSCTL_DMA_SELECT_MAX
} sysctl_dma_select_t;

/**
 * @brief       Git short commit id
 *
 *              No. 0 Register (0x00)
 */
typedef struct git_id
{
    volatile u32_t git_id : 32;
} __attribute__((packed, aligned(4))) sysctl_git_id_t;

/**
 * @brief       System clock base frequency
 *
 *              No. 1 Register (0x04)
 */
typedef struct clk_freq
{
    volatile u32_t clk_freq : 32;
} __attribute__((packed, aligned(4))) sysctl_clk_freq_t;

/**
 * @brief       PLL0 controller
 *
 *              No. 2 Register (0x08)
 */
typedef struct pll0
{
    volatile u32_t clkr0 : 4;
    volatile u32_t clkf0 : 6;
    volatile u32_t clkod0 : 4;
    volatile u32_t bwadj0 : 6;
    volatile u32_t pll_reset0 : 1;
    volatile u32_t pll_pwrd0 : 1;
    volatile u32_t pll_intfb0 : 1;
    volatile u32_t pll_bypass0 : 1;
    volatile u32_t pll_test0 : 1;
    volatile u32_t pll_out_en0 : 1;
    volatile u32_t pll_test_en : 1;
    volatile u32_t reserved : 5;
} __attribute__((packed, aligned(4))) sysctl_pll0_t;

/**
 * @brief       PLL1 controller
 *
 *              No. 3 Register (0x0c)
 */
typedef struct pll1
{
    volatile u32_t clkr1 : 4;
    volatile u32_t clkf1 : 6;
    volatile u32_t clkod1 : 4;
    volatile u32_t bwadj1 : 6;
    volatile u32_t pll_reset1 : 1;
    volatile u32_t pll_pwrd1 : 1;
    volatile u32_t pll_intfb1 : 1;
    volatile u32_t pll_bypass1 : 1;
    volatile u32_t pll_test1 : 1;
    volatile u32_t pll_out_en1 : 1;
    volatile u32_t reserved : 6;
} __attribute__((packed, aligned(4))) sysctl_pll1_t;

/**
 * @brief       PLL2 controller
 *
 *              No. 4 Register (0x10)
 */
typedef struct pll2
{
    volatile u32_t clkr2 : 4;
    volatile u32_t clkf2 : 6;
    volatile u32_t clkod2 : 4;
    volatile u32_t bwadj2 : 6;
    volatile u32_t pll_reset2 : 1;
    volatile u32_t pll_pwrd2 : 1;
    volatile u32_t pll_intfb2 : 1;
    volatile u32_t pll_bypass2 : 1;
    volatile u32_t pll_test2 : 1;
    volatile u32_t pll_out_en2 : 1;
    volatile u32_t pll_ckin_sel2 : 2;
    volatile u32_t reserved : 4;
} __attribute__((packed, aligned(4))) sysctl_pll2_t;

/**
 * @brief       PLL lock tester
 *
 *              No. 6 Register (0x18)
 */
typedef struct pll_lock
{
    volatile u32_t pll_lock0 : 2;
    volatile u32_t pll_slip_clear0 : 1;
    volatile u32_t test_clk_out0 : 1;
    volatile u32_t reserved0 : 4;
    volatile u32_t pll_lock1 : 2;
    volatile u32_t pll_slip_clear1 : 1;
    volatile u32_t test_clk_out1 : 1;
    volatile u32_t reserved1 : 4;
    volatile u32_t pll_lock2 : 2;
    volatile u32_t pll_slip_clear2 : 1;
    volatile u32_t test_clk_out2 : 1;
    volatile u32_t reserved2 : 12;
} __attribute__((packed, aligned(4))) sysctl_pll_lock_t;

/**
 * @brief       AXI ROM detector
 *
 *              No. 7 Register (0x1c)
 */
typedef struct rom_error
{
    volatile u32_t rom_mul_error : 1;
    volatile u32_t rom_one_error : 1;
    volatile u32_t reserved : 30;
} __attribute__((packed, aligned(4))) sysctl_rom_error_t;

/**
 * @brief       Clock select controller0
 *
 *              No. 8 Register (0x20)
 */
typedef struct clk_sel0
{
    volatile u32_t aclk_sel : 1;
    volatile u32_t aclk_divider_sel : 2;
    volatile u32_t apb0_clk_sel : 3;
    volatile u32_t apb1_clk_sel : 3;
    volatile u32_t apb2_clk_sel : 3;
    volatile u32_t spi3_clk_sel : 1;
    volatile u32_t timer0_clk_sel : 1;
    volatile u32_t timer1_clk_sel : 1;
    volatile u32_t timer2_clk_sel : 1;
    volatile u32_t reserved : 16;
} __attribute__((packed, aligned(4))) sysctl_clk_sel0_t;

/**
 * @brief       Clock select controller1
 *
 *              No. 9 Register (0x24)
 */
typedef struct clk_sel1
{
    volatile u32_t spi3_sample_clk_sel : 1;
    volatile u32_t reserved0 : 30;
    volatile u32_t reserved1 : 1;
} __attribute__((packed, aligned(4))) sysctl_clk_sel1_t;

/**
 * @brief       Central clock enable
 *
 *              No. 10 Register (0x28)
 */
typedef struct clk_en_cent
{
    volatile u32_t cpu_clk_en : 1;
    volatile u32_t sram0_clk_en : 1;
    volatile u32_t sram1_clk_en : 1;
    volatile u32_t apb0_clk_en : 1;
    volatile u32_t apb1_clk_en : 1;
    volatile u32_t apb2_clk_en : 1;
    volatile u32_t reserved : 26;
} __attribute__((packed, aligned(4))) sysctl_clk_en_cent_t;

/**
 * @brief       Peripheral clock enable
 *
 *              No. 11 Register (0x2c)
 */
typedef struct clk_en_peri
{
    volatile u32_t rom_clk_en : 1;
    volatile u32_t dma_clk_en : 1;
    volatile u32_t ai_clk_en : 1;
    volatile u32_t dvp_clk_en : 1;
    volatile u32_t fft_clk_en : 1;
    volatile u32_t gpio_clk_en : 1;
    volatile u32_t spi0_clk_en : 1;
    volatile u32_t spi1_clk_en : 1;
    volatile u32_t spi2_clk_en : 1;
    volatile u32_t spi3_clk_en : 1;
    volatile u32_t i2s0_clk_en : 1;
    volatile u32_t i2s1_clk_en : 1;
    volatile u32_t i2s2_clk_en : 1;
    volatile u32_t i2c0_clk_en : 1;
    volatile u32_t i2c1_clk_en : 1;
    volatile u32_t i2c2_clk_en : 1;
    volatile u32_t uart1_clk_en : 1;
    volatile u32_t uart2_clk_en : 1;
    volatile u32_t uart3_clk_en : 1;
    volatile u32_t aes_clk_en : 1;
    volatile u32_t fpioa_clk_en : 1;
    volatile u32_t timer0_clk_en : 1;
    volatile u32_t timer1_clk_en : 1;
    volatile u32_t timer2_clk_en : 1;
    volatile u32_t wdt0_clk_en : 1;
    volatile u32_t wdt1_clk_en : 1;
    volatile u32_t sha_clk_en : 1;
    volatile u32_t otp_clk_en : 1;
    volatile u32_t reserved : 1;
    volatile u32_t rtc_clk_en : 1;
    volatile u32_t reserved0 : 2;
} __attribute__((packed, aligned(4))) sysctl_clk_en_peri_t;

/**
 * @brief       Soft reset ctrl
 *
 *              No. 12 Register (0x30)
 */
typedef struct soft_reset
{
    volatile u32_t soft_reset : 1;
    volatile u32_t reserved : 31;
} __attribute__((packed, aligned(4))) sysctl_soft_reset_t;

/**
 * @brief       Peripheral reset controller
 *
 *              No. 13 Register (0x34)
 */
typedef struct peri_reset
{
    volatile u32_t rom_reset : 1;
    volatile u32_t dma_reset : 1;
    volatile u32_t ai_reset : 1;
    volatile u32_t dvp_reset : 1;
    volatile u32_t fft_reset : 1;
    volatile u32_t gpio_reset : 1;
    volatile u32_t spi0_reset : 1;
    volatile u32_t spi1_reset : 1;
    volatile u32_t spi2_reset : 1;
    volatile u32_t spi3_reset : 1;
    volatile u32_t i2s0_reset : 1;
    volatile u32_t i2s1_reset : 1;
    volatile u32_t i2s2_reset : 1;
    volatile u32_t i2c0_reset : 1;
    volatile u32_t i2c1_reset : 1;
    volatile u32_t i2c2_reset : 1;
    volatile u32_t uart1_reset : 1;
    volatile u32_t uart2_reset : 1;
    volatile u32_t uart3_reset : 1;
    volatile u32_t aes_reset : 1;
    volatile u32_t fpioa_reset : 1;
    volatile u32_t timer0_reset : 1;
    volatile u32_t timer1_reset : 1;
    volatile u32_t timer2_reset : 1;
    volatile u32_t wdt0_reset : 1;
    volatile u32_t wdt1_reset : 1;
    volatile u32_t sha_reset : 1;
    volatile u32_t reserved : 2;
    volatile u32_t rtc_reset : 1;
    volatile u32_t reserved0 : 2;
} __attribute__((packed, aligned(4))) sysctl_peri_reset_t;

/**
 * @brief       Clock threshold controller 0
 *
 *              No. 14 Register (0x38)
 */
typedef struct clk_th0
{
    volatile u32_t sram0_gclk_threshold : 4;
    volatile u32_t sram1_gclk_threshold : 4;
    volatile u32_t ai_gclk_threshold : 4;
    volatile u32_t dvp_gclk_threshold : 4;
    volatile u32_t rom_gclk_threshold : 4;
    volatile u32_t reserved : 12;
} __attribute__((packed, aligned(4))) sysctl_clk_th0_t;

/**
 * @brief       Clock threshold controller 1
 *
 *              No. 15 Register (0x3c)
 */
typedef struct clk_th1
{
    volatile u32_t spi0_clk_threshold : 8;
    volatile u32_t spi1_clk_threshold : 8;
    volatile u32_t spi2_clk_threshold : 8;
    volatile u32_t spi3_clk_threshold : 8;
} __attribute__((packed, aligned(4))) sysctl_clk_th1_t;

/**
 * @brief       Clock threshold controller 2
 *
 *              No. 16 Register (0x40)
 */
typedef struct clk_th2
{
    volatile u32_t timer0_clk_threshold : 8;
    volatile u32_t timer1_clk_threshold : 8;
    volatile u32_t timer2_clk_threshold : 8;
    volatile u32_t reserved : 8;
} __attribute__((packed, aligned(4))) sysctl_clk_th2_t;

/**
 * @brief       Clock threshold controller 3
 *
 *              No. 17 Register (0x44)
 */
typedef struct clk_th3
{
    volatile u32_t i2s0_clk_threshold : 16;
    volatile u32_t i2s1_clk_threshold : 16;
} __attribute__((packed, aligned(4))) sysctl_clk_th3_t;

/**
 * @brief       Clock threshold controller 4
 *
 *              No. 18 Register (0x48)
 */
typedef struct clk_th4
{
    volatile u32_t i2s2_clk_threshold : 16;
    volatile u32_t i2s0_mclk_threshold : 8;
    volatile u32_t i2s1_mclk_threshold : 8;
} __attribute__((packed, aligned(4))) sysctl_clk_th4_t;

/**
 * @brief       Clock threshold controller 5
 *
 *              No. 19 Register (0x4c)
 */
typedef struct clk_th5
{
    volatile u32_t i2s2_mclk_threshold : 8;
    volatile u32_t i2c0_clk_threshold : 8;
    volatile u32_t i2c1_clk_threshold : 8;
    volatile u32_t i2c2_clk_threshold : 8;
} __attribute__((packed, aligned(4))) sysctl_clk_th5_t;

/**
 * @brief       Clock threshold controller 6
 *
 *              No. 20 Register (0x50)
 */
typedef struct clk_th6
{
    volatile u32_t wdt0_clk_threshold : 8;
    volatile u32_t wdt1_clk_threshold : 8;
    volatile u32_t reserved0 : 8;
    volatile u32_t reserved1 : 8;
} __attribute__((packed, aligned(4))) sysctl_clk_th6_t;

/**
 * @brief       Miscellaneous controller
 *
 *              No. 21 Register (0x54)
 */
typedef struct misc
{
    volatile u32_t debug_sel : 6;
    volatile u32_t reserved0 : 4;
    volatile u32_t spi_dvp_data_enable: 1;
    volatile u32_t reserved1 : 21;
} __attribute__((packed, aligned(4))) sysctl_misc_t;

/**
 * @brief       Peripheral controller
 *
 *              No. 22 Register (0x58)
 */
typedef struct peri
{
    volatile u32_t timer0_pause : 1;
    volatile u32_t timer1_pause : 1;
    volatile u32_t timer2_pause : 1;
    volatile u32_t timer3_pause : 1;
    volatile u32_t timer4_pause : 1;
    volatile u32_t timer5_pause : 1;
    volatile u32_t timer6_pause : 1;
    volatile u32_t timer7_pause : 1;
    volatile u32_t timer8_pause : 1;
    volatile u32_t timer9_pause : 1;
    volatile u32_t timer10_pause : 1;
    volatile u32_t timer11_pause : 1;
    volatile u32_t spi0_xip_en : 1;
    volatile u32_t spi1_xip_en : 1;
    volatile u32_t spi2_xip_en : 1;
    volatile u32_t spi3_xip_en : 1;
    volatile u32_t spi0_clk_bypass : 1;
    volatile u32_t spi1_clk_bypass : 1;
    volatile u32_t spi2_clk_bypass : 1;
    volatile u32_t i2s0_clk_bypass : 1;
    volatile u32_t i2s1_clk_bypass : 1;
    volatile u32_t i2s2_clk_bypass : 1;
    volatile u32_t jtag_clk_bypass : 1;
    volatile u32_t dvp_clk_bypass : 1;
    volatile u32_t debug_clk_bypass : 1;
    volatile u32_t reserved0 : 1;
    volatile u32_t reserved1 : 6;
} __attribute__((packed, aligned(4))) sysctl_peri_t;

/**
 * @brief       SPI sleep controller
 *
 *              No. 23 Register (0x5c)
 */
typedef struct spi_sleep
{
    volatile u32_t ssi0_sleep : 1;
    volatile u32_t ssi1_sleep : 1;
    volatile u32_t ssi2_sleep : 1;
    volatile u32_t ssi3_sleep : 1;
    volatile u32_t reserved : 28;
} __attribute__((packed, aligned(4))) sysctl_spi_sleep_t;

/**
 * @brief       Reset source status
 *
 *              No. 24 Register (0x60)
 */
typedef struct reset_status
{
    volatile u32_t reset_sts_clr : 1;
    volatile u32_t pin_reset_sts : 1;
    volatile u32_t wdt0_reset_sts : 1;
    volatile u32_t wdt1_reset_sts : 1;
    volatile u32_t soft_reset_sts : 1;
    volatile u32_t reserved : 27;
} __attribute__((packed, aligned(4))) sysctl_reset_status_t;

/**
 * @brief       DMA handshake selector
 *
 *              No. 25 Register (0x64)
 */
typedef struct dma_sel0
{
    volatile u32_t dma_sel0 : 6;
    volatile u32_t dma_sel1 : 6;
    volatile u32_t dma_sel2 : 6;
    volatile u32_t dma_sel3 : 6;
    volatile u32_t dma_sel4 : 6;
    volatile u32_t reserved : 2;
} __attribute__((packed, aligned(4))) sysctl_dma_sel0_t;

/**
 * @brief       DMA handshake selector
 *
 *              No. 26 Register (0x68)
 */
typedef struct dma_sel1
{
    volatile u32_t dma_sel5 : 6;
    volatile u32_t reserved : 26;
} __attribute__((packed, aligned(4))) sysctl_dma_sel1_t;

/**
 * @brief       IO Power Mode Select controller
 *
 *              No. 27 Register (0x6c)
 */
typedef struct power_sel
{
    volatile u32_t power_mode_sel0 : 1;
    volatile u32_t power_mode_sel1 : 1;
    volatile u32_t power_mode_sel2 : 1;
    volatile u32_t power_mode_sel3 : 1;
    volatile u32_t power_mode_sel4 : 1;
    volatile u32_t power_mode_sel5 : 1;
    volatile u32_t power_mode_sel6 : 1;
    volatile u32_t power_mode_sel7 : 1;
    volatile u32_t reserved : 24;
} __attribute__((packed, aligned(4))) sysctl_power_sel_t;

/**
 * @brief       System controller object
 *
 *              The System controller is a peripheral device mapped in the
 *              internal memory map, discoverable in the Configuration String.
 *              It is responsible for low-level configuration of all system
 *              related peripheral device. It contain PLL controller, clock
 *              controller, reset controller, DMA handshake controller, SPI
 *              controller, timer controller, WDT controller and sleep
 *              controller.
 */
typedef struct _kendryte_sysctl
{
    /* No. 0 (0x00): Git short commit id */
    sysctl_git_id_t git_id;
    /* No. 1 (0x04): System clock base frequency */
    sysctl_clk_freq_t clk_freq;
    /* No. 2 (0x08): PLL0 controller */
    sysctl_pll0_t pll0;
    /* No. 3 (0x0c): PLL1 controller */
    sysctl_pll1_t pll1;
    /* No. 4 (0x10): PLL2 controller */
    sysctl_pll2_t pll2;
    /* No. 5 (0x14): Reserved */
    volatile u32_t resv5;
    /* No. 6 (0x18): PLL lock tester */
    sysctl_pll_lock_t pll_lock;
    /* No. 7 (0x1c): AXI ROM detector */
    sysctl_rom_error_t rom_error;
    /* No. 8 (0x20): Clock select controller0 */
    sysctl_clk_sel0_t clk_sel0;
    /* No. 9 (0x24): Clock select controller1 */
    sysctl_clk_sel1_t clk_sel1;
    /* No. 10 (0x28): Central clock enable */
    sysctl_clk_en_cent_t clk_en_cent;
    /* No. 11 (0x2c): Peripheral clock enable */
    sysctl_clk_en_peri_t clk_en_peri;
    /* No. 12 (0x30): Soft reset ctrl */
    sysctl_soft_reset_t soft_reset;
    /* No. 13 (0x34): Peripheral reset controller */
    sysctl_peri_reset_t peri_reset;
    /* No. 14 (0x38): Clock threshold controller 0 */
    sysctl_clk_th0_t clk_th0;
    /* No. 15 (0x3c): Clock threshold controller 1 */
    sysctl_clk_th1_t clk_th1;
    /* No. 16 (0x40): Clock threshold controller 2 */
    sysctl_clk_th2_t clk_th2;
    /* No. 17 (0x44): Clock threshold controller 3 */
    sysctl_clk_th3_t clk_th3;
    /* No. 18 (0x48): Clock threshold controller 4 */
    sysctl_clk_th4_t clk_th4;
    /* No. 19 (0x4c): Clock threshold controller 5 */
    sysctl_clk_th5_t clk_th5;
    /* No. 20 (0x50): Clock threshold controller 6 */
    sysctl_clk_th6_t clk_th6;
    /* No. 21 (0x54): Miscellaneous controller */
    sysctl_misc_t misc;
    /* No. 22 (0x58): Peripheral controller */
    sysctl_peri_t peri;
    /* No. 23 (0x5c): SPI sleep controller */
    sysctl_spi_sleep_t spi_sleep;
    /* No. 24 (0x60): Reset source status */
    sysctl_reset_status_t reset_status;
    /* No. 25 (0x64): DMA handshake selector */
    sysctl_dma_sel0_t dma_sel0;
    /* No. 26 (0x68): DMA handshake selector */
    sysctl_dma_sel1_t dma_sel1;
    /* No. 27 (0x6c): IO Power Mode Select controller */
    sysctl_power_sel_t power_sel;
    /* No. 28 (0x70): Reserved */
    volatile u32_t resv28;
    /* No. 29 (0x74): Reserved */
    volatile u32_t resv29;
    /* No. 30 (0x78): Reserved */
    volatile u32_t resv30;
    /* No. 31 (0x7c): Reserved */
    volatile u32_t resv31;
} __attribute__((packed, aligned(4))) kendryte_sysctl;

/**
 * @brief       Abstruct PLL struct
 */
typedef struct general_pll
{
    volatile u32_t clkr : 4;
    volatile u32_t clkf : 6;
    volatile u32_t clkod : 4;
    volatile u32_t bwadj : 6;
    volatile u32_t pll_reset : 1;
    volatile u32_t pll_pwrd : 1;
    volatile u32_t pll_intfb : 1;
    volatile u32_t pll_bypass : 1;
    volatile u32_t pll_test : 1;
    volatile u32_t pll_out_en : 1;
    volatile u32_t pll_ckin_sel : 2;
    volatile u32_t reserved : 4;
} __attribute__((packed, aligned(4))) sysctl_general_pll_t;

int kendryte_clock_set_threshold(struct device *dev,
			       kendryte_threshold_t thres, int threshold);
u32_t kendryte_pll_set_freq(volatile kendryte_sysctl *sysctl,
				kendryte_pll_t pll, u32_t pll_freq);

int sysctl_dma_select(struct device *dev,sysctl_dma_channel_t channel, sysctl_dma_select_t select);
void sysctl_reset(struct device *dev, sysctl_reset_t reset);

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_KENDRYTE_CLOCK_H_ */
