/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file GPIO driver for Kendtyte K210 SoC
 */

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <misc/util.h>
#include <clock_control.h>

#include <clock_control/kendryte_clock.h>
#include "gpio_utils.h"

struct gpio_kendryte_t {
	unsigned int out;
	unsigned int dir;
	unsigned int source;
	unsigned int res_3_11[9];
	unsigned int int_en;
	unsigned int int_mask;
	unsigned int int_level;
	unsigned int int_pol;
	unsigned int int_status;
	unsigned int int_status_raw;
	unsigned int int_deb;
	unsigned int int_clr;
	unsigned int in;
	unsigned int res_21_23[3];
	unsigned int sync_level;
	unsigned int id_code;
	unsigned int int_both;
}__attribute__((packed, aligned(4)));

struct gpio_kendryte_config {
	u32_t            gpio_base_addr;
};

/* Helper Macros for GPIO */
#define DEV_GPIO_CFG(dev)						\
	((const struct gpio_kendryte_config * const)(dev)->config->config_info)
#define DEV_GPIO(dev)							\
	((volatile struct gpio_kendryte_t *)(DEV_GPIO_CFG(dev))->gpio_base_addr)

/**
 * @brief Configure pin
 *
 * @param dev Device structure
 * @param access_op Access operation
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_kendryte_cfg(struct device *dev,
			     int access_op,
			     u32_t pin,
			     int flags)
{
	volatile struct gpio_kendryte_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= KENDRYTE_MAX_GPIO)
		return -EINVAL;

	/* Configure gpio direction */
	if (flags & GPIO_DIR_OUT) {
		gpio->dir |= BIT(pin);
	} else {
		gpio->dir &= ~BIT(pin);

		/*
		 * No pull-up configuration for now
		 */
	}

	return 0;
}

/**
 * @brief Set the pin
 *
 * @param dev Device struct
 * @param access_op Access operation
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_kendryte_write(struct device *dev,
			    int access_op,
			    u32_t pin,
			    u32_t value)
{
	volatile struct gpio_kendryte_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= KENDRYTE_MAX_GPIO)
		return -EINVAL;

	/* If pin is configured as input return with error */
	if (!(gpio->dir & BIT(pin)))
		return -EINVAL;

	if (value)
		gpio->out |= BIT(pin);
	else
		gpio->out &= ~BIT(pin);

	return 0;
}

/**
 * @brief Read the pin
 *
 * @param dev Device struct
 * @param access_op Access operation
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_kendryte_read(struct device *dev,
			   int access_op,
			   u32_t pin,
			   u32_t *value)
{
	volatile struct gpio_kendryte_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= KENDRYTE_MAX_GPIO)
		return -EINVAL;

	/*
	 * If gpio is configured as output,
	 * read gpio value from out register,
	 * otherwise read gpio value from in register
	 */
	if (gpio->dir & BIT(pin))
		*value = !!(gpio->out & BIT(pin));
	else
		*value = !!(gpio->in & BIT(pin));

	return 0;
}

static const struct gpio_driver_api gpio_kendryte_driver = {
	.config              = gpio_kendryte_cfg,
	.write               = gpio_kendryte_write,
	.read                = gpio_kendryte_read,
};

/**
 * @brief Initialize a GPIO controller
 *
 * Perform basic initialization of a GPIO controller
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpio_kendryte_init(struct device *dev)
{
	volatile struct gpio_kendryte_t *gpio = DEV_GPIO(dev);
	struct device *clk =
		device_get_binding(CONFIG_KENDRYTE_SYSCTL_NAME);

	/* enable clock */
	clock_control_on(clk, (void *)KENDRYTE_CLOCK_GPIO);

	/* Ensure that all gpio registers are reset to 0 initially */
	gpio->in   = 0;
	gpio->out  = 0;

	return 0;
}

static const struct gpio_kendryte_config gpio_kendryte_config0 = {
	.gpio_base_addr    = CONFIG_KENDRYTE_GPIO_BASE_ADDR,
};

DEVICE_AND_API_INIT(gpio_kendryte, CONFIG_GPIO_KENDRYTE_GPIO_NAME,
		    gpio_kendryte_init,
		    NULL, &gpio_kendryte_config0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_kendryte_driver);

#ifdef CONFIG_GPIOHS_KENDRYTE

struct gpiohs_kendryte_t
{
    /* Address offset 0x00, Input Values */
    u32_t input_val;
    /* Address offset 0x04, Input enable */
    u32_t input_en;
    /* Address offset 0x08, Output enable */
    u32_t output_en;
    /* Address offset 0x0c, Onput Values */
    u32_t output_val;
    /* Address offset 0x10, Internal Pull-Ups enable */
    u32_t pullup_en;
    /* Address offset 0x14, Drive Strength */
    u32_t drive;
    /* Address offset 0x18, Rise interrupt enable */
    u32_t rise_ie;
    /* Address offset 0x1c, Rise interrupt pending */
    u32_t rise_ip;
    /* Address offset 0x20, Fall interrupt enable */
    u32_t fall_ie;
    /* Address offset 0x24, Fall interrupt pending */
    u32_t fall_ip;
    /* Address offset 0x28, High interrupt enable */
    u32_t high_ie;
    /* Address offset 0x2c, High interrupt pending */
    u32_t high_ip;
    /* Address offset 0x30, Low interrupt enable */
    u32_t low_ie;
    /* Address offset 0x34, Low interrupt pending */
    u32_t low_ip;
    /* Address offset 0x38, HW I/O Function enable */
    u32_t iof_en;
    /* Address offset 0x3c, HW I/O Function select */
    u32_t iof_sel;
    /* Address offset 0x40, Output XOR (invert) */
    u32_t output_xor;
}__attribute__((packed, aligned(4)));

struct gpiohs_kendryte_config {
	u32_t            gpiohs_base_addr;
};

/* Helper Macros for GPIO */
#define DEV_GPIOHS_CFG(dev)						\
	((const struct gpiohs_kendryte_config * const)(dev)->config->config_info)
#define DEV_GPIOHS(dev)							\
	((volatile struct gpiohs_kendryte_t *)(DEV_GPIOHS_CFG(dev))->gpiohs_base_addr)

/**
 * @brief Configure pin
 *
 * @param dev Device structure
 * @param access_op Access operation
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpiohs_kendryte_cfg(struct device *dev,
			     int access_op,
			     u32_t pin,
			     int flags)
{
	volatile struct gpiohs_kendryte_t *gpiohs = DEV_GPIOHS(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= KENDRYTE_MAX_GPIO)
		return -EINVAL;

	/* Configure gpio direction */
	if (flags & GPIO_DIR_OUT) {
		gpiohs->output_en |= BIT(pin);
		gpiohs->input_en &= ~BIT(pin);
	} else {
		gpiohs->output_en &= ~BIT(pin);
		gpiohs->input_en |= BIT(pin);

		/*
		 * No pull-up configuration for now
		 */
	}

	return 0;
}

/**
 * @brief Set the pin
 *
 * @param dev Device struct
 * @param access_op Access operation
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpiohs_kendryte_write(struct device *dev,
			    int access_op,
			    u32_t pin,
			    u32_t value)
{
	volatile struct gpiohs_kendryte_t *gpiohs = DEV_GPIOHS(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= KENDRYTE_MAX_GPIO)
		return -EINVAL;

	/* If pin is configured as input return with error */
	if ((gpiohs->input_en & BIT(pin)))
		return -EINVAL;

	if (value)
		gpiohs->output_val |= BIT(pin);
	else
		gpiohs->output_val &= ~BIT(pin);

	return 0;
}

/**
 * @brief Read the pin
 *
 * @param dev Device struct
 * @param access_op Access operation
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpiohs_kendryte_read(struct device *dev,
			   int access_op,
			   u32_t pin,
			   u32_t *value)
{
	volatile struct gpiohs_kendryte_t *gpiohs = DEV_GPIOHS(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= KENDRYTE_MAX_GPIO)
		return -EINVAL;

	/*
	 * If gpio is configured as output,
	 * read gpio value from out register,
	 * otherwise read gpio value from in register
	 */
	if (gpiohs->output_en & BIT(pin))
		*value = !!(gpiohs->output_val & BIT(pin));
	else
		*value = !!(gpiohs->input_val & BIT(pin));

	return 0;
}

static const struct gpio_driver_api gpiohs_kendryte_driver = {
	.config              = gpiohs_kendryte_cfg,
	.write               = gpiohs_kendryte_write,
	.read                = gpiohs_kendryte_read,
};

/**
 * @brief Initialize a GPIO controller
 *
 * Perform basic initialization of a GPIO controller
 *
 * @param dev GPIO device struct
 *
 * @return 0
 */
static int gpiohs_kendryte_init(struct device *dev)
{
	volatile struct gpiohs_kendryte_t *gpiohs = DEV_GPIOHS(dev);
	struct device *clk =
		device_get_binding(CONFIG_KENDRYTE_SYSCTL_NAME);

	/* enable clock */
	clock_control_on(clk, (void *)KENDRYTE_CLOCK_GPIO);

	/* Ensure that all gpio registers are reset to 0 initially */
	gpiohs->input_val = 0;
	gpiohs->output_val = 0;

	return 0;
}

static const struct gpiohs_kendryte_config gpiohs_kendryte_config0 = {
	.gpiohs_base_addr    = CONFIG_KENDRYTE_GPIOHS_BASE_ADDR,
};

DEVICE_AND_API_INIT(gpiohs_kendryte, CONFIG_GPIOHS_KENDRYTE_GPIO_NAME,
		    gpiohs_kendryte_init,
		    NULL, &gpiohs_kendryte_config0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpiohs_kendryte_driver);
#endif
