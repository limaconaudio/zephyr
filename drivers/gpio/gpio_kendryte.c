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
};

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
