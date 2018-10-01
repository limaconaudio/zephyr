/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file GPIO driver for spinalhdl SoC
 */

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <misc/util.h>

#include "gpio_utils.h"

/* spinalhdl GPIO register-set structure */
struct gpio_spinalhdl_t {
	unsigned int write_en;
	unsigned int write;
	unsigned int read;
};

struct gpio_spinalhdl_config {
	u32_t gpio_base_addr;
};

/* Helper Macros for GPIO */
#define DEV_GPIO_CFG(dev)						\
	((const struct gpio_spinalhdl_config * const)(dev)->config->config_info)
#define DEV_GPIO(dev)							\
	((volatile struct gpio_spinalhdl_t *)(DEV_GPIO_CFG(dev))->gpio_base_addr)

#define GPIO_PINS 32

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
static int gpio_spinalhdl_config(struct device *dev,
			     int access_op,
			     u32_t pin,
			     int flags)
{
	volatile struct gpio_spinalhdl_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= GPIO_PINS)
		return -EINVAL;

	/* Configure gpio direction */
	if (flags & GPIO_DIR_OUT)
		gpio->write_en |= BIT(pin);
	else
		gpio->write_en &= ~BIT(pin);

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
static int gpio_spinalhdl_write(struct device *dev,
			    int access_op,
			    u32_t pin,
			    u32_t value)
{
	volatile struct gpio_spinalhdl_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= GPIO_PINS)
		return -EINVAL;

	/* If pin is configured as input return with error */
	if (gpio->write_en & BIT(pin))
		return -EINVAL;

	if (value)
		gpio->write |= BIT(pin);
	else
		gpio->write &= ~BIT(pin);

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
static int gpio_spinalhdl_read(struct device *dev,
			   int access_op,
			   u32_t pin,
			   u32_t *value)
{
	volatile struct gpio_spinalhdl_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= GPIO_PINS)
		return -EINVAL;

	/*
	 * If gpio is configured as output,
	 * read gpio value from out_val register,
	 * otherwise read gpio value from in_val register
	 */
	if (gpio->write & BIT(pin))
		*value = !!(gpio->write & BIT(pin));
	else
		*value = !!(gpio->read & BIT(pin));

	return 0;
}

static const struct gpio_driver_api gpio_spinalhdl_driver = {
	.config              = gpio_spinalhdl_config,
	.write               = gpio_spinalhdl_write,
	.read                = gpio_spinalhdl_read,
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
static int gpio_spinalhdl_init(struct device *dev)
{
	volatile struct gpio_spinalhdl_t *gpio = DEV_GPIO(dev);

	/* Setup GPIO pins to be Input to save power */
	gpio->write_en = 0;
	gpio->write   = 0;

	return 0;
}

static const struct gpio_spinalhdl_config gpio_spinalhdl_config0 = {
	.gpio_base_addr    = spinalhdl_GPIO_0_BASE_ADDR,
};

DEVICE_AND_API_INIT(gpio_spinalhdl_0, CONFIG_GPIO_spinalhdl_GPIO_NAME,
		    gpio_spinalhdl_init,
		    NULL, &gpio_spinalhdl_config0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_spinalhdl_driver);
