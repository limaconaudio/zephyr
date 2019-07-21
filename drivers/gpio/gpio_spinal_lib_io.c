#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <misc/util.h>

#include "gpio_utils.h"

#include <gpio/gpio_spinal_lib_io.h>

typedef void (*spinal_lib_io_cfg_func_t)(void);


/* Helper Macros for GPIO */
#define DEV_GPIO_CFG(dev)						\
	((const struct gpio_spinal_lib_io_config * const)(dev)->config->config_info)
#define DEV_GPIO(dev)							\
	((volatile struct gpio_spinal_lib_io_t *)(DEV_GPIO_CFG(dev))->gpio_base_addr)
#define DEV_GPIO_DATA(dev)				\
	((struct gpio_spinal_lib_io_data *)(dev)->driver_data)


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
static int gpio_spinal_lib_io_config(struct device *dev,
			     int access_op,
			     u32_t pin,
			     int flags)
{
	volatile struct gpio_spinal_lib_io_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= 32)
		return -EINVAL;

	/* Configure gpio direction */
	if (flags & GPIO_DIR_OUT) {
		gpio->OUTPUT_ENABLE |= BIT(pin);
	} else {
		gpio->OUTPUT_ENABLE &= ~BIT(pin);
	}

	/*
	 * Configure interrupt if GPIO_INT is set.
	 * Here, we just configure the gpio interrupt behavior,
	 * we do not enable/disable interrupt for a particular
	 * gpio.
	 * Interrupt for a gpio is:
	 * 1) enabled only via a call to gpio_spinal_lib_io_enable_callback.
	 * 2) disabled only via a call to gpio_spinal_lib_io_disabled_callback.
	 */
	if (!(flags & GPIO_INT))
		return 0;


	/* Edge or Level triggered ? */
	if (flags & GPIO_INT_EDGE) {
		gpio->INTERRUPT_ENABLE_HIGH &= ~BIT(pin);
		gpio->INTERRUPT_ENABLE_LOW &= ~BIT(pin);

		/* Rising Edge, Falling Edge or Double Edge ? */
		if (flags & GPIO_INT_DOUBLE_EDGE) {
			gpio->INTERRUPT_ENABLE_RISE |= BIT(pin);
			gpio->INTERRUPT_ENABLE_FALL |= BIT(pin);
		} else if (flags & GPIO_INT_ACTIVE_HIGH) {
			gpio->INTERRUPT_ENABLE_RISE |= BIT(pin);
			gpio->INTERRUPT_ENABLE_FALL &= ~BIT(pin);
		} else {
			gpio->INTERRUPT_ENABLE_RISE &= ~BIT(pin);
			gpio->INTERRUPT_ENABLE_FALL |= BIT(pin);
		}
	} else {
		gpio->INTERRUPT_ENABLE_RISE &= ~BIT(pin);
		gpio->INTERRUPT_ENABLE_FALL &= ~BIT(pin);

		/* Level High ? */
		if (flags & GPIO_INT_ACTIVE_HIGH) {
			gpio->INTERRUPT_ENABLE_HIGH |= BIT(pin);
			gpio->INTERRUPT_ENABLE_LOW &= ~BIT(pin);
		} else {
			gpio->INTERRUPT_ENABLE_HIGH &= ~BIT(pin);
			gpio->INTERRUPT_ENABLE_LOW |= BIT(pin);
		}
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
static int gpio_spinal_lib_io_write(struct device *dev,
			    int access_op,
			    u32_t pin,
			    u32_t value)
{
	volatile struct gpio_spinal_lib_io_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= 32)
		return -EINVAL;

	if (value)
		gpio->OUTPUT |= BIT(pin);
	else
		gpio->OUTPUT &= ~BIT(pin);

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
static int gpio_spinal_lib_io_read(struct device *dev,
			   int access_op,
			   u32_t pin,
			   u32_t *value)
{
	volatile struct gpio_spinal_lib_io_t *gpio = DEV_GPIO(dev);

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= 32)
		return -EINVAL;

	/*
	 * If gpio is configured as output,
	 * read gpio value from OUTPUT register,
	 * otherwise read gpio value from INPUT register
	 */
	if (gpio->OUTPUT_ENABLE & BIT(pin))
		*value = (gpio->OUTPUT & BIT(pin));
	else
		*value = (gpio->INPUT & BIT(pin));

	return 0;
}



const struct gpio_driver_api gpio_spinal_lib_io_driver = {
	.config              = gpio_spinal_lib_io_config,
	.write               = gpio_spinal_lib_io_write,
	.read                = gpio_spinal_lib_io_read,
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
int gpio_spinal_lib_io_init(struct device *dev)
{
	volatile struct gpio_spinal_lib_io_t *gpio = DEV_GPIO(dev);

	/* Ensure that all gpio registers are reset to 0 initially */
	gpio->OUTPUT_ENABLE  = 0U;
	gpio->INTERRUPT_ENABLE_RISE = 0U;
	gpio->INTERRUPT_ENABLE_FALL = 0U;
	gpio->INTERRUPT_ENABLE_HIGH = 0U;
	gpio->INTERRUPT_ENABLE_LOW  = 0U;

	return 0;
}

GPIO_SPINAL_LIB_IO_DEVICE(DT_SPINAL_LIB_IO_GPIO_10000000_LABEL,DT_SPINAL_LIB_IO_GPIO_10000000_BASE_ADDRESS);
