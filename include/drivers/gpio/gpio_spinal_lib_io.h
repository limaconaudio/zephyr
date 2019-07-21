#ifndef GPIO_SPINAL_LIB_IO_H
#define GPIO_SPINAL_LIB_IO_H

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <gpio.h>


/* spinal_lib_io GPIO register-set structure */
struct gpio_spinal_lib_io_t {
	u32_t INPUT;
	u32_t OUTPUT;
	u32_t OUTPUT_ENABLE;
	u32_t DUMMY[5];
	u32_t INTERRUPT_ENABLE_RISE;
	u32_t INTERRUPT_ENABLE_FALL;
	u32_t INTERRUPT_ENABLE_HIGH;
	u32_t INTERRUPT_ENABLE_LOW;

};

struct gpio_spinal_lib_io_config {
	u32_t            gpio_base_addr;
};


extern const struct gpio_driver_api gpio_spinal_lib_io_driver;
extern int gpio_spinal_lib_io_init(struct device *dev);

#define xstr(s) str(s)
#define str(s) #s

#define GPIO_SPINAL_LIB_IO_DEVICE(name, address)						\
	static const struct gpio_spinal_lib_io_config gpio_spinal_lib_io_config_##name = {	\
		.gpio_base_addr = address	\
	};								\
									\
	DEVICE_AND_API_INIT(gpio_spinal_lib_io_##name,				\
			    name,	\
			    gpio_spinal_lib_io_init,				\
			    NULL,			\
			    &gpio_spinal_lib_io_config_##name,			\
			    POST_KERNEL,				\
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
			    &gpio_spinal_lib_io_driver)


#endif

