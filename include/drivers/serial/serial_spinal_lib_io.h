#ifndef UART_SPINAL_LIB_IO_H
#define UART_SPINAL_LIB_IO_H

#include <uart.h>
#include <errno.h>
#include <kernel.h>
#include <device.h>

struct uart_spinal_lib_com_regs{
	u32_t data;
	u32_t status;
};


struct uart_spinal_lib_com_device_config {
	u32_t base_address;
};


extern int uart_spinal_lib_com_init(struct device *dev);
extern const struct uart_driver_api uart_spinal_lib_com_driver_api;

#define xstr(s) str(s)
#define str(s) #s

#define SERIAL_SPINAL_LIB_IO_DEVICE(name, address)						\
	static const struct uart_spinal_lib_com_device_config uart_spinal_lib_com_dev_cfg_##name = {	\
		.base_address = address	\
	};								\
									\
	DEVICE_AND_API_INIT(uart_spinal_lib_com_##name,				\
			    str(name),	\
				uart_spinal_lib_com_init,				\
			    NULL,			\
			    &uart_spinal_lib_com_dev_cfg_##name,			\
				PRE_KERNEL_1,				\
				CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			    &uart_spinal_lib_com_driver_api)



//static const struct uart_spinal_lib_com_device_config uart_spinal_lib_com_dev_cfg_0 = {
//	.base_address = DT_SPINAL_LIB_COM_UART_0_BASE_ADDRESS
//};
//
//DEVICE_AND_API_INIT(uart_spinal_lib_com_0, "uart0",
//		    uart_spinal_lib_com_init, NULL,
//		    &uart_spinal_lib_com_dev_cfg_0,
//		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
//		    (void *)&uart_spinal_lib_com_driver_api);


#endif

