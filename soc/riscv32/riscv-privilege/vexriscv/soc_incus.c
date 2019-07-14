#include <generated_dts_board.h>
#include <gpio/gpio_spinal_lib_io.h>
#include <serial/serial_spinal_lib_io.h>

#if CONFIG_INCUS_GPIOA
GPIO_SPINAL_LIB_IO_DEVICE(gpioA, DT_SPINAL_LIB_IO_GPIO_10000000_BASE_ADDRESS);
#endif

SERIAL_SPINAL_LIB_IO_DEVICE(uart0, DT_SPINAL_LIB_COM_UART_10010000_BASE_ADDRESS);
