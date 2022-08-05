#ifndef bme280_init_h
#define bme280_init_h

#include <bmp280.h>

bmp280_t bme280_init(gpio_num_t sda_pin, gpio_num_t scl_pin);

#endif