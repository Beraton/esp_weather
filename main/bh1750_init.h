#ifndef bh1750_init_h
#define bh1750_init_h

#include <bh1750.h>

i2c_dev_t bh1750_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint8_t addr);

#endif