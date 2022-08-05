#include <string.h>
#include "freertos/FreeRTOSConfig.h"
#include <bh1750.h>

i2c_dev_t bh1750_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint8_t addr) {
    i2c_dev_t bh1750_dev;
    memset(&bh1750_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(bh1750_init_desc(&bh1750_dev, addr, 0, sda_pin, scl_pin));
    ESP_ERROR_CHECK(bh1750_setup(&bh1750_dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH));
    return bh1750_dev;
}