#include <bmp280.h>
#include "freertos/FreeRTOSConfig.h"
#include <string.h>

bmp280_t bme280_init(gpio_num_t sda_pin, gpio_num_t scl_pin) {
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t bme280_dev;
    memset(&bme280_dev, 0, sizeof(bmp280_t));
    ESP_ERROR_CHECK(bmp280_init_desc(&bme280_dev, BMP280_I2C_ADDRESS_0, 0, sda_pin, scl_pin));
    ESP_ERROR_CHECK(bmp280_init(&bme280_dev, &params));
    return bme280_dev;
}
