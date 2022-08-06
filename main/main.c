#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include "wifi_conn.h"
#include <mqtt_client.h>
#include "esp_tls.h"
#include <string.h>
#include <bh1750.h>
#include <bmp280.h>
//#include "esp_heap_trace.h"

#include "bme280_init.h"
#include "bh1750_init.h"

#define TASK_SIZE_KB 1024

#if defined(CONFIG_BH1750_I2C_ADDRESS_LO)
#define ADDR BH1750_ADDR_LO
#endif
#if defined(CONFIG_BH1750_I2C_ADDRESS_HI)
#define ADDR BH1750_ADDR_HI
#endif

static const char* TAG = "MQTT";

extern const uint8_t ca_cert_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_cert_end[] asm("_binary_ca_crt_end");
extern const uint8_t client_crt_start[] asm("_binary_client_crt_start");
extern const uint8_t client_crt_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_end[] asm("_binary_client_key_end");


typedef struct sensorData {
    int32_t temperature;
    uint32_t humidity;
    uint32_t pressure;
    uint16_t lux;
} sensorData;

QueueHandle_t sensorQueue;
TaskHandle_t connectionHandle;

const uint32_t WIFI_CONNECTED = BIT1;
const uint32_t MQTT_CONNECTED = BIT2;
const uint32_t MQTT_PUBLISHED = BIT3;

//#define NUM_RECORDS 100
//static heap_trace_record_t trace_record[NUM_RECORDS];

void mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
  switch (event->event_id)
  {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    xTaskNotify(connectionHandle, MQTT_CONNECTED, eSetValueWithOverwrite);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    xTaskNotify(connectionHandle, MQTT_PUBLISHED, eSetValueWithOverwrite);
    break;
  case MQTT_EVENT_BEFORE_CONNECT:
    ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  mqtt_event_handler_cb(event_data);
}

char* create_json_payload(sensorData *data) {
  //ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
  char* payload;
  char *buf = malloc(sizeof(char) * 20);
  cJSON *json_payload = cJSON_CreateObject();

  sprintf(buf, "%d", data->temperature);
  cJSON_AddStringToObject(json_payload, "temperature", buf);
  sprintf(buf, "%d", data->humidity);
  cJSON_AddStringToObject(json_payload, "humidity", buf);
  sprintf(buf, "%d", data->pressure);
  cJSON_AddStringToObject(json_payload, "pressure", buf);
  sprintf(buf, "%d", data->lux);
  cJSON_AddStringToObject(json_payload, "light", buf);
  payload = cJSON_Print(json_payload);
  cJSON_Delete(json_payload);
  free(buf);
  buf = NULL;
  //ESP_ERROR_CHECK(heap_trace_stop());
  //heap_trace_dump();
  return payload;
}

void sensor_measurement(void *pvParameters)
{
    bmp280_t bme280_dev = bme280_init(CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL);
    i2c_dev_t bh1750_dev = bh1750_init(CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL, ADDR);

    bool bme280p = bme280_dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    sensorData data;

    while (true)
    {
        if (bmp280_read_fixed(&bme280_dev, &data.temperature, &data.pressure, &data.humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }
        if (bh1750_read(&bh1750_dev, &data.lux) != ESP_OK)
            printf("Could not read lux data\n");
        /*
        printf("Temperature: %d C, Pressure: %d Pa", data.temperature, data.humidity);
        printf("Temperature: %s C,", print_float(data.temperature));
        printf(" Pressure: %s Pa,", print_float(data.pressure));
        if (bme280p)
            printf(", Humidity: %s %%", print_float(data.humidity));
            printf(", Humidity: %d %%", data.pressure);
        printf(", lux: %d lx\n", data.lux);
        */
        xQueueSend(sensorQueue, &data, 2000 / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_MEASUREMENT_INTERVAL));
    }
}

void mqtt_fn(sensorData data) {
    uint32_t command = 0;
    esp_mqtt_client_config_t mqttConfig = {
      .uri = CONFIG_BROKER_URI,
      .cert_pem = (const char*)ca_cert_start,
      .client_cert_pem = (const char*)client_crt_start,
      .client_key_pem = (const char*)client_key_start,
      .username = CONFIG_MQTT_USERNAME,
      .password = CONFIG_MQTT_PASSWORD,
    };
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = NULL;

    while (true)
    {
        xTaskNotifyWait(0, 0, &command, portMAX_DELAY);
        switch (command)
        {
        case WIFI_CONNECTED:
            client = esp_mqtt_client_init(&mqttConfig);
            esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
            esp_mqtt_client_start(client);
            break;
        case MQTT_CONNECTED:
            char* buffer;
            buffer = create_json_payload(&data);
            esp_mqtt_client_publish(client, "beraton_pub", buffer, strlen(buffer), 2, false);
            free(buffer);
            break;
        case MQTT_PUBLISHED:
            esp_mqtt_client_stop(client);
            esp_mqtt_client_destroy(client);
            esp_wifi_stop();
            return;
        default:
            break;
        }
    }
}

void on_connected(void* params) {
    while(true) {
        sensorData data;
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY)) {
            ESP_ERROR_CHECK(esp_wifi_start());
            mqtt_fn(data);
        }
    }
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    //ESP_ERROR_CHECK(heap_trace_init_standalone(trace_record, NUM_RECORDS));
    i2cdev_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    wifi_connect_sta(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD, 10000);
    
    sensorQueue = xQueueCreate(10, sizeof(sensorData));
    xTaskCreate(sensor_measurement, "Sensor measurement", TASK_SIZE_KB * 10, NULL, 5, NULL);
    xTaskCreate(on_connected, "Connect/reconnect to wifi", TASK_SIZE_KB * 4, NULL, 5, &connectionHandle);
}