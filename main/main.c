#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <cJSON.h>
#include "wifi_conn.h"
#include <mqtt_client.h>
#include "esp_tls.h"
#include <string.h>
#include <bh1750.h>
#include <bmp280.h>
#include "bme280_init.h"
#include "bh1750_init.h"

#define TASK_SIZE_KB 1024
#define GPIO_OUTPUT_SEL_PIN ((1ULL << CONFIG_I2C_MASTER_SCL | 1ULL << CONFIG_I2C_MASTER_SDA))

#if defined(CONFIG_BH1750_I2C_ADDRESS_LO)
#define ADDR BH1750_ADDR_LO
#endif
#if defined(CONFIG_BH1750_I2C_ADDRESS_HI)
#define ADDR BH1750_ADDR_HI
#endif

// Uncomment to trace potential memory leaks
//#define HEAP_TRACE

#ifdef HEAP_TRACE
#include "esp_heap_trace.h"
#endif

static const char *TAG = "MQTT";
uint8_t mac_addr[6];

extern const uint8_t ca_cert_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_cert_end[] asm("_binary_ca_crt_end");
extern const uint8_t client_crt_start[] asm("_binary_client_crt_start");
extern const uint8_t client_crt_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_end[] asm("_binary_client_key_end");

typedef struct sensorData
{
  int32_t temperature;
  uint32_t humidity;
  uint32_t pressure;
  uint16_t lux;
} sensorData;

QueueHandle_t sensorQueue;
TaskHandle_t connectionHandle;
TaskHandle_t sensorHandle;
SemaphoreHandle_t mutexBus;

const uint32_t WIFI_CONNECTED = BIT1;
const uint32_t MQTT_CONNECTED = BIT2;
const uint32_t MQTT_PUBLISHED = BIT3;

#ifdef HEAP_TRACE
#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS];
#endif

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

char *create_json_payload(sensorData *data)
{

#ifdef HEAP_TRACE
  ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
#endif

  char *payload;
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

#ifdef HEAP_TRACE
  ESP_ERROR_CHECK(heap_trace_stop());
  heap_trace_dump();
#endif

  return payload;
}

void sensor_measure(void *pvParameters)
{
  sensorData data;

  // Initializing i2c devices
  bmp280_t bme280_dev = bme280_init(CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL);
  i2c_dev_t bh1750_dev = bh1750_init(CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL, ADDR);

  // Checking for BME280
  bool bme280p = bme280_dev.id == BME280_CHIP_ID;
  printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

  // Checking for BH1750
  bool bh1750 = bh1750_dev.addr == ADDR;
  printf("BH1750:%s found!\n", bh1750 ? "" : "not");

  while (true)
  {
    if (bmp280_read_fixed(&bme280_dev, &data.temperature, &data.pressure, &data.humidity) != ESP_OK)
    {
      printf("Temperature/pressure reading failed\n");
      continue;
    }
    if (bh1750_read(&bh1750_dev, &data.lux) != ESP_OK)
      printf("Could not read lux data\n");
    ESP_LOGI("SENSOR_MEAS", "Releasing mutex -> sending queue item -> suspending task...");
    xQueueSend(sensorQueue, &data, 2000 / portTICK_PERIOD_MS);
    vTaskSuspend(sensorHandle);
  }
}

void on_connected(void *params)
{
  uint32_t command = 0;
  esp_mqtt_client_config_t mqttConfig =
      {
          .uri = CONFIG_BROKER_URI,
          .cert_pem = (const char *)ca_cert_start,
          .client_cert_pem = (const char *)client_crt_start,
          .client_key_pem = (const char *)client_key_start,
          .username = CONFIG_MQTT_USERNAME,
          .password = CONFIG_MQTT_PASSWORD,
      };
  // ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  esp_mqtt_client_handle_t client = NULL;
  sensorData data;

  while (true)
  {
    ESP_LOGI("ON_CONN", "Queue message number: %d", uxQueueMessagesWaiting(sensorQueue));
    ESP_ERROR_CHECK(esp_wifi_start());
    xTaskNotifyWait(0, 0, &command, portMAX_DELAY);
    switch (command)
    {
    case WIFI_CONNECTED:
      client = esp_mqtt_client_init(&mqttConfig);
      esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
      esp_mqtt_client_start(client);
      break;
    case MQTT_CONNECTED:
    {
      if (xQueueReceive(sensorQueue, &data, portMAX_DELAY))
      {
        char *buffer;
        uint8_t my_mac[6];
        esp_efuse_mac_get_default(my_mac);
        buffer = create_json_payload(&data);
        ESP_LOGI("MQTT_CONN", "JSON payload: %s", buffer);
        esp_mqtt_client_publish(client, CONFIG_MQTT_TOPIC, buffer, strlen(buffer), 2, false);
        free(buffer);
      }
      else
      {
        ESP_LOGI("ON_CONN", "Failed to receive queue item");
      }
      break;
    }
    case MQTT_PUBLISHED:
      esp_mqtt_client_stop(client);
      esp_mqtt_client_destroy(client);
      wifi_disconnect();
      ESP_LOGI("ON_CONN", "Entering deep sleep for %d ms...", CONFIG_MEASUREMENT_INTERVAL);
      esp_deep_sleep_set_rf_option(1);
      esp_deep_sleep(CONFIG_MEASUREMENT_INTERVAL * 1.0e3); // interval set in [ms]
      // Everything after esp_deep_sleep() is irrelevant as after it the ESP performs a restart
      // Ending code just to make sure it works properly
      ESP_LOGI("ON_CONN", "Waking up from sleep, resuming task...");
      vTaskResume(sensorHandle);
      break;
    }
  }
}

void app_main()
{
  ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac_addr));
  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

#ifdef HEAP_TRACE
  ESP_ERROR_CHECK(heap_trace_init_standalone(trace_record, NUM_RECORDS));
#endif

  // Initializing i2c_dev
  ESP_ERROR_CHECK(i2cdev_init());

  // WiFi initialization
  ESP_ERROR_CHECK(nvs_flash_init());
  wifi_init_sta();
  wifi_connect_sta(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD, 3000);

  // Defining queues and tasks
  sensorQueue = xQueueCreate(10, sizeof(sensorData));
  mutexBus = xSemaphoreCreateMutex();
  xTaskCreate(sensor_measure, "Sensor measure", TASK_SIZE_KB * 10, NULL, 5, &sensorHandle);
  xTaskCreate(on_connected, "On WIFI connection trigger our other tasks", TASK_SIZE_KB * 4, NULL, 5, &connectionHandle);
}
