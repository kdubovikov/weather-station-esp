#include "nvs_flash.h"

#include "weather_station.h"
#include "sensor.h"
#include "wifi.h"
#include "mqtt.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "weather_station";

/**
 * @brief encode a WeatherMessage in JSON format
 * 
 * @param msg the output result will be written here
 * @param msg_struct WeatherMessage to be encoded
 */
void create_weather_msg(char *msg, struct WeatherMessage *msg_struct)
{
    sprintf(msg,
            "{\"temp\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"humidity\":%.2f}",
            msg_struct->temperature, msg_struct->pressure, 0.0, msg_struct->humidity);
}

/**
 * @brief Main function of the allication. Orchestrates all tasks.
 * 1. Collects data from the BME280 sensor
 * 2. Starts the WiFi module and connects to the access point
 * 3. Sends collected data to the MQTT server
 * 4. Enters deep sleep and wakes after predetermined interval
 */
void app_main()
{
    // let's create an EventGroup which will be used to block until WiFi has connected
    EventGroupHandle_t s_connect_event_group = xEventGroupCreate();

    // a queue for WeatherMessages from the sensor
    QueueHandle_t queue = xQueueCreate(2, sizeof(struct WeatherMessage));

    ESP_LOGI(TAG, "QueueHandle: %p", &queue);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    // initialize I2C
    ESP_ERROR_CHECK(i2cdev_init());
    // this FreeRTOS function runs the bmp280_collect_data function in parallel
    // we don't need to block on it since we are using a queue to send WeatherMessages
    // so all functions will wait for new messages on the queue
    xTaskCreatePinnedToCore(bmp280_collect_data, "bmp280_collect_data", configMINIMAL_STACK_SIZE * 8, (void *)queue, 5, NULL, APP_CPU_NUM);

    // Initialize the NonVolatileStorage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // connect to the WiFi
    wifi_connect_blocking(&s_connect_event_group);

    // send all incoming WeatherMessages to the MQTT topic
    mqtt_app_start(queue);
}
