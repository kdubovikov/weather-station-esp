#include "nvs_flash.h"

#include "weather_station.h"
#include "sensor.h"
#include "wifi.h"
#include "mqtt.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "weather_station";

void create_weather_msg(char *msg, struct WeatherMessage *msg_struct)
{
    sprintf(msg,
            "{\"temp\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"humidity\":%.2f}",
            msg_struct->temperature, msg_struct->pressure, 0.0, msg_struct->humidity);
}

void app_main()
{
    EventGroupHandle_t s_connect_event_group = xEventGroupCreate();
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

    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(bmp280_collect_data, "bmp280_collect_data", configMINIMAL_STACK_SIZE * 8, (void *)queue, 5, NULL, APP_CPU_NUM);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_connect_blocking(&s_connect_event_group);
    mqtt_app_start(queue);
}
