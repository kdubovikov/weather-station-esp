#include "sensor.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

static const char *TAG = "weather_station_sensor";
const TickType_t xQueueBlockTime = pdMS_TO_TICKS(200);

void bmp280_collect_data(void *pvParamters)
{
    QueueHandle_t queue = (QueueHandle_t) pvParamters;
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_PIN, SCL_PIN));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    float pressure, temperature, humidity;

    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
    {
        ESP_LOGI(TAG, "Temperature/pressure reading failed\n");
    }
    else
    {
        struct WeatherMessage msg;
        msg.temperature = temperature;
        msg.humidity = humidity;
        msg.pressure = pressure;
        ESP_LOGI(TAG, "Sending WeatherMessage to queue: %f %f %f\n", msg.temperature, msg.pressure, msg.humidity);

        xQueueSendToFront(queue, (void *)&msg, xQueueBlockTime);
    }
    vTaskDelete(NULL);
}
