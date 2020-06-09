#include "sensor.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

static const char *TAG = "weather_station_sensor";
const TickType_t xQueueBlockTime = pdMS_TO_TICKS(200);

/**
 * @brief collect data from BME280 sensor
 * 
 * @param pvParamters a pointer to QueueHandle_t to which this function will send the data collected from the sensor
 */
void bmp280_collect_data(void *pvParamters)
{
    // cast parameter pointer to QueueHandle_t
    QueueHandle_t queue = (QueueHandle_t) pvParamters;

    // initialize the sensor
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    // we use pins 21 and 22 for data transfer between sensor and ESP32
    // note that BMP280_I2C_ADDRESS_0 holds a constant preset address 0x76
    // which is used by all BMP280 sensors
    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_PIN, SCL_PIN));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    // check if we have detected BME280 or BMP280 modification of the sensor
    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    // the fun part starts here
    // read data from the sensor, create a WeatherMessage struct and send it to the queue
    float pressure, temperature, humidity;

    // we will wait for 500ms before reading any data
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
    {
        ESP_LOGI(TAG, "Temperature/pressure reading failed\n");
    }
    else
    {
        // WeatherMessage is defined in weather_station.h
        struct WeatherMessage msg;
        msg.temperature = temperature;
        msg.humidity = humidity;
        msg.pressure = pressure;
        ESP_LOGI(TAG, "Sending WeatherMessage to queue: %f %f %f\n", msg.temperature, msg.pressure, msg.humidity);

        xQueueSendToFront(queue, (void *)&msg, xQueueBlockTime);
    }

    // this should be called if you want FreeRTOS to execute the task once and finish afterwards
    vTaskDelete(NULL);
}
