#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_sleep.h"

#include "mqtt_client.h"

#include "weather_station.h"


void mqtt_app_start(QueueHandle_t queue);