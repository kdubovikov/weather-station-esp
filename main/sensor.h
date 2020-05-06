#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include <bmp280.h>
#include "weather_station.h"
#include <string.h>

void bmp280_collect_data(void *pvParamters);