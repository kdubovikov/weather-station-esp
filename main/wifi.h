#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_log.h"

void wifi_connect_blocking(EventGroupHandle_t* connected_event_group);