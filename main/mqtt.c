#include "mqtt.h"

static const char *TAG = "weather_station_mqtt";

static void send_weather_to_mqtt(esp_mqtt_client_handle_t *client, QueueHandle_t* weather_msg_queue)
{
    int msg_id;
    if (weather_msg_queue != 0)
    {
        ESP_LOGI(TAG, "Recieving message from the queue");
        struct WeatherMessage msg;
        if (xQueueReceive(weather_msg_queue, &(msg), pdMS_TO_TICKS(1000)))
        {
            ESP_LOGI(TAG, "Got new message from the queue");
            char json_msg[90];
            create_weather_msg(json_msg, &msg);
            ESP_LOGI(TAG, "Sending JSON message: %s", json_msg);
            msg_id = esp_mqtt_client_publish(*client, "weather", json_msg, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            // const float SLEEP_TIME = 2 * 1e6;
            const float SLEEP_TIME = 1.44 * 1e10;
            ESP_LOGI(TAG, "going to deep sleep for %.1f", SLEEP_TIME / 1e6);
            ESP_ERROR_CHECK(esp_wifi_stop());
            esp_deep_sleep(SLEEP_TIME);
        }
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event,  QueueHandle_t weather_msg_queue)
{
    esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        send_weather_to_mqtt(&client, weather_msg_queue);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS)
        {
            ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            ESP_LOGE(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        }
        else
        {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "MQTT recieved event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    QueueHandle_t weather_msg_queue = (QueueHandle_t) handler_args;
    ESP_LOGI(TAG, "mqtt_event_handler QueueHandle: %p", weather_msg_queue);
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data, weather_msg_queue);
}

void mqtt_app_start(QueueHandle_t queue)   
{
    ESP_LOGI(TAG, "mqtt_app_start QueueHandle: %p", queue);
    ESP_LOGI(TAG, "Connecting to MQTT server at %s", CONFIG_BROKER_URL);
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, (void*) queue);
    esp_mqtt_client_start(client);
}
