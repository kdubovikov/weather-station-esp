#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "mqtt_client.h"

// #include "bme280.h"
#include "bmp280.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

/* Set the SSID and Password via project configuration, or can set directly here */
#define DEFAULT_SSID "MGTS_GPON_39F0"
#define DEFAULT_PWD "3gYgqSeA"

#if CONFIG_EXAMPLE_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_EXAMPLE_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_EXAMPLE_SCAN_METHOD*/

#if CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SECURITY
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#else
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#endif /*CONFIG_EXAMPLE_SORT_METHOD*/

#if CONFIG_EXAMPLE_FAST_SCAN_THRESHOLD
#define DEFAULT_RSSI CONFIG_EXAMPLE_FAST_SCAN_MINIMUM_SIGNAL
#if CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_OPEN
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WEP
#define DEFAULT_AUTHMODE WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WPA
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WPA2
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA2_PSK
#else
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif
#else
#define DEFAULT_RSSI -127
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif /*CONFIG_EXAMPLE_FAST_SCAN_THRESHOLD*/

#define WIFI_CONNECTED_BITS BIT(1)

// BME280 constants
#define TAG_BME280 "BME280"
#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

static const char *TAG = "weather_station";
static EventGroupHandle_t s_connect_event_group;

static void create_weather_msg(char* msg, float temp, float pressure, float altitude, float humidity) {
    sprintf(msg, 
            "{\"temp\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"humidity\":%.2f}", 
            temp, pressure, altitude, humidity);
}

// MQTT
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            char msg[90];
            create_weather_msg(msg, 0., 1., 2., 3.);
            ESP_LOGI(TAG, "Sending message: %s", msg);
            msg_id = esp_mqtt_client_publish(client, "weather", msg, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            const float SLEEP_TIME = 2 * 1e6;
            ESP_LOGI(TAG, "going to deep sleep for %.1f", SLEEP_TIME / 1e6);
            ESP_ERROR_CHECK(esp_wifi_stop());
            esp_deep_sleep(SLEEP_TIME);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT connection error: %d", event->error_handle->connect_return_code);
        default:
            ESP_LOGI(TAG, "MQTT recieved event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "Connecting to MQTT server at %s", CONFIG_BROKER_URL);
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL 
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}
// END MQTT

/* Initialize Wi-Fi as sta and set scan method */
static void event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip: %s", ip4addr_ntoa(&event->ip_info.ip));
        xEventGroupSetBits(s_connect_event_group, WIFI_CONNECTED_BITS);
    }
}

static void wifi_connect_blocking(void)
{
    s_connect_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PWD,
            .scan_method = DEFAULT_SCAN_METHOD,
            .sort_method = DEFAULT_SORT_METHOD,
            .threshold.rssi = DEFAULT_RSSI,
            .threshold.authmode = DEFAULT_AUTHMODE,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // wait for connection
    xEventGroupWaitBits(s_connect_event_group, WIFI_CONNECTED_BITS, true, true, portMAX_DELAY);
}


void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

s8_t BME280_I2C_bus_write(u8_t dev_addr, u8_t reg_addr, u8_t *reg_data, u16_t cnt)
{
	s32_t iError = 0;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = 0;
	} else {
		iError = 1;
	}
	i2c_cmd_link_delete(cmd);

	return (s8_t)iError;
}

s8_t BME280_I2C_bus_read(u8_t dev_addr, u8_t reg_addr, u8_t *reg_data, u16_t cnt)
{
	s32_t iError = 0;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = 0;
	} else {
		iError = 1;
	}

	i2c_cmd_link_delete(cmd);

	return (s8_t)iError;
}

void BME280_delay_msek(u32_t msek)
{
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

/*
void task_bme280_normal_mode(void *ignore)
{
	struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS2,
		.delay_msec = BME280_delay_msek
	};

	s32_t com_rslt;
	s32_t v_uncomp_pressure_s32;
	s32_t v_uncomp_temperature_s32;
	s32_t v_uncomp_humidity_s32;

	com_rslt = bme280_init(&bme280);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	if (com_rslt == SUCCESS) {
		while(true) {
			vTaskDelay(40 / portTICK_PERIOD_MS);

			com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

			if (com_rslt == SUCCESS) {
				ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
					bme280_compensate_temperature_double(v_uncomp_temperature_s32),
					bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
					bme280_compensate_humidity_double(v_uncomp_humidity_s32));
			} else {
				ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
			}
		}
	} else {
		ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
	}

	vTaskDelete(NULL);
}
*/
// void print_sensor_data(struct bme80_data *comp_data) {
// #ifdef BME280_FLOAT_ENABLE
//         printf("FLOAT %0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
// #else
//         printf("INT %ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
// #endif
// }

// void stream_sensor_data_normal_mode() {
// 	struct bme280_dev dev;
// 	int8_t rslt = BME280_OK;

// 	dev.dev_id = BME280_I2C_ADDR_PRIM;
// 	dev.intf = BME280_I2C_INTF;
// 	dev.read = BME280_I2C_bus_read;
// 	dev.write = BME280_I2C_bus_write;
// 	dev.delay_ms = BME280_delay_msek;

// 	rslt = bme280_init(&dev);

// 	uint8_t settings_sel;
// 	struct bme280_data comp_data;

// 	/* Recommended mode of operation: Indoor navigation */
// 	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
// 	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
// 	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
// 	dev.settings.filter = BME280_FILTER_COEFF_16;
// 	dev.settings.standby_time = BME280_STANDBY_TIME_1000_MS;

// 	settings_sel = BME280_OSR_PRESS_SEL;
// 	settings_sel |= BME280_OSR_TEMP_SEL;
// 	settings_sel |= BME280_OSR_HUM_SEL;
// 	settings_sel |= BME280_STANDBY_SEL;
// 	settings_sel |= BME280_FILTER_SEL;
// 	rslt = bme280_set_sensor_settings(settings_sel, &dev);
// 	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

// 	printf("Temperature, Pressure, Humidity\r\n");
// 	while (1) {
// 		/* Delay while the sensor completes a measurement */
// 		dev.delay_ms(10);
// 		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
// 		print_sensor_data(&comp_data);
// 	}

// 	vTaskDelete(NULL);
// }

// void stream_sensor_data_forced_mode()
// {
// 	struct bme280_dev dev;
// 	int8_t rslt = BME280_OK;

// 	dev.dev_id = BME280_I2C_ADDR_PRIM;
// 	dev.intf = BME280_I2C_INTF;
// 	dev.read = BME280_I2C_bus_read;
// 	dev.write = BME280_I2C_bus_write;
// 	dev.delay_ms = BME280_delay_msek;

// 	rslt = bme280_init(&dev);
//     uint8_t settings_sel;
// 	uint32_t req_delay;
//     struct bme280_data comp_data;

//     /* Recommended mode of operation: Indoor navigation */
//     dev.settings.osr_h = BME280_OVERSAMPLING_1X;
//     dev.settings.osr_p = BME280_OVERSAMPLING_16X;
//     dev.settings.osr_t = BME280_OVERSAMPLING_2X;
//     dev.settings.filter = BME280_FILTER_COEFF_16;

//     settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

//     rslt = bme280_set_sensor_settings(settings_sel, &dev);
	
// 	/*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
//      *  and the oversampling configuration. */
//     req_delay = bme280_cal_meas_delay(&dev.settings);

//     printf("Temperature, Pressure, Humidity\r\n");
//     /* Continuously stream sensor data */
//     while (1) {
//         rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
//         /* Wait for the measurement to complete and print data @25Hz */
//         dev.delay_ms(req_delay);
//         rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
//         print_sensor_data(&comp_data);
//     }

// 	vTaskDelete(NULL);
// }


/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

void stream_sensor_data() {
	int8_t rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    int32_t temp32;
    double temp;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = BME280_delay_msek;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = BME280_I2C_bus_read;
    bmp.write = BME280_I2C_bus_write;

    /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

    /*
     * bmp.dev_id = 0;
     * bmp.read = spi_reg_read;
     * bmp.write = spi_reg_write;
     * bmp.intf = BMP280_SPI_INTF;
     */
    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt);

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt);

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Temperature oversampling set at 4x */
    conf.os_temp = BMP280_OS_4X;

    /* Pressure over sampling none (disabling pressure measurement) */
    conf.os_pres = BMP280_OS_NONE;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);
    while (1)
    {
        /* Reading the raw data from sensor */
        rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

        /* Getting the 32 bit compensated temperature */
        rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);

        /* Getting the compensated temperature as floating point value */
        rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
        printf("UT: %d, T32: %d, T: %f \r\n", ucomp_data.uncomp_temp, temp32, temp);

        /* Sleep time between measurements = BMP280_ODR_1000_MS */
        bmp.delay_ms(1000);
    }

	vTaskDelete(NULL);
}

void task_i2cscanner(void *ignore) {
	ESP_LOGD("i2c_scanner", ">> i2cScanner");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA_PIN;
	conf.scl_io_num = SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_NUM_0, &conf);

	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

	int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	vTaskDelete(NULL);
}

void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // wifi_connect_blocking();
    // mqtt_app_start();

	i2c_master_init();


	TaskHandle_t xHandle = NULL;
	xTaskCreate(stream_sensor_data, "stream_sensor_data", 2048, NULL, 6, &xHandle);
	// xTaskCreate(task_i2cscanner, "i2c_scanner", 2048, NULL, 6, NULL);
}
