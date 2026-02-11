#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble.h"
#include "throttle.h"
#include "led.h"
#include "bms.h"
#include "bldc_interface.h"
#include "bms_interface_uart.h"
#include "bldc_interface_can.h"
#include "aux_output.h"
#include "servo.h"
#include "usb_serial.h"

static const char *TAG = "MAIN";

static mc_values stored_values;
static mc_temp_config_t stored_mc_temp_conf = {0};

// Forward declarations
mc_temp_config_t* get_stored_mc_temp_config(void);

static void bldc_values_received(mc_values *values) {
    stored_values = *values;
    // VESC data is streamed via USB serial when streaming is enabled
}

static void mcconf_temp_received(mc_temp_config_t *conf) {
    stored_mc_temp_conf = *conf;
    if (conf && conf->valid) {
        ESP_LOGI(TAG, "MC temp conf received: poles=%u gear=%.3f wheel_diam=%.3f",
                 conf->motor_poles,
                 conf->gear_ratio,
                 conf->wheel_diameter);
    } else {
        ESP_LOGW(TAG, "MC temp conf invalid or not received");
    }
}

static void vesc_task(void *pvParameters) {
    uint32_t config_retry_counter = 0;
    const uint32_t CONFIG_RETRY_INTERVAL = 100; // Retry every 100 iterations = 5 seconds

    while (1) {
        bldc_interface_can_get_values();

        // Periodically check if motor config is valid, and retry if not
        // Only request when BLE is connected to avoid unnecessary requests
        // This ensures config is available even if initial request failed or VESC was not ready
        config_retry_counter++;
        if (config_retry_counter >= CONFIG_RETRY_INTERVAL) {
            config_retry_counter = 0;
            // Only request MC config when BLE is connected
            if (ble_is_connected()) {
                mc_temp_config_t* config = get_stored_mc_temp_config();
                if (config == NULL || !config->valid) {
                    ESP_LOGD(TAG, "Motor config invalid, requesting...");
                    bldc_interface_can_get_mcconf_temp();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


mc_values* get_stored_vesc_values(void) {
    return &stored_values;
}

mc_temp_config_t* get_stored_mc_temp_config(void) {
    return &stored_mc_temp_conf;
}

void app_main(void) {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(led_init());
    ESP_ERROR_CHECK(aux_output_init());
    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(bms_uart_init());

    // Initialize USB serial
    usb_serial_init();

    ret = ble_spp_server_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE SPP server: %s", esp_err_to_name(ret));
        return;
    }

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);

    ret = ble_spp_server_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE SPP server: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "BLE SPP server started successfully");

    // Start USB serial task
    usb_serial_start_task();

    throttle_init();
    bms_interface_uart_init();

    // Initialize CAN interface for VESC communication (replaces UART)
    bldc_interface_can_init();
    bldc_interface_can_set_rx_value_func(bldc_values_received);
    bldc_interface_can_set_rx_mcconf_temp_func(mcconf_temp_received);

    // Make bldc_interface use CAN for sending packets (for throttle, etc.)
    bldc_interface_init(bldc_interface_can_send_packet);
    bldc_interface_set_rx_value_func(bldc_values_received);
    bldc_interface_set_rx_mcconf_temp_func(mcconf_temp_received);

    // MC config will be requested when BLE connects (see ble.c ESP_GATTS_CONNECT_EVT)

    // Start CAN RX task to process incoming CAN frames
    xTaskCreate(bldc_interface_can_rx_task, "can_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(vesc_task, "vesc_task", 4096, NULL, 5, NULL);
}
