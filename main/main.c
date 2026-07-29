#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hw_config.h"
#include "ble.h"
#include "throttle.h"
#include "failsafe.h"
#include "led.h"
#include "bms.h"
#include "bldc_interface_can.h"
#include "aux_output.h"
#include "usb_serial.h"
#include "wifi_ap.h"

static const char *TAG = "MAIN";

static mc_values stored_values;
static mc_temp_config_t stored_mc_temp_conf = {0};

mc_temp_config_t* get_stored_mc_temp_config(void);

static void bldc_values_received(mc_values *values) {
    float last_good_v_in = stored_values.v_in;
    stored_values = *values;
    // A v_in at or below 1 V means the VESC did not report it in this frame —
    // keep the last good reading so the UI doesn't flicker to zero.
    if (values->v_in <= 1.0f) {
        stored_values.v_in = last_good_v_in;
    }
}

static void mcconf_temp_received(mc_temp_config_t *conf) {
    if (conf && conf->valid) {
        stored_mc_temp_conf = *conf;
        ESP_LOGI(TAG, "MC temp conf received: poles=%u gear=%.3f wheel_diam=%.3f",
                 conf->motor_poles,
                 conf->gear_ratio,
                 conf->wheel_diameter);
    } else {
        ESP_LOGW(TAG, "MC temp conf invalid or not received");
    }
}

// After an OTA, the new image boots in PENDING_VERIFY. If we reboot before
// confirming it, the bootloader rolls back to the previous slot. We give the
// device 60 s of uptime — long enough to catch boot-loop bugs, short enough
// that a power-cycle right after an update doesn't surprise the user with a
// rollback.
static void ota_boot_verifier_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(60000));
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGW(TAG, "OTA boot verified (%s)", esp_err_to_name(err));
    }
    vTaskDelete(NULL);
}

static void vesc_task(void *pvParameters) {
    uint32_t config_retry_counter = 0;
    const uint32_t CONFIG_RETRY_INTERVAL = 100; // Retry every 100 iterations = 5 seconds

    while (1) {
        bldc_interface_can_get_values();
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

    // Print the reason for the most recent reset so we can tell crashes from
    // brown-outs from clean reboots in the field log.
    ESP_LOGW(TAG, "Reset reason: %d", (int)esp_reset_reason());

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(led_init());
    ESP_ERROR_CHECK(aux_output_init());
    ESP_ERROR_CHECK(bms_uart_init());

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

    // Wi-Fi stack is initialized here so the SoftAP fallback can start quickly
    // when the idle timer fires; the AP itself stays off until then.
    wifi_ap_init();

    usb_serial_start_task();

    // Initialize CAN interface for VESC communication
    bldc_interface_can_init(CAN_TX_PIN, CAN_RX_PIN);
    bldc_interface_can_set_rx_value_func(bldc_values_received);
    bldc_interface_can_set_rx_mcconf_temp_func(mcconf_temp_received);

    // Drive motors into a known-quiescent state before anything that might
    // command them starts running. Guards against the case where the receiver
    // power-cycles while the VESCs are still alive and holding a previous command.
    bldc_interface_can_motors_safe_stop();

    // Failsafe must be initialized after CAN so the dropout callback is registered
    // before the first RX task iteration runs.
    failsafe_init();

    // Throttle task runs last — by this point the failsafe is armed, CAN is up,
    // and any first throttle command has a clean path through.
    throttle_init();

    // MC config will be requested when BLE connects (see ble.c ESP_GATTS_CONNECT_EVT)

    // Start CAN RX task to process incoming CAN frames
    xTaskCreate(bldc_interface_can_rx_task, "can_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(vesc_task, "vesc_task", 4096, NULL, 5, NULL);
    xTaskCreate(ota_boot_verifier_task, "ota_verify", 2048, NULL, 1, NULL);
}
