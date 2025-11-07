#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

#include "ble.h"
#include "throttle.h"
#include "led.h"
#include "bms.h"
#include "bldc_interface.h"
#include "bms_interface_uart.h"
#include "bldc_interface_uart.h"

static const char *TAG = "MAIN";

static mc_values stored_values;
static bool uart_logging_enabled = false;

static void bldc_values_received(mc_values *values) {
    stored_values = *values;
    bms_values_t* bms_data = get_stored_bms_values();
    if (bms_data != NULL) {
        send_telemetry_data(&stored_values, bms_data);
    }
}

static void vesc_task(void *pvParameters) {
    while (1) {
        bldc_interface_get_values();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void uart_command_handler_task(void *pvParameters) {
    uint8_t data[128];
    char command[64];
    int command_idx = 0;

    ESP_LOGI(TAG, "UART command handler task started");

    while (1) {
        int len = uart_read_bytes(UART_NUM_0, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';

            for (int i = 0; i < len; i++) {
                if (data[i] == '\n' || data[i] == '\r') {
                    if (command_idx > 0) {
                        command[command_idx] = '\0';

                        if (strcmp(command, "ENABLE_LOG") == 0) {
                            uart_logging_enabled = true;
                        } else if (strcmp(command, "DISABLE_LOG") == 0) {
                            uart_logging_enabled = false;
                        } else {
                            ESP_LOGW(TAG, "Unknown command: %s", command);
                        }

                        command_idx = 0;
                    }
                } else if (command_idx < (int)sizeof(command) - 1) {
                    command[command_idx++] = data[i];
                } else {
                    command_idx = 0;
                }
            }
        }
    }
}

mc_values* get_stored_vesc_values(void) {
    return &stored_values;
}

void send_telemetry_data(const mc_values* vesc_data, const bms_values_t* bms_data) {
    if (!uart_logging_enabled) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON *vesc = cJSON_CreateObject();
    cJSON *bms = cJSON_CreateObject();
    cJSON *cells = cJSON_CreateArray();

    cJSON_AddNumberToObject(root, "timestamp", esp_timer_get_time() / 1000);

    cJSON_AddNumberToObject(vesc, "voltage", vesc_data->v_in);
    cJSON_AddNumberToObject(vesc, "current_motor", vesc_data->current_motor);
    cJSON_AddNumberToObject(vesc, "current_input", vesc_data->current_in);
    cJSON_AddNumberToObject(vesc, "duty", vesc_data->duty_now);
    cJSON_AddNumberToObject(vesc, "rpm", vesc_data->rpm);
    cJSON_AddNumberToObject(vesc, "temp_mos", vesc_data->temp_mos);
    cJSON_AddNumberToObject(vesc, "temp_motor", vesc_data->temp_motor);
    cJSON_AddItemToObject(root, "vesc", vesc);

    cJSON_AddNumberToObject(bms, "total_voltage", bms_data->total_voltage);
    cJSON_AddNumberToObject(bms, "current", bms_data->current);
    cJSON_AddNumberToObject(bms, "remaining_capacity", bms_data->remaining_capacity);
    cJSON_AddNumberToObject(bms, "nominal_capacity", bms_data->nominal_capacity);

    for (int i = 0; i < bms_data->num_cells; i++) {
        cJSON_AddItemToArray(cells, cJSON_CreateNumber(bms_data->cell_voltages[i]));
    }
    cJSON_AddItemToObject(bms, "cell_voltages", cells);
    cJSON_AddItemToObject(root, "bms", bms);

    char *json_string = cJSON_PrintUnformatted(root);
    size_t json_len = strlen(json_string);

    uart_write_bytes(UART_NUM_0, json_string, json_len);
    uart_write_bytes(UART_NUM_0, "\n", 1);

    cJSON_free(json_string);
    cJSON_Delete(root);
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
    ESP_ERROR_CHECK(bms_uart_init());

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

    throttle_init();
    bms_interface_uart_init();
    bldc_interface_uart_init(bms_interface_uart_send_function);
    bldc_interface_set_rx_value_func(bldc_values_received);

    xTaskCreate(vesc_task, "vesc_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_command_handler_task, "uart_cmd_handler", 2048, NULL, 5, NULL);
}
