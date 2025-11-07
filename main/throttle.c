#include "throttle.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <stdio.h>
#include "bldc_interface.h"
#include "hw_config.h"
#include "led.h"

#define THROTTLE_TAG "THROTTLE"

uint16_t current_throttle_value = THROTTLE_NEUTRAL_VALUE;

static TimerHandle_t throttle_timeout_timer = NULL;
static bool timeout_monitoring_active = false;

static void throttle_timeout_callback(TimerHandle_t xTimer);
static void send_nunchuck_throttle(void *pvParameters);

//TaskHandle_t throttle_print_task_handle = NULL;

esp_err_t throttle_init(void)
{
    // Create task for nunchuck testing
    xTaskCreate(send_nunchuck_throttle, "nunchuck_test", 2048, NULL, 5, NULL);

    ESP_LOGI(THROTTLE_TAG, "Throttle initialized");

    // Create the timeout timer
    throttle_timeout_timer = xTimerCreate(
        "throttle_timeout",
        pdMS_TO_TICKS(THROTTLE_TIMEOUT_MS),
        pdTRUE,              // Auto reload
        NULL,
        throttle_timeout_callback
    );

    if (throttle_timeout_timer == NULL) {
        ESP_LOGE(THROTTLE_TAG, "Failed to create throttle timeout timer");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void throttle_update_value(uint16_t value)
{
    current_throttle_value = value;
    throttle_reset_timeout();  // Reset the timeout timer
    led_set_connection_state(true);  // Set LED to connected state on packet reception
    ESP_LOGD(THROTTLE_TAG, "%d", value);
}

void throttle_reset_value(void)
{
    current_throttle_value = THROTTLE_NEUTRAL_VALUE;
}

void throttle_timeout_callback(TimerHandle_t xTimer)
{
    throttle_reset_value();
    led_set_connection_state(false);  // Set LED to disconnected state on timeout
}
/*
void throttle_print_task(void *pvParameters)
{
    while (1) {
        mc_values* vesc_values = get_stored_vesc_values();
        ESP_LOGI(THROTTLE_TAG, "Throttle: %d | VESC Voltage: %.2fV | Motor RPM: %.1f",
            current_throttle_value,
            vesc_values->v_in,
            vesc_values->rpm);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}*/

void throttle_reset_timeout(void)
{
    if (timeout_monitoring_active && throttle_timeout_timer != NULL) {
        if (xTimerReset(throttle_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(THROTTLE_TAG, "Failed to reset throttle timeout timer");
        }
    }
}

void throttle_start_timeout_monitor(void)
{
    if (throttle_timeout_timer != NULL) {
        if (xTimerStart(throttle_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(THROTTLE_TAG, "Failed to start throttle timeout timer");
        } else {
            timeout_monitoring_active = true;
            led_set_connection_state(false);  // Start with LED in disconnected state
            ESP_LOGI(THROTTLE_TAG, "Throttle timeout monitoring started");
        }
    }
}

void throttle_stop_timeout_monitor(void)
{
    if (throttle_timeout_timer != NULL) {
        if (xTimerStop(throttle_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(THROTTLE_TAG, "Failed to stop throttle timeout timer");
        } else {
            timeout_monitoring_active = false;
            ESP_LOGI(THROTTLE_TAG, "Throttle timeout monitoring stopped");
        }
    }
}

static void send_nunchuck_throttle(void *pvParameters) {

    while (1) {
        uint8_t y_value = current_throttle_value;
        // Create packet for nunchuck data
        uint8_t buffer[5];
        int32_t ind = 0;

        buffer[ind++] = COMM_SET_CHUCK_DATA;  // Command ID
        buffer[ind++] = 128;                  // x-axis centered
        buffer[ind++] = y_value;              // y-axis variable
        buffer[ind++] = 0;                    // buttons released
        buffer[ind++] = 0;                    // extension data

        // Send the packet
        bldc_interface_send_packet(buffer, ind);

        // Delay before next update
        vTaskDelay(pdMS_TO_TICKS(20));  // 5ms delay for smooth ramping
    }
}

