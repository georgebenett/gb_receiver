#include "throttle.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <stdio.h>
#include <stdatomic.h>
#include "bldc_interface_can.h"
#include "failsafe.h"
#include "hw_config.h"
#include "led.h"
#include "aux_output.h"

#define THROTTLE_TAG "THROTTLE"

static _Atomic uint16_t current_throttle_value = THROTTLE_NEUTRAL_VALUE;
static bool throttle_packet_received = false;

uint16_t throttle_get_value(void) {
    return atomic_load(&current_throttle_value);
}

static TimerHandle_t throttle_timeout_timer = NULL;
static bool timeout_monitoring_active = false;

static void throttle_timeout_callback(TimerHandle_t xTimer);
static void send_nunchuck_throttle(void *pvParameters);

esp_err_t throttle_init(void)
{
    // Create task for nunchuck testing
    // Increased stack size from 2048 to 4096 to prevent stack overflow
    xTaskCreate(send_nunchuck_throttle, "nunchuck_test", 4096, NULL, 5, NULL);

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
    // Attempt to clear an active failsafe when the remote link is restored.
    // failsafe_try_clear() enforces its own speed guard — it is a no-op while moving.
    throttle_packet_received = true;

    if (failsafe_is_active()) {
        failsafe_try_clear();
        return;  // Discard the throttle value until failsafe is resolved
    }

    atomic_store(&current_throttle_value, value);
    throttle_reset_timeout();
    led_set_connection_state(true);
    aux_output_update_pwm();
    ESP_LOGD(THROTTLE_TAG, "%d", value);
}

void throttle_reset_value(void)
{
    atomic_store(&current_throttle_value, (uint16_t)THROTTLE_NEUTRAL_VALUE);
}

void throttle_timeout_callback(TimerHandle_t xTimer)
{
    throttle_reset_value();
    led_set_connection_state(false);
    // Only trigger failsafe if at least one packet was received — avoids a
    // spurious trigger on boot before the remote has had time to connect.
    if (throttle_packet_received) {
        failsafe_trigger(FAILSAFE_REASON_THROTTLE_TIMEOUT);
    }
}

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
            throttle_packet_received = false;  // Reset so next connect needs a real packet first
            ESP_LOGI(THROTTLE_TAG, "Throttle timeout monitoring stopped");
        }
    }
}

static void send_nunchuck_throttle(void *pvParameters) {

    while (1) {
        // Do not inject throttle while failsafe is active — failsafe task owns the motors
        if (!failsafe_is_active()) {
            uint8_t y_value = (uint8_t)throttle_get_value();
            uint8_t buffer[5];
            int32_t ind = 0;

            buffer[ind++] = COMM_SET_CHUCK_DATA;
            buffer[ind++] = 128;     // x-axis centered
            buffer[ind++] = y_value; // y-axis
            buffer[ind++] = 0;       // buttons released
            buffer[ind++] = 0;       // extension data

            // Send to every detected VESC independently — no CAN forwarding assumed
            bldc_interface_can_send_to_all(buffer, ind);
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
}

