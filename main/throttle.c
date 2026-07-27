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
static void send_throttle_task(void *pvParameters);

esp_err_t throttle_init(void)
{
    xTaskCreate(send_throttle_task, "throttle", 4096, NULL, 5, NULL);

    ESP_LOGI(THROTTLE_TAG, "Throttle initialized");

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
        if (failsafe_try_clear()) {
            throttle_reset_timeout();  // prevent auto-reload timer from immediately re-triggering
        }
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

// ERPM below this magnitude is treated as "essentially stopped" for the purpose
// of brake-vs-reverse transition. ~1 km/h on a typical esk8 setup.
#define THROTTLE_STOPPED_ERPM   300

// Map a 0..255 throttle byte to the appropriate VESC command using smart-reverse logic:
//   - Above neutral while moving reverse → brake (regen against reverse motion)
//   - Above neutral while stopped or moving forward → drive forward (SET_CURRENT_REL +)
//   - Below neutral while moving forward → brake (regen against forward motion)
//   - Below neutral while stopped or moving reverse → drive reverse (SET_CURRENT_REL -)
// This matches VESC "smart reverse" nunchuck behavior but lives in receiver firmware,
// so it works the same regardless of the rider's VESC app config. All commands scale
// against each VESC's own |l_current_max| / |l_current_min|.
static void send_throttle_task(void *pvParameters) {
    while (1) {
        // Failsafe task owns the motors while active — do not inject throttle.
        if (!failsafe_is_active()) {
            uint16_t raw = throttle_get_value();
            uint8_t value = (raw > 255) ? 255 : (uint8_t)raw;

            if (value == THROTTLE_NEUTRAL_VALUE) {
                bldc_interface_can_set_current_rel_all(0.0f);
            } else {
                int32_t erpm = bldc_interface_can_get_signed_dominant_erpm();

                if (value > THROTTLE_NEUTRAL_VALUE) {
                    float magnitude = (float)(value - THROTTLE_NEUTRAL_VALUE) /
                                      (float)(255 - THROTTLE_NEUTRAL_VALUE);
                    if (erpm < -THROTTLE_STOPPED_ERPM) {
                        // Moving reverse, rider wants forward — brake first.
                        bldc_interface_can_set_current_brake_rel_all(magnitude);
                    } else {
                        // Stopped or already forward — drive forward.
                        bldc_interface_can_set_current_rel_all(magnitude);
                    }
                } else {
                    float magnitude = (float)(THROTTLE_NEUTRAL_VALUE - value) /
                                      (float)THROTTLE_NEUTRAL_VALUE;
                    if (erpm > THROTTLE_STOPPED_ERPM) {
                        // Moving forward, rider wants brake or reverse — brake first.
                        bldc_interface_can_set_current_brake_rel_all(magnitude);
                    } else {
                        // Stopped or already reverse — drive reverse.
                        bldc_interface_can_set_current_rel_all(-magnitude);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
}

