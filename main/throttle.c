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
#include "smart_reverse.h"
#include "assist_push.h"
#include "ble.h"
#include <math.h>

#define THROTTLE_TAG "THROTTLE"

extern mc_temp_config_t *get_stored_mc_temp_config(void);

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

#define THROTTLE_LOOP_MS        20  // 50 Hz

static _Atomic bool smart_reverse_enabled = false;
static _Atomic bool no_reverse_enabled = false;
static _Atomic bool assist_push_enabled = false;
// Rider-tunable, owned by the remote. Defaults apply until it sends its own.
static _Atomic int assist_strength_pct = (int)(ASSIST_MAX_CURRENT * 100.0f);
static _Atomic int assist_decay_rpm_s = (int)ASSIST_DECAY_RPM_S;

void throttle_set_smart_reverse(bool enabled)
{
    atomic_store(&smart_reverse_enabled, enabled);
    ESP_LOGI(THROTTLE_TAG, "Smart reverse %s", enabled ? "enabled" : "disabled");
}

// Below neutral only ever brakes. Overrides smart reverse so they cannot fight.
void throttle_set_no_reverse(bool enabled)
{
    atomic_store(&no_reverse_enabled, enabled);
    ESP_LOGI(THROTTLE_TAG, "Reverse %s", enabled ? "disabled" : "enabled");
}

void throttle_set_assist_push(bool enabled)
{
    atomic_store(&assist_push_enabled, enabled);
    ESP_LOGI(THROTTLE_TAG, "Assistive push %s", enabled ? "enabled" : "disabled");
}

// Strength and decay come from the remote, clamped here: the receiver must not
// take a bad value on faith, whatever sent it.
void throttle_set_assist_params(uint8_t strength_pct, uint8_t decay_rpm_s)
{
    float current = strength_pct / 100.0f;
    if (current < ASSIST_CURRENT_MIN) current = ASSIST_CURRENT_MIN;
    if (current > ASSIST_CURRENT_MAX) current = ASSIST_CURRENT_MAX;

    float decay = (float)decay_rpm_s;
    if (decay < ASSIST_DECAY_MIN) decay = ASSIST_DECAY_MIN;
    if (decay > ASSIST_DECAY_MAX) decay = ASSIST_DECAY_MAX;

    atomic_store(&assist_strength_pct, (int)(current * 100.0f));
    atomic_store(&assist_decay_rpm_s, (int)decay);
    ESP_LOGI(THROTTLE_TAG, "Assist params: strength %d%%, decay %d rpm/s",
             (int)(current * 100.0f), (int)decay);
}

// Rider speed window for assistive push. A push has to reach a decent rolling
// speed before assist takes over — below that the rider is still getting going,
// and assist would be shoving a board that is barely moving. Above the cap it
// lets go rather than holding the board back downhill.
#define ASSIST_MIN_KMH      8.0f   // a push must reach this before assist takes it
#define ASSIST_MAX_KMH      20.0f  // above this assist lets go instead of holding back
#define ASSIST_RELEASE_KMH  2.0f   // wind down to here, then let the board roll to rest
#define ASSIST_FULL_KMH     2.0f   // this far below target, assist gives its full current

// Convert km/h to ERPM for this drivetrain — the inverse of the ERPM → km/h
// conversion in failsafe.c. Returns false until the VESC has sent its config,
// so assist stays idle rather than guessing thresholds for an unknown board.
static bool assist_erpm_window(float *min_erpm, float *max_erpm, float *release_erpm,
                               float *full_erpm, float *decay_erpm_s)
{
    mc_temp_config_t *mc = get_stored_mc_temp_config();
    if (!mc || !mc->valid || mc->motor_poles == 0 ||
        mc->gear_ratio <= 0.0f || mc->wheel_diameter <= 0.0f) {
        return false;
    }

    float pole_pairs = mc->motor_poles / 2.0f;
    // kmh -> wheel rpm -> motor rpm -> ERPM
    float erpm_per_kmh = (1000.0f / 60.0f) / ((float)M_PI * mc->wheel_diameter) *
                         mc->gear_ratio * pole_pairs;
    *min_erpm = ASSIST_MIN_KMH * erpm_per_kmh;
    *max_erpm = ASSIST_MAX_KMH * erpm_per_kmh;
    *release_erpm = ASSIST_RELEASE_KMH * erpm_per_kmh;
    *full_erpm = ASSIST_FULL_KMH * erpm_per_kmh;
    *decay_erpm_s = (float)atomic_load(&assist_decay_rpm_s) * pole_pairs;
    return true;
}

// Map a 0..255 throttle byte to the appropriate VESC command using smart-reverse logic:
//   - Above neutral while moving reverse → brake (regen against reverse motion)
//   - Above neutral while stopped or moving forward → drive forward (SET_CURRENT_REL +)
//   - Below neutral while moving forward → brake (regen against forward motion)
//   - Below neutral while stopped or moving reverse → drive reverse (SET_CURRENT_REL -)
// The last case is replaced by smart reverse (see smart_reverse.h), or brakes
// instead when reverse is disabled - the lockout wins. With assistive push, a kick
// at neutral is held and decayed on forward current instead of coasting:
// see assist_push.h.
// Lives in receiver firmware, so it works the same regardless of the rider's VESC app
// config. Current commands scale against each VESC's own |l_current_max| / |l_current_min|.
static void send_throttle_task(void *pvParameters) {
    smart_reverse_t smart_rev = {0};
    assist_push_t assist = {0};

    while (1) {
        // Failsafe task owns the motors while active — do not inject throttle.
        if (failsafe_is_active()) {
            smart_rev = (smart_reverse_t){0};  // re-arm from scratch after failsafe
            assist = (assist_push_t){0};
        } else {
            uint16_t raw = throttle_get_value();
            uint8_t value = (raw > 255) ? 255 : (uint8_t)raw;
            float brake = (value < THROTTLE_NEUTRAL_VALUE)
                ? (float)(THROTTLE_NEUTRAL_VALUE - value) / (float)THROTTLE_NEUTRAL_VALUE
                : 0.0f;
            bool stopped = bldc_interface_can_get_max_abs_erpm() < THROTTLE_STOPPED_ERPM;

            bool no_reverse = atomic_load(&no_reverse_enabled);
            bool smart_reverse = atomic_load(&smart_reverse_enabled) && !no_reverse;

            if (!smart_reverse) {
                smart_rev = (smart_reverse_t){0};  // turned off mid-reverse must not resume later
            }

            // Assistive push: sustains the speed the rider kicked up to with forward
            // current only, then bleeds it off. Never runs without a remote paired — a
            // push in the garage must not drive the board away — and any throttle or
            // brake input cancels it.
            float assist_min, assist_max, assist_release, assist_full, assist_decay;
            bool assist_ready = atomic_load(&assist_push_enabled) && ble_is_connected() &&
                                !bldc_interface_can_has_fault() &&
                                assist_erpm_window(&assist_min, &assist_max, &assist_release,
                                                   &assist_full, &assist_decay);
            if (!assist_ready) {
                assist = (assist_push_t){0};
            }

            bool was_engaged = assist.engaged;
            if (assist_ready &&
                assist_push_step(&assist, (float)bldc_interface_can_get_signed_dominant_erpm(),
                                 value != THROTTLE_NEUTRAL_VALUE, stopped, assist_min,
                                 assist_max, assist_release, assist_full,
                                 atomic_load(&assist_strength_pct) / 100.0f, assist_decay,
                                 THROTTLE_LOOP_MS / 1000.0f)) {
                if (!was_engaged) {
                    ESP_LOGI(THROTTLE_TAG, "Assist engaged at %.0f ERPM", assist.target);
                }
                // Forward current only — never a brake command, so a rider
                // pushing faster than the target is never held back.
                bldc_interface_can_set_current_rel_all(assist.current);
            } else if (was_engaged && !assist.engaged) {
                ESP_LOGI(THROTTLE_TAG, "Assist released");
                bldc_interface_can_set_current_rel_all(0.0f);  // coast, never brake
            } else if (smart_reverse &&
                smart_reverse_step(&smart_rev, brake, stopped, THROTTLE_LOOP_MS / 1000.0f)) {
                bldc_interface_can_set_duty_all(smart_rev.duty);
            } else if (value == THROTTLE_NEUTRAL_VALUE) {
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
                    } else if (smart_reverse || no_reverse) {
                        // Smart reverse only reverses via the duty ramp above; below its
                        // 92% entry point, or with reverse off, a stopped board holds brake.
                        bldc_interface_can_set_current_brake_rel_all(magnitude);
                    } else {
                        // Stopped or already reverse — drive reverse.
                        bldc_interface_can_set_current_rel_all(-magnitude);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(THROTTLE_LOOP_MS));
    }
}

