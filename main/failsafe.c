#include "failsafe.h"
#include "bldc_interface_can.h"
#include "led.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

extern mc_temp_config_t *get_stored_mc_temp_config(void);

static const char *TAG = "FAILSAFE";

// Vehicle is considered stopped below this ERPM — full brake applies immediately with no ramp.
// Sized conservatively: ~30 RPM mechanical on a 7-pole motor = 210 ERPM.
#define ERPM_STOPPED_THRESHOLD   300

// Nunchuck Y-axis: 128 = neutral, 0 = full brake.
// We ramp from neutral down to BRAKE_Y_FINAL over BRAKE_RAMP_DURATION_MS,
// letting VESC's own ramp_time_neg smooth the actual current — avoiding the
// non-linear cliff of SET_CURRENT_BRAKE_REL.
#define BRAKE_Y_NEUTRAL          128    // hold value for stopped motors
#define BRAKE_Y_START            109    // ramp begins here — observed VESC nunchuck deadband edge
#define BRAKE_Y_FINAL            100    // tune toward 128 for softer, toward 0 for harder
#define BRAKE_RAMP_DURATION_MS   10000  // reach full brake in 10 s
#define BRAKE_RAMP_STEP_MS       50

// CAN fault is polled on this interval inside the failsafe task.
#define CAN_FAULT_POLL_MS        100

// Task runs at priority 10 — above all normal tasks (CAN RX, throttle, VESC poll at 5).
#define FAILSAFE_TASK_PRIORITY   10
#define FAILSAFE_TASK_STACK      4096

static volatile bool failsafe_active = false;
static volatile failsafe_reason_t last_reason = FAILSAFE_REASON_MANUAL;
static TaskHandle_t failsafe_task_handle = NULL;

// VESC dropout callback — wired to bldc_interface_can at init
static void on_vesc_dropout(uint8_t id) {
    ESP_LOGE(TAG, "VESC ID=%d dropped off CAN bus — triggering failsafe", id);
    failsafe_trigger(FAILSAFE_REASON_VESC_DROPOUT);
}

static void failsafe_task(void *pvParameters) {
    uint32_t last_can_check_ms = 0;

    while (1) {
        // Poll CAN bus fault independent of external triggers
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now_ms - last_can_check_ms >= CAN_FAULT_POLL_MS) {
            last_can_check_ms = now_ms;
            if (bldc_interface_can_has_fault() && !failsafe_active) {
                ESP_LOGE(TAG, "CAN bus fault detected — triggering failsafe");
                failsafe_trigger(FAILSAFE_REASON_CAN_FAULT);
            }
        }

        if (!failsafe_active) {
            // Block until notified (timeout allows periodic CAN fault polling)
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_FAULT_POLL_MS));
            continue;
        }

        // --- Failsafe active: execute ramped braking ---
        ESP_LOGW(TAG, "Failsafe active — reason=%d, beginning brake ramp", (int)last_reason);
        led_set_connection_state(false);

        uint32_t ramp_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool ramp_done = false;

        while (failsafe_active) {
            uint32_t elapsed_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - ramp_start_ms;

            // Once all motors have stopped, hold neutral — do NOT continue sending a
            // below-neutral nunchuck value because the VESC will interpret it as reverse
            // drive once the motor is no longer spinning forward.
            int32_t max_erpm = bldc_interface_can_get_max_abs_erpm();
            uint8_t y_value;
            if (max_erpm < ERPM_STOPPED_THRESHOLD) {
                y_value = BRAKE_Y_NEUTRAL;
                if (!ramp_done) {
                    ESP_LOGI(TAG, "All VESCs stopped — holding neutral to prevent reverse");
                    ramp_done = true;
                }
            } else {
                float progress = (float)elapsed_ms / (float)BRAKE_RAMP_DURATION_MS;
                if (progress > 1.0f) progress = 1.0f;
                y_value = (uint8_t)(BRAKE_Y_START - (BRAKE_Y_START - BRAKE_Y_FINAL) * progress);
                if (!ramp_done && elapsed_ms >= BRAKE_RAMP_DURATION_MS) {
                    ESP_LOGI(TAG, "Brake ramp complete — holding at y=%d", BRAKE_Y_FINAL);
                    ramp_done = true;
                }
            }

            // Send per-VESC nunchuck command — stopped VESCs get neutral to avoid reverse drive
            mc_temp_config_t *mc = get_stored_mc_temp_config();
            uint8_t vesc_count = bldc_interface_can_get_detected_vesc_count();
            for (uint8_t v = 0; v < vesc_count; v++) {
                uint8_t vesc_id  = bldc_interface_can_get_id_at(v);
                int32_t erpm     = bldc_interface_can_get_erpm_at(v);
                // Only brake motors spinning forward. A stopped or reverse-spinning
                // motor must get neutral — y < 128 is reverse throttle for negative ERPM.
                uint8_t vesc_y = (erpm > ERPM_STOPPED_THRESHOLD) ? y_value : BRAKE_Y_NEUTRAL;

                uint8_t buf[5];
                int32_t idx = 0;
                buf[idx++] = COMM_SET_CHUCK_DATA;
                buf[idx++] = 128;
                buf[idx++] = vesc_y;
                buf[idx++] = 0;
                buf[idx++] = 0;
                bldc_interface_can_send_to_id(vesc_id, buf, idx);

                float speed_kmh = 0.0f;
                if (mc && mc->valid && mc->motor_poles > 0 && mc->gear_ratio > 0.0f && mc->wheel_diameter > 0.0f) {
                    int32_t abs_erpm = erpm < 0 ? -erpm : erpm;
                    float mech_rpm  = (float)abs_erpm / (mc->motor_poles / 2.0f);
                    float wheel_rpm = mech_rpm / mc->gear_ratio;
                    speed_kmh = wheel_rpm * (float)M_PI * mc->wheel_diameter / 60.0f * 3.6f;
                }
                ESP_LOGI(TAG, "BRAKE_CURVE %"PRIu32",%d,%d,%"PRId32",%.2f",
                         elapsed_ms, vesc_y, vesc_id, erpm, speed_kmh);
            }

            vTaskDelay(pdMS_TO_TICKS(BRAKE_RAMP_STEP_MS));
        }

        ESP_LOGI(TAG, "Failsafe cleared");
    }
}

void failsafe_init(void) {
    bldc_interface_can_set_vesc_dropout_func(on_vesc_dropout);

    xTaskCreate(failsafe_task, "failsafe", FAILSAFE_TASK_STACK, NULL,
                FAILSAFE_TASK_PRIORITY, &failsafe_task_handle);

    ESP_LOGI(TAG, "Failsafe initialized (priority=%d, stopped_threshold=%d ERPM)",
             FAILSAFE_TASK_PRIORITY, ERPM_STOPPED_THRESHOLD);
}

void failsafe_trigger(failsafe_reason_t reason) {
    if (!failsafe_active) {
        last_reason = reason;
        failsafe_active = true;
        ESP_LOGE(TAG, "FAILSAFE TRIGGERED reason=%d", (int)reason);
    }
    // Wake task immediately regardless of whether it was already active
    if (failsafe_task_handle) {
        xTaskNotifyGive(failsafe_task_handle);
    }
}

bool failsafe_is_active(void) {
    return failsafe_active;
}

bool failsafe_try_clear(void) {
    if (!failsafe_active) {
        return true;
    }

    // Refuse to clear if any VESC is still moving
    int32_t max_erpm = bldc_interface_can_get_max_abs_erpm();
    if (max_erpm >= ERPM_STOPPED_THRESHOLD) {
        ESP_LOGW(TAG, "Failsafe clear refused — vehicle still moving (ERPM=%"PRId32")", max_erpm);
        return false;
    }

    // Refuse to clear if the original fault condition persists
    if (last_reason == FAILSAFE_REASON_CAN_FAULT && bldc_interface_can_has_fault()) {
        ESP_LOGW(TAG, "Failsafe clear refused — CAN fault still active");
        return false;
    }

    failsafe_active = false;
    ESP_LOGI(TAG, "Failsafe cleared by operator");
    return true;
}
