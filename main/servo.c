#include "servo.h"
#include "hw_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERVO";

/* 50 Hz servo PWM; use a dedicated timer (LEDC_TIMER_1) to avoid conflict with LED/aux. */
#define SERVO_LEDC_TIMER       LEDC_TIMER_1
#define SERVO_LEDC_CHANNEL     LEDC_CHANNEL_2
#define SERVO_PWM_FREQ_HZ      50
#define SERVO_PWM_RESOLUTION   LEDC_TIMER_14_BIT

/* Pulse width in microseconds for 0° and 180° (typical hobby servo range). */
#define SERVO_PULSE_US_MIN     500
#define SERVO_PULSE_US_MAX     2500

/* Period in us (20 ms at 50 Hz). */
#define SERVO_PERIOD_US        20000
#define SERVO_INIT_STEP_MS     30   /* ms per degree when sweeping 0 -> neutral at init */

static uint32_t duty_min_ticks = 0;
static uint32_t duty_max_ticks = 0;
static uint8_t last_logged_angle = 0xFF;  /* Force first log */

esp_err_t servo_init(void)
{
    const uint32_t max_duty = (1u << SERVO_PWM_RESOLUTION);

    /* Duty ticks for min/max pulse width. */
    duty_min_ticks = (SERVO_PULSE_US_MIN * max_duty) / SERVO_PERIOD_US;
    duty_max_ticks = (SERVO_PULSE_US_MAX * max_duty) / SERVO_PERIOD_US;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .timer_num       = SERVO_LEDC_TIMER,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start at 0°; we will sweep up to neutral in init */
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = duty_min_ticks,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty_min_ticks);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);

    /* Sweep from 0° to SERVO_NEUTRAL at startup */
    for (uint8_t angle = 0; angle <= SERVO_NEUTRAL; angle++) {
        uint32_t duty = duty_min_ticks + ((uint32_t)angle * (duty_max_ticks - duty_min_ticks)) / SERVO_ANGLE_MAX;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(SERVO_INIT_STEP_MS));
    }
    last_logged_angle = SERVO_NEUTRAL;

    ESP_LOGI(TAG, "Servo initialized on GPIO %d (angle mapped to throttle)", SERVO_GPIO);
    return ESP_OK;
}

void servo_set_angle(uint8_t angle_deg)
{
    if (angle_deg > SERVO_ANGLE_MAX) {
        angle_deg = SERVO_ANGLE_MAX;
    }

    uint32_t duty = duty_min_ticks + ((uint32_t)angle_deg * (duty_max_ticks - duty_min_ticks)) / SERVO_ANGLE_MAX;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);

    if (angle_deg != last_logged_angle) {
        last_logged_angle = angle_deg;
        ESP_LOGI(TAG, "Servo angle: %u°", (unsigned)angle_deg);
    }
}

void servo_set_angle_from_throttle(uint16_t throttle_value)
{
    if (throttle_value > 255) {
        throttle_value = 255;
    }
    /* Continuous-rotation: throttle 0 -> 50° (max CW), 127 -> 69° (stop), 255 -> 83° (max CCW). */
    uint8_t angle;
    if (throttle_value <= 127) {
        angle = (uint8_t)(SERVO_ANGLE_MAX_CW + ((SERVO_NEUTRAL - SERVO_ANGLE_MAX_CW) * throttle_value) / 127);
    } else {
        angle = (uint8_t)(SERVO_NEUTRAL + ((SERVO_ANGLE_MAX_CCW - SERVO_NEUTRAL) * (throttle_value - 127)) / (255 - 127));
    }
    servo_set_angle(angle);
}
