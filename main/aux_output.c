#include "aux_output.h"
#include "hw_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "throttle.h"

static const char *TAG = "AUX_OUTPUT";

// PWM configuration for aux output LED
#define AUX_LED_PWM_FREQ         5000        // 5 kHz (same as main LED)
#define AUX_LED_PWM_RESOLUTION   8           // 8-bit resolution (0-255)
#define AUX_LED_PWM_TIMER        LEDC_TIMER_0  // Share timer with main LED
#define AUX_LED_PWM_CHANNEL      LEDC_CHANNEL_1 // Different channel from main LED

// PWM duty scaling configuration
#define AUX_LED_PWM_MAX_DUTY     50         // Maximum PWM duty value (out of 255)
#define THROTTLE_MAX_VALUE       255         // Maximum throttle value from BLE

static uint8_t aux_output_state = 0;

esp_err_t aux_output_init(void) {
    // Configure GPIO for aux output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AUX_OUTPUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start with output OFF
    gpio_set_level(AUX_OUTPUT_GPIO, 0);

    // Configure PWM for aux output LED
    // Note: Timer is shared with main LED, so we only configure the channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = AUX_OUTPUT_LED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = AUX_LED_PWM_CHANNEL,
        .timer_sel = AUX_LED_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE
    };

    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED PWM channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initial PWM duty to 0 (OFF)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL);

    ESP_LOGI(TAG, "Auxiliary output initialized on GPIO %d, LED PWM on GPIO %d",
             AUX_OUTPUT_GPIO, AUX_OUTPUT_LED_PIN);
    return ESP_OK;
}

void aux_output_set(uint8_t state) {
    aux_output_state = state ? 1 : 0;
    gpio_set_level(AUX_OUTPUT_GPIO, aux_output_state);

    // Update PWM to reflect the new state
    aux_output_update_pwm();

    //ESP_LOGI(TAG, "Aux output set to: %s", state ? "ON" : "OFF");
}

void aux_output_update_pwm(void) {
    uint32_t duty_scaled = 0;

    if (aux_output_state) {
        // When ON, scale throttle value to PWM duty
        // Scale from throttle range (0-THROTTLE_MAX_VALUE) to PWM duty (0-AUX_LED_PWM_MAX_DUTY)
        duty_scaled = (current_throttle_value * AUX_LED_PWM_MAX_DUTY) / THROTTLE_MAX_VALUE;
    } else {
        // When OFF, PWM duty is 0
        duty_scaled = 0;
    }

    // Set LEDC duty (already in 0-255 range for 8-bit resolution)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL, duty_scaled);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL);
}

