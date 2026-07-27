#include "aux_output.h"
#include "hw_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs_flash.h"
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

// NVS storage for aux output state
#define AUX_NVS_NAMESPACE  "aux_cfg"
#define AUX_NVS_KEY_STATE  "aux_state"

static uint8_t aux_output_state = 0;

static void aux_output_save_state(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUX_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open aux NVS: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u8(handle, AUX_NVS_KEY_STATE, aux_output_state);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Aux NVS commit failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "Aux NVS set failed: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
}

static void aux_output_load_state(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(AUX_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        uint8_t state = 0;
        if (nvs_get_u8(handle, AUX_NVS_KEY_STATE, &state) == ESP_OK) {
            aux_output_state = state;
        }
        nvs_close(handle);
    }
}

esp_err_t aux_output_init(void) {
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

    aux_output_load_state();
    gpio_set_level(AUX_OUTPUT_GPIO, aux_output_state);

    // Note: Timer is shared with main LED
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

    ledc_set_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL);

    ESP_LOGI(TAG, "Auxiliary output initialized on GPIO %d, LED PWM on GPIO %d",
             AUX_OUTPUT_GPIO, AUX_OUTPUT_LED_PIN);
    return ESP_OK;
}

void aux_output_set(uint8_t state) {
    aux_output_state = state ? 1 : 0;
    gpio_set_level(AUX_OUTPUT_GPIO, aux_output_state);
    aux_output_save_state();

    aux_output_update_pwm();
}

uint8_t aux_output_get_state(void) {
    return aux_output_state;
}

void aux_output_update_pwm(void) {
    uint32_t duty_scaled = 0;

    if (aux_output_state) {
        // When ON, scale throttle value to PWM duty
        duty_scaled = (throttle_get_value() * AUX_LED_PWM_MAX_DUTY) / THROTTLE_MAX_VALUE;
    } else {
        duty_scaled = 0;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL, duty_scaled);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, AUX_LED_PWM_CHANNEL);
}

