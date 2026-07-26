#include "led.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hw_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "LED";
static uint8_t current_duty = 0;

void led_transition_task(void *pvParameters);
static volatile uint8_t pending_target_duty = 0;
static TaskHandle_t transition_task_handle = NULL;
static bool current_connected_state = false;
static bool connection_state_known = false;

esp_err_t led_init(void)
{
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LED_PWM_RESOLUTION,
        .timer_num = LED_PWM_TIMER,
        .freq_hz = LED_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LED_PWM_CHANNEL,
        .timer_sel = LED_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Create the persistent transition task (starts suspended, waiting for notifications)
    xTaskCreate(led_transition_task,
                "led_transition",
                2048,
                NULL,
                1,
                &transition_task_handle);

    // Set initial state (disconnected)
    led_set_connection_state(false);

    ESP_LOGI(TAG, "LED initialized");
    return ESP_OK;
}

void led_set_duty(uint8_t duty)
{
    uint32_t duty_scaled = (duty * ((1 << LED_PWM_RESOLUTION) - 1)) / 255;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_PWM_CHANNEL, duty_scaled);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_PWM_CHANNEL);
    current_duty = duty;
}

void led_transition_task(void *pvParameters)
{
    while (1) {
        // Block indefinitely until a new target is signalled
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t target_duty = pending_target_duty;
        float start_duty = current_duty;
        float duty_diff = (float)target_duty - start_duty;

        for (int i = 0; i < LED_TRANSITION_STEPS; i++) {
            // If a new target arrived mid-transition, restart from current position
            if (ulTaskNotifyTake(pdTRUE, 0)) {
                target_duty = pending_target_duty;
                start_duty = current_duty;
                duty_diff = (float)target_duty - start_duty;
                i = 0;
            }

            float progress = (float)i / (LED_TRANSITION_STEPS - 1);
            float smooth_progress = (1.0f - cosf(progress * M_PI)) / 2.0f;
            uint8_t new_duty = (uint8_t)(start_duty + (duty_diff * smooth_progress));

            led_set_duty(new_duty);
            vTaskDelay(pdMS_TO_TICKS(LED_TRANSITION_STEP_MS));
        }

        // Ensure we reach exactly the target value
        led_set_duty(target_duty);
    }
}

void led_set_connection_state(bool connected)
{
    // Ignore redundant calls (e.g. one per received packet) so an in-progress
    // fade isn't constantly restarted and never allowed to complete.
    if (connection_state_known && connected == current_connected_state) {
        return;
    }
    connection_state_known = true;
    current_connected_state = connected;

    pending_target_duty = connected ? LED_PWM_CONNECTED : LED_PWM_DISCONNECTED;

    if (transition_task_handle != NULL) {
        xTaskNotifyGive(transition_task_handle);
    }
}
