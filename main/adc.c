#include "adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <stdio.h>
#include "bldc_interface.h"
#include "hw_config.h"
#include "led.h"

#define ADC_TAG "ADC"

uint16_t current_adc_value = THROTTLE_NEUTRAL_VALUE;

static TimerHandle_t adc_timeout_timer = NULL;
static bool timeout_monitoring_active = false;

static void adc_timeout_callback(TimerHandle_t xTimer);
static void send_nunchuck_throttle(void *pvParameters);

//TaskHandle_t adc_print_task_handle = NULL;

esp_err_t adc_init(void)
{
    // Create task for nunchuck testing
    xTaskCreate(send_nunchuck_throttle, "nunchuck_test", 2048, NULL, 5, NULL);

    ESP_LOGI(ADC_TAG, "ADC initialized");

    // Create the timeout timer
    adc_timeout_timer = xTimerCreate(
        "adc_timeout",
        pdMS_TO_TICKS(ADC_TIMEOUT_MS),
        pdTRUE,              // Auto reload
        NULL,
        adc_timeout_callback
    );

    if (adc_timeout_timer == NULL) {
        ESP_LOGE(ADC_TAG, "Failed to create ADC timeout timer");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void adc_update_value(uint16_t value)
{
    current_adc_value = value;
    adc_reset_timeout();  // Reset the timeout timer
    led_set_connection_state(true);  // Set LED to connected state on packet reception
    ESP_LOGD(ADC_TAG, "%d", value);
}

void adc_reset_value(void)
{
    current_adc_value = THROTTLE_NEUTRAL_VALUE;
}

void adc_timeout_callback(TimerHandle_t xTimer)
{
    adc_reset_value();
    led_set_connection_state(false);  // Set LED to disconnected state on timeout
}
/*
void adc_print_task(void *pvParameters)
{
    while (1) {
        mc_values* vesc_values = get_stored_vesc_values();
        ESP_LOGI(ADC_TAG, "ADC: %d | VESC Voltage: %.2fV | Motor RPM: %.1f",
            current_adc_value,
            vesc_values->v_in,
            vesc_values->rpm);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}*/

void adc_reset_timeout(void)
{
    if (timeout_monitoring_active && adc_timeout_timer != NULL) {
        if (xTimerReset(adc_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(ADC_TAG, "Failed to reset ADC timeout timer");
        }
    }
}

void adc_start_timeout_monitor(void)
{
    if (adc_timeout_timer != NULL) {
        if (xTimerStart(adc_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(ADC_TAG, "Failed to start ADC timeout timer");
        } else {
            timeout_monitoring_active = true;
            led_set_connection_state(false);  // Start with LED in disconnected state
            ESP_LOGI(ADC_TAG, "ADC timeout monitoring started");
        }
    }
}

void adc_stop_timeout_monitor(void)
{
    if (adc_timeout_timer != NULL) {
        if (xTimerStop(adc_timeout_timer, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(ADC_TAG, "Failed to stop ADC timeout timer");
        } else {
            timeout_monitoring_active = false;
            ESP_LOGI(ADC_TAG, "ADC timeout monitoring stopped");
        }
    }
}

static void send_nunchuck_throttle(void *pvParameters) {

    while (1) {
        uint8_t y_value = current_adc_value;
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
