#include "aux_output.h"
#include "hw_config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "AUX_OUTPUT";

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
    
    // Start with output OFF
    gpio_set_level(AUX_OUTPUT_GPIO, 0);
    
    ESP_LOGI(TAG, "Auxiliary output initialized on GPIO %d", AUX_OUTPUT_GPIO);
    return ESP_OK;
}

void aux_output_set(uint8_t state) {
    gpio_set_level(AUX_OUTPUT_GPIO, state ? 1 : 0);
    //ESP_LOGI(TAG, "Aux output set to: %s", state ? "ON" : "OFF");
}

