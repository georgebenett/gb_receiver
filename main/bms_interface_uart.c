#include "bms_interface_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "bldc_interface_uart.h"
#include "bldc_interface.h"
#include "hw_config.h"

#define BMS_UART_TAG "BMS_UART"

static void configure_uart(void);

esp_err_t bms_interface_uart_init(void)
{
    // Configure UART
    configure_uart();

    // Create task to read UART data with increased stack size
    xTaskCreate(bms_interface_uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);

    ESP_LOGI(BMS_UART_TAG, "BMS UART interface initialized");

    return ESP_OK;
}

void bms_interface_uart_send_function(unsigned char *data, unsigned int len)
{
    uart_write_bytes(UART_NUM_1, (const char*)data, len);
}

void bms_interface_uart_rx_task(void *pvParameters)
{
    uint8_t data[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, data, sizeof(data), pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++) {
            bldc_interface_uart_process_byte(data[i]);
        }
        bldc_interface_uart_run_timer();
    }
}

static void configure_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters for VESC only
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));

    // Set UART pins for VESC
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART1_VESC_TX_PIN, UART1_VESC_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver for VESC only
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0));
}
