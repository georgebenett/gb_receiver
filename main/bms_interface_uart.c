#include "bms_interface_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "bldc_interface_can.h"
#include "bldc_interface.h"
#include "hw_config.h"

#define BMS_UART_TAG "BMS_UART"

esp_err_t bms_interface_uart_init(void)
{
    // BMS UART is only for BMS communication (UART_NUM_2)
    // VESC communication is now via CAN, so no VESC UART setup needed
    
    ESP_LOGI(BMS_UART_TAG, "BMS UART interface initialized (VESC now uses CAN)");

    return ESP_OK;
}

void bms_interface_uart_send_function(unsigned char *data, unsigned int len)
{
    // Send to VESC via CAN instead of UART
    bldc_interface_can_send_packet(data, len);
}
