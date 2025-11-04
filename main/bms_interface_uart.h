#pragma once

#include "esp_err.h"

/**
 * @brief Initialize BMS UART interface
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t bms_interface_uart_init(void);

/**
 * @brief Send function for UART communication
 * @param data Pointer to data to send
 * @param len Length of data to send
 */
void bms_interface_uart_send_function(unsigned char *data, unsigned int len);

/**
 * @brief UART receive task
 * @param pvParameters Task parameters (unused)
 */
void bms_interface_uart_rx_task(void *pvParameters);
