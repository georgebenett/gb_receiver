#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

// Pin definitions
#define LED_PIN                   GPIO_NUM_12

#define AUX_OUTPUT_GPIO           GPIO_NUM_6
#define AUX_OUTPUT_LED_PIN        GPIO_NUM_10

#define BMS_UART_TX_PIN           GPIO_NUM_16
#define BMS_UART_RX_PIN           GPIO_NUM_17

// UART configurations
#define BMS_UART_PORT             UART_NUM_2

// CAN pin definitions (using old VESC UART pins)
#define CAN_TX_PIN                GPIO_NUM_18   // Was UART1_VESC_TX_PIN
#define CAN_RX_PIN                GPIO_NUM_5 // Was UART1_VESC_RX_PIN
