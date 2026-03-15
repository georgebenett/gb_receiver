#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

// Pin definitions
#define LED_PIN                   GPIO_NUM_10

#define AUX_OUTPUT_GPIO           GPIO_NUM_1
#define AUX_OUTPUT_LED_PIN        GPIO_NUM_0

// BMS UART: two GPIOs; TX/RX are auto-detected at init by probing received data
#define BMS_UART_PIN_A            GPIO_NUM_6
#define BMS_UART_PIN_B            GPIO_NUM_7

// UART configurations
#define BMS_UART_PORT             UART_NUM_1

#define CAN_TX_PIN                GPIO_NUM_4
#define CAN_RX_PIN                GPIO_NUM_5
