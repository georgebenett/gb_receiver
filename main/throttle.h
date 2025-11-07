#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "datatypes.h"
#include "esp_err.h"

#define THROTTLE_TIMEOUT_MS 200  // 200ms timeout
#define THROTTLE_NEUTRAL_VALUE 127
#define VESC_UPDATE_INTERVAL_MS 50

extern uint16_t current_throttle_value;
extern TaskHandle_t throttle_print_task_handle;

// Function declarations
esp_err_t throttle_init(void);
void throttle_update_value(uint16_t value);
void throttle_reset_value(void);
void throttle_reset_timeout(void);
void throttle_start_timeout_monitor(void);
void throttle_stop_timeout_monitor(void);
//void throttle_print_task(void *pvParameters);

// External variable declarations
//extern TaskHandle_t throttle_print_task_handle;
//extern mc_values* get_stored_vesc_values(void);

