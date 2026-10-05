#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "datatypes.h"
#include "esp_err.h"
#include <stdbool.h>

#define THROTTLE_TIMEOUT_MS 200  // 200ms timeout
#define THROTTLE_NEUTRAL_VALUE 128

// Function declarations
esp_err_t throttle_init(void);
uint16_t throttle_get_value(void);
void throttle_update_value(uint16_t value);
void throttle_reset_value(void);
void throttle_reset_timeout(void);
void throttle_start_timeout_monitor(void);
void throttle_stop_timeout_monitor(void);
void throttle_set_smart_reverse(bool enabled);  // setting owned by the remote, sent over BLE
void throttle_set_no_reverse(bool enabled);     // setting owned by the remote, sent over BLE
void throttle_set_assist_push(bool enabled);    // setting owned by the remote, sent over BLE
void throttle_set_assist_params(uint8_t strength_pct, uint8_t decay_rpm_s);  // ditto
void throttle_get_settings(uint8_t out[4]);  // [flags smart|no_rev<<1|assist<<2, speed_limit, strength%, decay] echoed to the remote
void throttle_set_current_scale(uint8_t pct);  // ride profile's share of max drive current, 1..100; brakes unscaled
void throttle_set_speed_limit(uint8_t cap_kmh);  // setting owned by the remote, sent over BLE; 0 = off

