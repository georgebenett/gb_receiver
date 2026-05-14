#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Start the SoftAP if no BLE remote has been connected for this long.
#define WIFI_AP_IDLE_TIMEOUT_MS 60000

esp_err_t wifi_ap_init(void);
esp_err_t wifi_ap_start(void);
esp_err_t wifi_ap_stop(void);
bool wifi_ap_is_running(void);
// Stop the AP after `delay_ms`. Used right after a successful pair so the HTTP
// response can flush and the radio then frees up for the new BLE link.
void wifi_ap_stop_after(uint32_t delay_ms);
