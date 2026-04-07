/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include "esp_err.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/*
 * DEFINES
 ****************************************************************************************
 */
//#define SUPPORT_HEARTBEAT
//#define SPP_DEBUG_MODE

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_CHAR,
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

// Paired remote NVS keys (receiver stores the last connected remote's MAC)
#define BLE_PAIRED_REMOTE_NVS_NAMESPACE "ble_paired"
#define BLE_PAIRED_REMOTE_NVS_KEY_MAC   "remote_mac"
#define BLE_PAIRED_REMOTE_NVS_KEY_VALID "mac_valid"

// How long to prefer the previously paired remote on boot before opening to all
#define BLE_PREFER_PAIRED_MS 15000  // 15 seconds

// Initialize the BLE SPP server
esp_err_t ble_spp_server_init(void);

// Start the BLE SPP server
esp_err_t ble_spp_server_start(void);

// Get BLE connection status
bool ble_is_connected(void);

// Reset trip distance stored on receiver (called from USB serial)
void ble_reset_trip_distance(void);
