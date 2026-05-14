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

    SPP_IDX_NB,
};

#define BLE_PAIRED_REMOTE_NVS_NAMESPACE "ble_paired"
#define BLE_PAIRED_REMOTE_NVS_KEY_LIST  "paired_list"
// Legacy keys, migrated on first boot of this firmware version.
#define BLE_PAIRED_REMOTE_NVS_KEY_MAC   "remote_mac"
#define BLE_PAIRED_REMOTE_NVS_KEY_VALID "mac_valid"

#define BLE_MAX_PAIRED_REMOTES 4
#define BLE_SCAN_DURATION_S    8
#define BLE_SCAN_MAX_RESULTS   16
#define BLE_SCAN_NAME_MAX      16

typedef struct {
    uint8_t mac[6];
    uint8_t addr_type;
} ble_paired_entry_t;

typedef struct {
    uint8_t mac[6];
    uint8_t addr_type;
    int8_t  rssi;
    uint8_t name_len;
    char    name[BLE_SCAN_NAME_MAX];
} ble_scan_entry_t;

esp_err_t ble_spp_server_init(void);
esp_err_t ble_spp_server_start(void);
bool ble_is_connected(void);
void ble_reset_trip_distance(void);
float ble_get_trip_km(void);
// Request the BLE module to re-evaluate advertising state (used by the Wi-Fi
// fallback when it starts / stops, so adv is paused while the AP is up).
void ble_refresh_advertising(void);

// Discover remotes by scanning for advertised name "GS-REM".
// Returns false if a remote is connected (radio busy) or scan already running.
bool ble_start_scan(void);
bool ble_is_scanning(void);
uint8_t ble_get_scan_results(ble_scan_entry_t *out, uint8_t max_count);

// Persistent whitelist of remotes allowed to connect.
uint8_t ble_get_paired_count(void);
uint8_t ble_get_paired_list(ble_paired_entry_t *out, uint8_t max_count);
// Returns false (and leaves out untouched) if no remote is connected.
bool ble_get_connected_mac(uint8_t out[6]);
// Returns false if the list is full or that MAC is already paired.
bool ble_pair_remote(const uint8_t mac[6], uint8_t addr_type);
// Returns false if the MAC is not in the paired list.
bool ble_unpair_remote_by_mac(const uint8_t mac[6]);
