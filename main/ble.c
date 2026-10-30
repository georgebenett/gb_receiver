/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/gpio.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "ble.h"
#include "esp_gatt_common_api.h"
#include "throttle.h"
#include "failsafe.h"
#include "led.h"
#include "version.h"
#include "bms.h"
#include "datatypes.h"
#include "bldc_interface_can.h"
#include "aux_output.h"
#include "wifi_ap.h"
#include "esp_ota_ops.h"
#include "freertos/semphr.h"
#include "miniz.h"

extern mc_values* get_stored_vesc_values(void);
extern bms_values_t* get_stored_bms_values(void);
extern mc_temp_config_t* get_stored_mc_temp_config(void);

#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"
#define CLIENT_NAME      "GS-THUMB"

#define TRIP_NVS_NAMESPACE   "trip_data"
#define TRIP_NVS_KEY_KM      "trip_km"
#define TRIP_NVS_SAVE_INTERVAL_KM 0.1f  // save every 100 meters

#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          CLIENT_NAME
#define SPP_SVC_INST_ID             0

// BLE Security Configuration
#define BLE_PASSKEY                 483265  // Fixed passkey for pairing
#define BLE_CMD_RESET_ODOMETER      0x01   // Command: reset trip odometer
#define BLE_CMD_SHUTDOWN            0x02   // Command: remote is powering off intentionally
#define BLE_CMD_SET_SMART_REVERSE   0x03   // Command: [0x03, enabled] smart reverse setting from the remote
#define BLE_CMD_SET_ASSIST_PUSH      0x04   // Command: [0x04, enabled, strength%, decay] from the remote
#define BLE_CMD_SET_NO_REVERSE      0x05   // Command: [0x05, enabled] reverse lockout from the remote
#define BLE_CMD_SET_CURRENT_SCALE   0x09   // Command: [0x09, pct] drive current share from the remote's ride profile
#define BLE_CMD_SET_SPEED_LIMIT     0x06   // Command: [0x06, cap_kmh] speed limit from the remote, 0 = off
#define BLE_CMD_GET_FW_VERSION      0x07   // Command: [0x07], replied on status as [0x07, major, minor, patch, mac x6]
#define BLE_STATUS_FRAME            0x08   // Status, once a second: see send_status_frame()
// Firmware update from the remote. Every reply is [cmd, status, next_offset u32 LE] on status.
#define BLE_CMD_OTA_BEGIN           0x0A   // Command: [0x0A, zlib_len u32, image_len u32] -> ERASING, then OK
#define BLE_CMD_OTA_DATA            0x0B   // Command: [0x0B, offset u32, zlib bytes...]
#define BLE_CMD_OTA_END             0x0C   // Command: [0x0C] verify, switch boot slot, reboot; early, it aborts
#define OTA_ST_OK                   0
#define OTA_ST_REFUSED              1      // wheels turning, or no update in progress
#define OTA_ST_FLASH_ERR            2
#define OTA_ST_BAD_OFFSET           3      // next_offset says where to resume
#define OTA_ST_ERASING              4      // BEGIN accepted; OK follows once erased
#define OTA_MAX_STANDSTILL_ERPM     100.0f
#define OTA_ACK_EVERY               4096   // bytes between DATA acks
#define OTA_WINDOW                  8192   // inflate ring; the remote packs with an 8 KB zlib window
#define OTA_BLOCK                   0x10000
#define OTA_SECTOR                  0x1000


static const uint16_t spp_service_uuid = 0xABF0;
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

/* Little-endian writers for the telemetry frame. Return the byte count so the
 * packer reads as `idx += put_u16(&buffer[idx], value);`. */
static uint16_t put_u16(uint8_t *p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    return 2;
}

static uint16_t put_u32(uint8_t *p, uint32_t v) {
    put_u16(p, (uint16_t)(v & 0xFFFF));
    put_u16(p + 2, (uint16_t)(v >> 16));
    return 4;
}

static void send_telemetry_task(void *pvParameters);
static void trip_nvs_load(void);
static void trip_nvs_save(void);

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'G', 'S', '-', 'T', 'H', 'U', 'M', 'B', 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
static bool enable_data_ntf = false;
static bool is_connected = false;
static bool is_authenticated = false;  // Connection is encrypted/authenticated
static bool remote_shutdown_requested = false;  // BLE_CMD_SHUTDOWN seen before disconnect

// BLE firmware update in progress. Throttle is held neutral while set.
static volatile bool ota_active = false;
static const esp_partition_t *ota_part = NULL;
static uint32_t ota_size = 0;     // zlib stream bytes
static uint32_t ota_written = 0;  // zlib stream bytes received
static uint32_t ota_raw = 0;      // app image bytes
static uint32_t ota_out = 0;      // app image bytes written to ota_part

static float trip_km = 0.0f;
static uint32_t trip_last_update_ms = 0;
static float trip_km_nvs_committed = 0.0f;

static esp_bd_addr_t spp_remote_bda = {0x0,};

static ble_paired_entry_t paired_list[BLE_MAX_PAIRED_REMOTES];
static uint8_t paired_count = 0;
static TaskHandle_t adv_task_handle = NULL;

static ble_scan_entry_t scan_results[BLE_SCAN_MAX_RESULTS];
static uint8_t scan_result_count = 0;
static bool scan_active = false;

// Brings up the SoftAP fallback after a stretch of no remote connection.
static TimerHandle_t wifi_idle_timer = NULL;

// adv_task notification bits — keep BLE stack calls off timer/GATT callbacks.
#define ADV_TASK_NOTIFY_REFRESH_ADV      (1U << 0)
#define ADV_TASK_NOTIFY_RELOAD_PAIRED    (1U << 1)

static uint16_t spp_handle_table[SPP_IDX_NB];
static TaskHandle_t telemetry_task_handle = NULL;

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

// --- Paired remote NVS helpers ---

static int paired_find_index(const uint8_t mac[6])
{
    for (int i = 0; i < paired_count; i++) {
        if (memcmp(paired_list[i].mac, mac, 6) == 0) return i;
    }
    return -1;
}

static void paired_save_nvs(void)
{
    nvs_handle_t nvs_handle;
    if (nvs_open(BLE_PAIRED_REMOTE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK) return;
    if (paired_count == 0) {
        nvs_erase_key(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_LIST);
    } else {
        nvs_set_blob(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_LIST,
                     paired_list, paired_count * sizeof(ble_paired_entry_t));
    }
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

static void paired_load_nvs(void)
{
    paired_count = 0;
    nvs_handle_t nvs_handle;
    if (nvs_open(BLE_PAIRED_REMOTE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK) return;

    size_t len = sizeof(paired_list);
    esp_err_t err = nvs_get_blob(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_LIST, paired_list, &len);
    if (err == ESP_OK) {
        paired_count = len / sizeof(ble_paired_entry_t);
        if (paired_count > BLE_MAX_PAIRED_REMOTES) paired_count = BLE_MAX_PAIRED_REMOTES;
    } else {
        // Migrate legacy single-MAC schema if present.
        uint8_t valid = 0;
        size_t mac_len = 6;
        uint8_t legacy_mac[6];
        if (nvs_get_u8(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_VALID, &valid) == ESP_OK && valid == 1 &&
            nvs_get_blob(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_MAC, legacy_mac, &mac_len) == ESP_OK) {
            memcpy(paired_list[0].mac, legacy_mac, 6);
            paired_list[0].addr_type = BLE_WL_ADDR_TYPE_PUBLIC;
            paired_count = 1;
            nvs_set_blob(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_LIST,
                         paired_list, sizeof(ble_paired_entry_t));
            nvs_commit(nvs_handle);
            nvs_erase_key(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_MAC);
            nvs_erase_key(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_VALID);
            nvs_commit(nvs_handle);
            ESP_LOGI(GATTS_TABLE_TAG, "Migrated legacy paired MAC to new schema");
        }
    }
    nvs_close(nvs_handle);
    ESP_LOGI(GATTS_TABLE_TAG, "Loaded %d paired remote(s)", paired_count);
}

// Reconcile controller whitelist with paired_list[].
static void whitelist_sync(void)
{
    esp_ble_gap_clear_whitelist();
    for (int i = 0; i < paired_count; i++) {
        esp_ble_gap_update_whitelist(true, paired_list[i].mac, paired_list[i].addr_type);
    }
}

// Advertise iff we have ≥1 paired remote and aren't already connected.
// We let adv run alongside the Wi-Fi AP — the AP is dropped shortly after a
// successful pair so the radio is free for the new BLE link to come up.
static void apply_advertising_state(void)
{
    esp_ble_gap_stop_advertising();
    if (is_connected) return;
    if (paired_count == 0) return;
    spp_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST;
    esp_ble_gap_start_advertising(&spp_adv_params);
}

static void adv_task(void *pvParameters)
{
    uint32_t notification = 0;
    while (1) {
        if (xTaskNotifyWait(0, UINT32_MAX, &notification, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        // Filter-accept-list ops fail with HCI "Cmd Disallowed" while
        // advertising is using the list, so pause adv around the resync.
        if (notification & ADV_TASK_NOTIFY_RELOAD_PAIRED) {
            esp_ble_gap_stop_advertising();
            vTaskDelay(pdMS_TO_TICKS(50));
            whitelist_sync();
            apply_advertising_state();
        } else if (notification & ADV_TASK_NOTIFY_REFRESH_ADV) {
            apply_advertising_state();
        }
    }
}

static void wifi_idle_timer_cb(TimerHandle_t t)
{
    if (!is_connected) {
        wifi_ap_start();
    }
}

static void wifi_idle_timer_kick(void)
{
    if (wifi_idle_timer == NULL) return;
    xTimerStop(wifi_idle_timer, 0);
    xTimerChangePeriod(wifi_idle_timer, pdMS_TO_TICKS(WIFI_AP_IDLE_TIMEOUT_MS), 0);
    xTimerStart(wifi_idle_timer, 0);
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGI(GATTS_TABLE_TAG, "GAP_EVT, event %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        apply_advertising_state();
        break;

    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        // Triggered after esp_ble_gap_set_scan_params during a scan request.
        if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS && scan_active) {
            esp_ble_gap_start_scanning(BLE_SCAN_DURATION_S);
        } else if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            scan_active = false;
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        struct ble_scan_result_evt_param *r = &param->scan_rst;
        if (r->search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            uint8_t adv_name_len = 0;
            uint8_t *adv_name = esp_ble_resolve_adv_data(r->ble_adv,
                                                        ESP_BLE_AD_TYPE_NAME_CMPL,
                                                        &adv_name_len);
            if (adv_name == NULL || adv_name_len < 6 ||
                memcmp(adv_name, "GS-REM", 6) != 0) {
                break;
            }
            // De-dup by MAC; refresh RSSI on repeat sightings.
            int slot = -1;
            for (int i = 0; i < scan_result_count; i++) {
                if (memcmp(scan_results[i].mac, r->bda, 6) == 0) { slot = i; break; }
            }
            if (slot < 0) {
                if (scan_result_count >= BLE_SCAN_MAX_RESULTS) break;
                slot = scan_result_count++;
                memcpy(scan_results[slot].mac, r->bda, 6);
                scan_results[slot].addr_type = r->ble_addr_type;
            }
            scan_results[slot].rssi = r->rssi;
            uint8_t copy_len = adv_name_len < BLE_SCAN_NAME_MAX - 1 ? adv_name_len
                                                                   : BLE_SCAN_NAME_MAX - 1;
            memcpy(scan_results[slot].name, adv_name, copy_len);
            scan_results[slot].name[copy_len] = '\0';
            scan_results[slot].name_len = copy_len;
        } else if (r->search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            scan_active = false;
            ESP_LOGI(GATTS_TABLE_TAG, "Scan complete: %d remote(s) found", scan_result_count);
        }
        break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s", esp_err_to_name(err));
        }
        break;

    // Security events
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTS_TABLE_TAG, "Authentication SUCCESS, addr_type: %d, auth_mode: %d",
                    param->ble_security.auth_cmpl.addr_type,
                    param->ble_security.auth_cmpl.auth_mode);
            is_authenticated = true;
        } else {
            ESP_LOGW(GATTS_TABLE_TAG, "Authentication FAILED, reason: 0x%x",
                    param->ble_security.auth_cmpl.fail_reason);
            is_authenticated = false;
            // Disconnect unauthenticated client
            esp_ble_gap_disconnect(param->ble_security.auth_cmpl.bd_addr);
        }
        break;

    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_KEY_EVT, key_type: %d",
                param->ble_security.ble_key.key_type);
        break;

    default:
        break;
    }
}

static void ota_reply(uint8_t cmd, uint8_t status)
{
    uint8_t rsp[6] = {cmd, status};
    put_u32(&rsp[2], ota_written);
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
        spp_handle_table[SPP_IDX_SPP_STATUS_VAL], sizeof(rsp), rsp, false);
}

static volatile bool ota_erasing = false;
static volatile uint32_t ota_epoch = 0;  // bumped by every cancel, incl. disconnect
static SemaphoreHandle_t slot_lock = NULL;  // the BEGIN erase vs the background one
static TaskHandle_t ota_prep_handle = NULL;
static bool slot_held = false;  // the Wi-Fi /api/ota upload owns the slot
static tinfl_decompressor *inf = NULL;
static uint8_t *inf_ring = NULL;
static uint32_t inf_ring_pos = 0;
static tinfl_status inf_status;
static int64_t ota_t0_us = 0;    // erase done
static int64_t ota_busy_us = 0;  // inside ota_inflate: inflate + flash writes
static uint32_t ota_bad = 0;     // out-of-order chunks: the remote lost or resent some

static void ota_cancel(void)
{
    ota_epoch++;
    if (ota_active) {
        ota_active = false;
        ESP_LOGW(GATTS_TABLE_TAG, "BLE OTA aborted at %" PRIu32 "/%" PRIu32, ota_written, ota_size);
        xTaskNotifyGive(ota_prep_handle);  // blank the half-written slot again
    }
}

static void ble_ota_reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));  // let the END reply leave the radio
    esp_restart();
}

/** True if [off, off + len) of `part` reads as erased. Caller holds slot_lock. */
static bool slot_blank(const esp_partition_t *part, uint32_t off, uint32_t len)
{
    static uint32_t buf[256];
    for (uint32_t pos = off; pos < off + len; pos += sizeof(buf)) {
        if (esp_partition_read(part, pos, buf, sizeof(buf)) != ESP_OK) {
            return false;
        }
        for (int i = 0; i < 256; i++) {
            if (buf[i] != UINT32_MAX) {
                return false;
            }
        }
    }
    return true;
}

// Erases what the background pass hasn't, in 64 KB blocks. Each block stalls
// the single core for ~150 ms, so it runs off the BTC task with the throttle
// timeout paused; the wheels are stopped.
static void ota_erase_task(void *arg)
{
    uint32_t epoch = (uint32_t)(uintptr_t)arg;
    throttle_stop_timeout_monitor();
    xSemaphoreTake(slot_lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    for (uint32_t off = 0; err == ESP_OK && epoch == ota_epoch && off < ota_raw; off += OTA_BLOCK) {
        if (!slot_blank(ota_part, off, OTA_BLOCK)) {
            err = esp_partition_erase_range(ota_part, off, OTA_BLOCK);
        }
    }
    xSemaphoreGive(slot_lock);
    ota_erasing = false;
    if (epoch != ota_epoch) {  // cancelled or disconnected while we erased
        if (is_connected) {
            throttle_start_timeout_monitor();
        }
        vTaskDelete(NULL);
    }
    ota_active = err == ESP_OK;
    ota_t0_us = esp_timer_get_time();
    ota_busy_us = 0;
    ota_bad = 0;
    throttle_start_timeout_monitor();
    ESP_LOGW(GATTS_TABLE_TAG, "BLE OTA %s: %" PRIu32 " -> %" PRIu32 " bytes into %s",
             err == ESP_OK ? "started" : "erase failed", ota_size, ota_raw, ota_part->label);
    ota_reply(BLE_CMD_OTA_BEGIN, err == ESP_OK ? OTA_ST_OK : OTA_ST_FLASH_ERR);
    vTaskDelete(NULL);
}

/** Inflate the next `n` stream bytes into the slot; `last` marks the end. */
static esp_err_t ota_inflate(const uint8_t *in, size_t n, bool last)
{
    uint32_t flags = TINFL_FLAG_PARSE_ZLIB_HEADER | (last ? 0 : TINFL_FLAG_HAS_MORE_INPUT);
    for (;;) {
        size_t in_n = n;
        size_t out_n = OTA_WINDOW - inf_ring_pos;
        inf_status = tinfl_decompress(inf, in, &in_n, inf_ring, inf_ring + inf_ring_pos,
                                      &out_n, flags);
        in += in_n;
        n -= in_n;
        if (out_n > 0) {
            if (ota_out + out_n > ota_raw ||
                esp_partition_write(ota_part, ota_out, inf_ring + inf_ring_pos, out_n) != ESP_OK) {
                return ESP_FAIL;
            }
            ota_out += out_n;
            inf_ring_pos = (inf_ring_pos + out_n) & (OTA_WINDOW - 1);
        }
        if (inf_status < TINFL_STATUS_DONE) {
            return ESP_FAIL;
        }
        if (inf_status != TINFL_STATUS_HAS_MORE_OUTPUT && n == 0) {
            return ESP_OK;
        }
        if (inf_status == TINFL_STATUS_DONE) {
            return ESP_FAIL;  // bytes past the end of the stream
        }
    }
}

static bool ota_prep_must_wait(void)
{
    esp_ota_img_states_t st;
    return fabsf(get_stored_vesc_values()->rpm) > OTA_MAX_STANDSTILL_ERPM ||
           (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK &&
            st == ESP_OTA_IMG_PENDING_VERIFY);  // the other slot is our rollback copy
}

// Blanks the spare slot after boot and after a failed update, so the next BLE
// update skips its multi-second erase. 4 KB at a time (~45 ms core freeze
// each), only at standstill; an update in progress ends the pass.
static void ota_prep_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
        bool busy = false;
        for (uint32_t off = 0; part != NULL && !busy && off < part->size; off += OTA_SECTOR) {
            while (ota_prep_must_wait()) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            xSemaphoreTake(slot_lock, portMAX_DELAY);
            busy = ota_erasing || ota_active || slot_held;
            if (!busy && !slot_blank(part, off, OTA_SECTOR)) {
                esp_partition_erase_range(part, off, OTA_SECTOR);
            }
            xSemaphoreGive(slot_lock);
            vTaskDelay(pdMS_TO_TICKS(10));  // let throttle and CAN traffic through
        }
    }
}

void ble_ota_slot_hold(bool hold)
{
    // Under the lock: an erase already under way finishes before we return.
    xSemaphoreTake(slot_lock, portMAX_DELAY);
    slot_held = hold;
    xSemaphoreGive(slot_lock);
    if (!hold) {
        xTaskNotifyGive(ota_prep_handle);  // blank whatever a failed upload left
    }
}

// Runs on the BTC task. The stream is zlib; each chunk is inflated straight
// into the pre-erased slot. The remote keeps several chunks in flight and
// resyncs on BAD_OFFSET.
static void ota_handle_cmd(const uint8_t *v, uint16_t len)
{
    uint8_t cmd = v[0];
    if (cmd == BLE_CMD_OTA_BEGIN && len >= 9) {
        if (ota_erasing) {
            ota_cancel();  // the erase task sees it and stops
            ota_reply(cmd, OTA_ST_REFUSED);
            return;
        }
        ota_cancel();
        ota_written = 0;
        if (fabsf(get_stored_vesc_values()->rpm) > OTA_MAX_STANDSTILL_ERPM) {
            ota_reply(cmd, OTA_ST_REFUSED);
            return;
        }
        ota_size = v[1] | (v[2] << 8) | (v[3] << 16) | ((uint32_t)v[4] << 24);
        ota_raw = v[5] | (v[6] << 8) | (v[7] << 16) | ((uint32_t)v[8] << 24);
        ota_part = esp_ota_get_next_update_partition(NULL);
        if (inf == NULL) {
            inf = malloc(sizeof(*inf));
            inf_ring = malloc(OTA_WINDOW);
        }
        if (ota_part == NULL || ota_size == 0 || ota_raw == 0 || ota_raw > ota_part->size ||
            inf == NULL || inf_ring == NULL) {
            ota_reply(cmd, OTA_ST_FLASH_ERR);
            return;
        }
        tinfl_init(inf);
        inf_status = TINFL_STATUS_NEEDS_MORE_INPUT;
        inf_ring_pos = 0;
        ota_out = 0;
        throttle_reset_value();
        ota_erasing = true;
        ota_reply(cmd, OTA_ST_ERASING);
        if (xTaskCreate(ota_erase_task, "ota_erase", 4096, (void *)(uintptr_t)ota_epoch, 4, NULL) != pdPASS) {
            ota_erasing = false;
            ota_reply(cmd, OTA_ST_FLASH_ERR);
        }
    } else if (cmd == BLE_CMD_OTA_DATA && len > 5) {
        if (!ota_active) {
            ota_reply(cmd, OTA_ST_REFUSED);
            return;
        }
        uint32_t off = v[1] | (v[2] << 8) | (v[3] << 16) | ((uint32_t)v[4] << 24);
        uint32_t n = len - 5;
        if (off != ota_written) {
            ota_bad++;
            ota_reply(cmd, OTA_ST_BAD_OFFSET);  // e.g. a resent chunk whose ack was lost
            return;
        }
        int64_t t = esp_timer_get_time();
        esp_err_t err = ota_written + n > ota_size
                            ? ESP_FAIL
                            : ota_inflate(&v[5], n, ota_written + n == ota_size);
        ota_busy_us += esp_timer_get_time() - t;
        if (err != ESP_OK) {
            ota_cancel();
            ota_reply(cmd, OTA_ST_FLASH_ERR);
            return;
        }
        ota_written += n;
        // Acks are cumulative, so one per 4 KB keeps the remote's window moving
        // with an eighth of the notifications.
        if (ota_written / OTA_ACK_EVERY != (ota_written - n) / OTA_ACK_EVERY ||
            ota_written == ota_size) {
            ota_reply(cmd, OTA_ST_OK);
        }
    } else if (cmd == BLE_CMD_OTA_END) {
        if (!ota_active || ota_written != ota_size || inf_status != TINFL_STATUS_DONE ||
            ota_out != ota_raw) {
            ota_cancel();
            ota_reply(cmd, OTA_ST_REFUSED);
            return;
        }
        ota_active = false;
        // Checks the image (magic, chip, SHA-256) before switching to it.
        esp_err_t err = esp_ota_set_boot_partition(ota_part);
        if (err != ESP_OK) {
            ESP_LOGE(GATTS_TABLE_TAG, "BLE OTA rejected: %s", esp_err_to_name(err));
            ota_reply(cmd, OTA_ST_FLASH_ERR);
            xTaskNotifyGive(ota_prep_handle);
            return;
        }
        ESP_LOGW(GATTS_TABLE_TAG, "BLE OTA complete in %" PRId64 " ms (%" PRId64 " ms inflating and writing flash, "
                 "%" PRIu32 " out-of-order chunks), rebooting",
                 (esp_timer_get_time() - ota_t0_us) / 1000, ota_busy_us / 1000, ota_bad);
        remote_shutdown_requested = true;  // the reboot's disconnect is not a lost link
        ota_reply(cmd, OTA_ST_OK);
        xTaskCreate(ble_ota_reboot_task, "ota_reboot", 2048, NULL, 1, NULL);
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    ESP_LOGD(GATTS_TABLE_TAG, "event = %x", event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	paired_load_nvs();
        	whitelist_sync();

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if (p_data->write.len < 2) {
                        break;
                    }
                    uint16_t descr_value = p_data->write.value[1]<<8 | p_data->write.value[0];
                    if (descr_value == 0x0001) {
                        enable_data_ntf = true;
                    } else if (descr_value == 0x0000) {
                        enable_data_ntf = false;
                    }
                }

                // SECURITY CHECK: Only accept commands from authenticated
                // connections. Remotes blast throttle frames at ~50 Hz from
                // the moment they connect, well before SMP completes — log
                // once per second so the boot/pair window isn't a wall of
                // warnings.
                if (!is_authenticated) {
                    static uint32_t last_log_ms = 0;
                    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                    if (now_ms - last_log_ms > 1000) {
                        ESP_LOGD(GATTS_TABLE_TAG, "Rejecting writes from unauthenticated client (waiting for SMP)");
                        last_log_ms = now_ms;
                    }
                    break;
                }

                if (res == SPP_IDX_SPP_DATA_RECV_VAL) {
                    if (ota_active || ota_erasing) {
                        // Mid-update: hold neutral whatever the lever says.
                        throttle_reset_value();
                        throttle_reset_timeout();
                        break;
                    }
                    if (p_data->write.len < 3) {
                        ESP_LOGW(GATTS_TABLE_TAG, "Throttle write too short: %d bytes", p_data->write.len);
                        break;
                    }
                    uint16_t adc_value = (uint16_t)p_data->write.value[0] |
                                               ((uint16_t)p_data->write.value[1] << 8);
                    // Throttle is interpreted as a 0..255 byte downstream — clamp at
                    // the trust boundary so a malformed remote can't wrap-around into
                    // an unintended full-throttle / full-brake command.
                    if (adc_value > 255) {
                        adc_value = 255;
                    }
                    uint8_t aux_output_state = p_data->write.value[2];
                    throttle_update_value(adc_value);
                    throttle_reset_timeout();
                    aux_output_set(aux_output_state);
                } else if (res == SPP_IDX_SPP_COMMAND_VAL) {
                    if (p_data->write.len < 1) {
                        break;
                    }
                    uint8_t cmd = p_data->write.value[0];
                    if (cmd == BLE_CMD_RESET_ODOMETER) {
                        ble_reset_trip_distance();
                        uint8_t ack[2] = {BLE_CMD_RESET_ODOMETER, 0x00};
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                            spp_handle_table[SPP_IDX_SPP_STATUS_VAL],
                            sizeof(ack), ack, false);
                        ESP_LOGI(GATTS_TABLE_TAG, "Odometer reset via BLE command");
                    } else if (cmd == BLE_CMD_GET_FW_VERSION) {
                        uint8_t rsp[10] = {BLE_CMD_GET_FW_VERSION};
                        sscanf(FW_VERSION, "%hhu.%hhu.%hhu", &rsp[1], &rsp[2], &rsp[3]);
                        esp_read_mac(&rsp[4], ESP_MAC_BT);
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                            spp_handle_table[SPP_IDX_SPP_STATUS_VAL],
                            sizeof(rsp), rsp, false);
                    } else if (cmd == BLE_CMD_SHUTDOWN) {
                        remote_shutdown_requested = true;
                        throttle_reset_value();
                        ESP_LOGI(GATTS_TABLE_TAG, "Remote powering off - suppressing failsafe on disconnect");
                    } else if (cmd == BLE_CMD_SET_SMART_REVERSE && p_data->write.len >= 2) {
                        throttle_set_smart_reverse(p_data->write.value[1] != 0);
                    } else if (cmd == BLE_CMD_SET_NO_REVERSE && p_data->write.len >= 2) {
                        throttle_set_no_reverse(p_data->write.value[1] != 0);
                    } else if (cmd == BLE_CMD_SET_ASSIST_PUSH && p_data->write.len >= 2) {
                        throttle_set_assist_push(p_data->write.value[1] != 0);
                        // Older remotes send only the flag; they keep the defaults.
                        if (p_data->write.len >= 4) {
                            throttle_set_assist_params(p_data->write.value[2],
                                                       p_data->write.value[3]);
                        }
                    } else if (cmd == BLE_CMD_SET_SPEED_LIMIT && p_data->write.len >= 2) {
                        throttle_set_speed_limit(p_data->write.value[1]);
                    } else if (cmd == BLE_CMD_SET_CURRENT_SCALE && p_data->write.len >= 2) {
                        throttle_set_current_scale(p_data->write.value[1]);
                    } else if (cmd >= BLE_CMD_OTA_BEGIN && cmd <= BLE_CMD_OTA_END) {
                        ota_handle_cmd(p_data->write.value, p_data->write.len);
                    }
                }
            }
      	 	break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    remote_shutdown_requested = false;  // Stale flag would wrongly suppress a future failsafe
    	    throttle_set_smart_reverse(false);  // until this remote sends its own setting
    	    throttle_set_no_reverse(false);     // until this remote sends its own setting
    	    throttle_set_assist_push(false);    // until this remote sends its own setting
    	    throttle_set_speed_limit(0);        // until this remote sends its own setting
    	    throttle_set_current_scale(100);    // until this remote sends its own setting
    	    throttle_reset_value();  // Reset to THROTTLE_NEUTRAL_VALUE on new connection
    	    throttle_start_timeout_monitor();
            bldc_interface_can_get_mcconf_temp(); // Fetch compact motor config for BLE telemetry
    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
    	    if (wifi_idle_timer != NULL) xTimerStop(wifi_idle_timer, 0);
    	    if (wifi_ap_is_running()) wifi_ap_stop();
    	    // Delete previous telemetry task if it leaked from a prior connection
    	    if (telemetry_task_handle != NULL) {
    	        vTaskDelete(telemetry_task_handle);
    	        telemetry_task_handle = NULL;
    	    }
    	    xTaskCreate(send_telemetry_task, "telemetry", 4096, NULL, 5, &telemetry_task_handle);
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
            spp_mtu_size = 23;
    	    ota_cancel();
    	    is_connected = false;
    	    is_authenticated = false;  // Reset authentication on disconnect
    	    throttle_reset_value();
    	    throttle_stop_timeout_monitor();
    	    if (remote_shutdown_requested) {
    	        remote_shutdown_requested = false;  // Intentional power-off, not a lost link
    	        ESP_LOGI(GATTS_TABLE_TAG, "Clean shutdown disconnect - failsafe skipped");
    	    } else {
    	        // Actively brake on link loss instead of coasting at neutral. The
    	        // throttle-timeout path also catches this in the supervision-timeout
    	        // window, but firing here closes the gap on a clean disconnect.
    	        failsafe_trigger(FAILSAFE_REASON_BLE_DISCONNECT);
    	    }
    	    enable_data_ntf = false;
    	    // Reset the odometer time reference so the disconnect gap is not
    	    // counted as riding distance when the next connection resumes.
    	    trip_last_update_ms = 0;
    	    if (adv_task_handle != NULL) {
    	        xTaskNotify(adv_task_handle, ADV_TASK_NOTIFY_REFRESH_ADV, eSetBits);
    	    }
    	    wifi_idle_timer_kick();
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(GATTS_TABLE_TAG, "EVT %d, gatts if %d", event, gatts_if);

    if (event == ESP_GATTS_REG_EVT && param->reg.status != ESP_GATT_OK) {
        ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",
                 param->reg.app_id, param->reg.status);
        return;
    }

    gatts_profile_event_handler(event, gatts_if, param);
}

esp_err_t ble_spp_server_init(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Release memory used by classic BT
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t ble_spp_server_start(void)
{
    esp_err_t ret;

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    xTaskCreate(adv_task, "ble_adv", 2048, NULL, 5, &adv_task_handle);
    slot_lock = xSemaphoreCreateMutex();
    xTaskCreate(ota_prep_task, "ota_prep", 3072, NULL, 1, &ota_prep_handle);
    xTaskNotifyGive(ota_prep_handle);
    wifi_idle_timer = xTimerCreate("wifi_idle",
                                   pdMS_TO_TICKS(WIFI_AP_IDLE_TIMEOUT_MS),
                                   pdFALSE, NULL, wifi_idle_timer_cb);
    wifi_idle_timer_kick();

    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;  // Secure Connections, MITM protection, Bonding
    esp_ble_io_cap_t io_cap = ESP_IO_CAP_OUT;  // Display only (server shows passkey)
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = BLE_PASSKEY;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;  // Only accept authenticated connections
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &io_cap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));

    ESP_LOGI(GATTS_TABLE_TAG, "BLE Security configured with passkey: %06lu", (unsigned long)passkey);

    trip_nvs_load();

    return ESP_OK;
}

bool ble_is_connected(void) {
    return is_connected;
}

void ble_refresh_advertising(void) {
    if (adv_task_handle != NULL) {
        xTaskNotify(adv_task_handle, ADV_TASK_NOTIFY_REFRESH_ADV, eSetBits);
    }
}

float ble_get_trip_km(void) {
    return trip_km;
}

bool ble_is_scanning(void) {
    return scan_active;
}

uint8_t ble_get_paired_count(void) {
    return paired_count;
}

uint8_t ble_get_paired_list(ble_paired_entry_t *out, uint8_t max_count) {
    uint8_t n = paired_count < max_count ? paired_count : max_count;
    memcpy(out, paired_list, n * sizeof(ble_paired_entry_t));
    return n;
}

bool ble_get_connected_mac(uint8_t out[6]) {
    if (!is_connected) return false;
    memcpy(out, spp_remote_bda, 6);
    return true;
}

bool ble_start_scan(void) {
    if (scan_active || is_connected) return false;
    scan_result_count = 0;
    memset(scan_results, 0, sizeof(scan_results));
    scan_active = true;
    static esp_ble_scan_params_t params = {
        .scan_type          = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval      = 0x50,
        .scan_window        = 0x30,
        .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
    };
    if (esp_ble_gap_set_scan_params(&params) != ESP_OK) {
        scan_active = false;
        return false;
    }
    return true;
}

uint8_t ble_get_scan_results(ble_scan_entry_t *out, uint8_t max_count) {
    uint8_t n = scan_result_count < max_count ? scan_result_count : max_count;
    memcpy(out, scan_results, n * sizeof(ble_scan_entry_t));
    return n;
}

bool ble_pair_remote(const uint8_t mac[6], uint8_t addr_type) {
    if (paired_find_index(mac) >= 0) return false;
    if (paired_count >= BLE_MAX_PAIRED_REMOTES) return false;
    memcpy(paired_list[paired_count].mac, mac, 6);
    paired_list[paired_count].addr_type = addr_type;
    paired_count++;
    paired_save_nvs();
    if (adv_task_handle != NULL) {
        xTaskNotify(adv_task_handle, ADV_TASK_NOTIFY_RELOAD_PAIRED | ADV_TASK_NOTIFY_REFRESH_ADV, eSetBits);
    }
    return true;
}

bool ble_unpair_remote_by_mac(const uint8_t mac[6]) {
    int idx = paired_find_index(mac);
    if (idx < 0) return false;
    esp_ble_remove_bond_device(paired_list[idx].mac);
    for (int i = idx; i < paired_count - 1; i++) {
        paired_list[i] = paired_list[i + 1];
    }
    paired_count--;
    memset(&paired_list[paired_count], 0, sizeof(ble_paired_entry_t));
    paired_save_nvs();
    if (is_connected && memcmp(spp_remote_bda, mac, 6) == 0) {
        esp_ble_gap_disconnect(spp_remote_bda);
    }
    if (adv_task_handle != NULL) {
        xTaskNotify(adv_task_handle, ADV_TASK_NOTIFY_RELOAD_PAIRED | ADV_TASK_NOTIFY_REFRESH_ADV, eSetBits);
    }
    return true;
}

void ble_reset_trip_distance(void) {
    trip_km = 0.0f;
    trip_km_nvs_committed = 0.0f;
    trip_last_update_ms = 0;
    trip_nvs_save();
    ESP_LOGI(GATTS_TABLE_TAG, "Trip distance reset");
}

static void trip_nvs_load(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TRIP_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        trip_km = 0.0f;
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "Error opening trip NVS: %s", esp_err_to_name(err));
        return;
    }
    float loaded = 0.0f;
    size_t len = sizeof(float);
    err = nvs_get_blob(nvs_handle, TRIP_NVS_KEY_KM, &loaded, &len);
    if (err == ESP_OK) {
        trip_km = loaded;
        ESP_LOGI(GATTS_TABLE_TAG, "Trip distance loaded: %d.%02d km", (int)trip_km, (int)(trip_km * 100) % 100);
    } else {
        trip_km = 0.0f;
    }
    nvs_close(nvs_handle);
}

static void trip_nvs_save(void) {
    nvs_handle_t nvs_handle;
    float trip_km_copy = trip_km;
    esp_err_t err = nvs_open(TRIP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "Error opening trip NVS for write: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(nvs_handle, TRIP_NVS_KEY_KM, &trip_km_copy, sizeof(float));
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(GATTS_TABLE_TAG, "Trip NVS commit failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Trip distance saved: %d.%02d km", (int)trip_km_copy, (int)(trip_km_copy * 100) % 100);
        }
    } else {
        ESP_LOGE(GATTS_TABLE_TAG, "Trip NVS set failed: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}

/* 11 bytes on the status characteristic:
 *   0 0x08   1 flags: smart_rev | no_rev<<1 | assist<<2 | failsafe<<3 | bms<<4
 *   2 vesc fault code   3 failsafe reason   4 speed limit   5 assist strength%
 *   6 assist decay      7-10 uptime s (u32)
 * The remote shows the settings as applied here and raises the faults. */
static void send_status_frame(void) {
    uint8_t set[4];
    throttle_get_settings(set);
    uint8_t buf[11] = {BLE_STATUS_FRAME};
    buf[1] = (set[0] & 7) | (failsafe_is_active() << 3) |
             ((get_stored_bms_values()->num_cells > 0) << 4);
    buf[2] = get_stored_vesc_values()->fault_code;
    buf[3] = failsafe_last_reason();
    memcpy(&buf[4], &set[1], 3);
    put_u32(&buf[7], (uint32_t)(esp_timer_get_time() / 1000000));
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
        spp_handle_table[SPP_IDX_SPP_STATUS_VAL], sizeof(buf), buf, false);
}

static void send_telemetry_task(void *pvParameters) {
    int tick = 0;
    while (1) {
        if (is_connected && tick++ % 10 == 0) {
            send_status_frame();
        }

        if (is_connected && enable_data_ntf) {
            mc_values* vesc_values = get_stored_vesc_values();
            bms_values_t* bms_values = get_stored_bms_values();

            // VESC (14) + BMS (41) + motor config (5) + aux state (1) + trip_km (4) = 65 bytes
            uint8_t buffer[65];
            int idx = 0;

            // VESC block, bytes 0-13
            idx += put_u16(&buffer[idx], (int16_t)(vesc_values->temp_mos * 100));
            idx += put_u16(&buffer[idx], (int16_t)(vesc_values->temp_motor * 100));
            idx += put_u16(&buffer[idx], (int16_t)(vesc_values->current_motor * 100));
            idx += put_u16(&buffer[idx], (int16_t)(vesc_values->current_in * 100));
            idx += put_u32(&buffer[idx], (uint32_t)(int32_t)vesc_values->rpm);
            idx += put_u16(&buffer[idx], (int16_t)(vesc_values->v_in * 100));

            // BMS block, bytes 14-54
            idx += put_u16(&buffer[idx], (int16_t)(bms_values->total_voltage * 100));
            idx += put_u16(&buffer[idx], (int16_t)(bms_values->current * 100));
            idx += put_u16(&buffer[idx], (int16_t)(bms_values->remaining_capacity * 100));
            idx += put_u16(&buffer[idx], (int16_t)(bms_values->nominal_capacity * 100));
            buffer[idx++] = bms_values->num_cells;

            // 16 cell voltages (mV), zero-padded past num_cells. The remote no
            // longer displays these, but they hold the frame layout — later
            // fields are read by fixed offset.
            for (int i = 0; i < 16; i++) {
                int16_t cell_mv = (i < bms_values->num_cells)
                                      ? (int16_t)(bms_values->cell_voltages[i] * 1000)
                                      : 0;
                idx += put_u16(&buffer[idx], cell_mv);
            }

            // Pack compact mcconf temp data (motor poles, gear ratio, wheel diameter)
            mc_temp_config_t* mc_temp_conf = get_stored_mc_temp_config();
            uint8_t motor_poles = 0;
            uint16_t gear_ratio_scaled = 0;
            uint16_t wheel_diameter_scaled = 0;

            if (mc_temp_conf != NULL && mc_temp_conf->valid) {
                motor_poles = mc_temp_conf->motor_poles;
                gear_ratio_scaled = (uint16_t)((mc_temp_conf->gear_ratio * 1000.0f) + 0.5f);
                wheel_diameter_scaled = (uint16_t)((mc_temp_conf->wheel_diameter * 1000.0f) + 0.5f);
            }

            buffer[idx++] = motor_poles;
            idx += put_u16(&buffer[idx], gear_ratio_scaled);
            idx += put_u16(&buffer[idx], wheel_diameter_scaled);

            // Pack aux output state (byte 60)
            buffer[idx++] = aux_output_get_state();

            // Accumulate trip distance from ERPM and motor config (bytes 61-64)
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (trip_last_update_ms > 0 && mc_temp_conf != NULL && mc_temp_conf->valid &&
                mc_temp_conf->motor_poles > 0 && mc_temp_conf->gear_ratio > 0.0f &&
                mc_temp_conf->wheel_diameter > 0.0f) {
                float elapsed_hours = (now_ms - trip_last_update_ms) / 3600000.0f;
                float pole_pairs = (float)mc_temp_conf->motor_poles / 2.0f;
                float shaft_rpm = (float)vesc_values->rpm / pole_pairs;
                float wheel_circumference_m = mc_temp_conf->wheel_diameter * (float)M_PI;
                float wheel_rpm = shaft_rpm / mc_temp_conf->gear_ratio;
                float speed_kmh = wheel_rpm * wheel_circumference_m * 60.0f / 1000.0f;
                if (speed_kmh < 0.0f) speed_kmh = -speed_kmh;
                trip_km += speed_kmh * elapsed_hours;
            }
            trip_last_update_ms = now_ms;

            // Commit to NVS every 100 m to minimise flash wear
            if (trip_km - trip_km_nvs_committed >= TRIP_NVS_SAVE_INTERVAL_KM) {
                trip_nvs_save();
                trip_km_nvs_committed = trip_km;
            }

            idx += put_u32(&buffer[idx], (uint32_t)(int32_t)(trip_km * 100.0f));

            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
                sizeof(buffer), buffer, false);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Update every 100ms
    }
}

