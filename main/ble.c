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
#include "bms.h"
#include "datatypes.h"
#include "bldc_interface_can.h"
#include "aux_output.h"
#include "wifi_ap.h"

extern mc_values* get_stored_vesc_values(void);
extern bms_values_t* get_stored_bms_values(void);
extern mc_temp_config_t* get_stored_mc_temp_config(void);

#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"
#define CLIENT_NAME      "GS-THUMB"

#define TRIP_NVS_NAMESPACE   "trip_data"
#define TRIP_NVS_KEY_KM      "trip_km"
#define TRIP_NVS_SAVE_INTERVAL_KM 0.1f  // save every 100 meters

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          CLIENT_NAME
#define SPP_SVC_INST_ID             0

// BLE Security Configuration
#define BLE_PASSKEY                 483265  // Fixed passkey for pairing
#define BLE_CMD_RESET_ODOMETER      0x01   // Command: reset trip odometer


static const uint16_t spp_service_uuid = 0xABF0;
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

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

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

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
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s", esp_err_to_name(err));
        }
        break;

    // Security events
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
        // Accept the security request from the client
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
                // connections. Remotes blast throttle frames at ~13 Hz from
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
    	    is_connected = false;
    	    is_authenticated = false;  // Reset authentication on disconnect
    	    throttle_reset_value();
    	    throttle_stop_timeout_monitor();
    	    // Actively brake on link loss instead of coasting at neutral. The
    	    // throttle-timeout path also catches this in the supervision-timeout
    	    // window, but firing here closes the gap on a clean disconnect.
    	    failsafe_trigger(FAILSAFE_REASON_BLE_DISCONNECT);
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

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

esp_err_t ble_spp_server_init(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Release memory used by classic BT
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize BT controller
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable BT controller in BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluedroid stack
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

    // Register callbacks
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    xTaskCreate(adv_task, "ble_adv", 2048, NULL, 5, &adv_task_handle);
    wifi_idle_timer = xTimerCreate("wifi_idle",
                                   pdMS_TO_TICKS(WIFI_AP_IDLE_TIMEOUT_MS),
                                   pdFALSE, NULL, wifi_idle_timer_cb);
    wifi_idle_timer_kick();

    // Set local MTU
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure BLE Security
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

static void send_telemetry_task(void *pvParameters) {
    while (1) {
        if (is_connected && enable_data_ntf) {
            mc_values* vesc_values = get_stored_vesc_values();
            bms_values_t* bms_values = get_stored_bms_values();

            // VESC (14) + BMS (41) + motor config (5) + aux state (1) + trip_km (4) = 65 bytes
            uint8_t buffer[65];
            int idx = 0;

            // Pack VESC data
            int16_t temp_mos = (int16_t)(vesc_values->temp_mos * 100);
            int16_t temp_motor = (int16_t)(vesc_values->temp_motor * 100);
            int16_t current_motor = (int16_t)(vesc_values->current_motor * 100);
            int16_t current_in = (int16_t)(vesc_values->current_in * 100);
            int32_t rpm = (int32_t)vesc_values->rpm;
            int16_t voltage = (int16_t)(vesc_values->v_in * 100);

            // Pack temp_mos (little-endian)
            buffer[idx++] = temp_mos & 0xFF;
            buffer[idx++] = (temp_mos >> 8) & 0xFF;

            // Pack temp_motor (little-endian)
            buffer[idx++] = temp_motor & 0xFF;
            buffer[idx++] = (temp_motor >> 8) & 0xFF;

            // Pack current_motor (little-endian)
            buffer[idx++] = current_motor & 0xFF;
            buffer[idx++] = (current_motor >> 8) & 0xFF;

            // Pack current_in (little-endian)
            buffer[idx++] = current_in & 0xFF;
            buffer[idx++] = (current_in >> 8) & 0xFF;

            // Pack RPM (little-endian)
            buffer[idx++] = rpm & 0xFF;
            buffer[idx++] = (rpm >> 8) & 0xFF;
            buffer[idx++] = (rpm >> 16) & 0xFF;
            buffer[idx++] = (rpm >> 24) & 0xFF;

            // Pack voltage (little-endian)
            buffer[idx++] = voltage & 0xFF;
            buffer[idx++] = (voltage >> 8) & 0xFF;

            // Pack BMS data
            int16_t bms_total_voltage = (int16_t)(bms_values->total_voltage * 100);
            int16_t bms_current = (int16_t)(bms_values->current * 100);
            int16_t remaining_capacity = (int16_t)(bms_values->remaining_capacity * 100);
            int16_t nominal_capacity = (int16_t)(bms_values->nominal_capacity * 100);

            // Pack BMS values (little-endian)
            buffer[idx++] = bms_total_voltage & 0xFF;
            buffer[idx++] = (bms_total_voltage >> 8) & 0xFF;
            buffer[idx++] = bms_current & 0xFF;
            buffer[idx++] = (bms_current >> 8) & 0xFF;
            buffer[idx++] = remaining_capacity & 0xFF;
            buffer[idx++] = (remaining_capacity >> 8) & 0xFF;
            buffer[idx++] = nominal_capacity & 0xFF;
            buffer[idx++] = (nominal_capacity >> 8) & 0xFF;

            // Pack number of cells
            buffer[idx++] = bms_values->num_cells;

            // Pack cell voltages (little-endian)
            for (int i = 0; i < 16; i++) {
                int16_t cell_voltage;
                if (i < bms_values->num_cells) {
                    cell_voltage = (int16_t)(bms_values->cell_voltages[i] * 1000);
                } else {
                    cell_voltage = 0;
                }
                buffer[idx++] = cell_voltage & 0xFF;
                buffer[idx++] = (cell_voltage >> 8) & 0xFF;
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
            buffer[idx++] = gear_ratio_scaled & 0xFF;
            buffer[idx++] = (gear_ratio_scaled >> 8) & 0xFF;
            buffer[idx++] = wheel_diameter_scaled & 0xFF;
            buffer[idx++] = (wheel_diameter_scaled >> 8) & 0xFF;

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

            int32_t trip_km_x100 = (int32_t)(trip_km * 100.0f);
            buffer[idx++] = trip_km_x100 & 0xFF;
            buffer[idx++] = (trip_km_x100 >> 8) & 0xFF;
            buffer[idx++] = (trip_km_x100 >> 16) & 0xFF;
            buffer[idx++] = (trip_km_x100 >> 24) & 0xFF;

            // Send notification
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
                sizeof(buffer), buffer, false);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Update every 100ms
    }
}

