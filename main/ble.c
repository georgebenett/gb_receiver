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

// Paired remote management (receiver prefers its last-connected remote on boot)
static esp_bd_addr_t paired_remote_mac = {0};
static bool has_paired_remote = false;
static TimerHandle_t prefer_paired_timer = NULL;
static TaskHandle_t prefer_paired_task_handle = NULL;

typedef enum {
    BLE_WINDOW_PAIRED = 0,  // Window 1: whitelist (last saved MAC)
    BLE_WINDOW_OPEN,        // Window 2: open to any remote
    BLE_WINDOW_CLOSED,      // Window 3: advertising stopped, waiting for reboot
} ble_window_state_t;

static ble_window_state_t window_state = BLE_WINDOW_PAIRED;

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

static void ble_paired_remote_load_mac(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(BLE_PAIRED_REMOTE_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        uint8_t valid = 0;
        if (nvs_get_u8(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_VALID, &valid) == ESP_OK && valid == 1) {
            size_t mac_len = sizeof(esp_bd_addr_t);
            if (nvs_get_blob(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_MAC, paired_remote_mac, &mac_len) == ESP_OK) {
                has_paired_remote = true;
                ESP_LOGI(GATTS_TABLE_TAG, "Loaded paired remote MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                         paired_remote_mac[0], paired_remote_mac[1], paired_remote_mac[2],
                         paired_remote_mac[3], paired_remote_mac[4], paired_remote_mac[5]);
            }
        }
        nvs_close(nvs_handle);
    }
}

static void ble_paired_remote_save_mac(esp_bd_addr_t mac)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(BLE_PAIRED_REMOTE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "Failed to open NVS for paired remote MAC: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_MAC, mac, sizeof(esp_bd_addr_t));
    if (err == ESP_OK) {
        uint8_t valid = 1;
        err = nvs_set_u8(nvs_handle, BLE_PAIRED_REMOTE_NVS_KEY_VALID, valid);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    if (err == ESP_OK) {
        memcpy(paired_remote_mac, mac, sizeof(esp_bd_addr_t));
        has_paired_remote = true;
        ESP_LOGI(GATTS_TABLE_TAG, "Saved paired remote MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    nvs_close(nvs_handle);
}

// --- Advertising window state machine ---

static void prefer_paired_timer_cb(TimerHandle_t xTimer)
{
    if (prefer_paired_task_handle != NULL) {
        xTaskNotifyGive(prefer_paired_task_handle);
    }
}

static void prefer_paired_task(void *pvParameters)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (is_connected) continue;

        if (window_state == BLE_WINDOW_PAIRED) {
            // Transition to window 2: open to any remote for 10s
            window_state = BLE_WINDOW_OPEN;
            spp_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
            esp_ble_gap_stop_advertising();
            esp_ble_gap_start_advertising(&spp_adv_params);
            xTimerChangePeriod(prefer_paired_timer, pdMS_TO_TICKS(BLE_WINDOW_OPEN_MS), 0);
            ESP_LOGI(GATTS_TABLE_TAG, "Window 2 (10s): open to all remotes");
        } else if (window_state == BLE_WINDOW_OPEN) {
            // Transition to window 3: stop advertising until reboot
            window_state = BLE_WINDOW_CLOSED;
            esp_ble_gap_stop_advertising();
            ESP_LOGI(GATTS_TABLE_TAG, "Window 3: pairing closed, waiting for reboot");
        }
    }
}

// Start or restart the 3-window advertising sequence.
// Sets adv_filter_policy in spp_adv_params — call before esp_ble_gap_start_advertising.
static void start_adv_window_sequence(void)
{
    if (prefer_paired_timer == NULL) {
        prefer_paired_timer = xTimerCreate("adv_window", pdMS_TO_TICKS(BLE_WINDOW_PAIRED_MS),
                                           pdFALSE, NULL, prefer_paired_timer_cb);
    }
    if (has_paired_remote) {
        window_state = BLE_WINDOW_PAIRED;
        spp_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST;
        xTimerChangePeriod(prefer_paired_timer, pdMS_TO_TICKS(BLE_WINDOW_PAIRED_MS), 0);
        ESP_LOGI(GATTS_TABLE_TAG, "Window 1 (10s): paired MAC only");
    } else {
        window_state = BLE_WINDOW_OPEN;
        spp_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
        xTimerChangePeriod(prefer_paired_timer, pdMS_TO_TICKS(BLE_WINDOW_OPEN_MS), 0);
        ESP_LOGI(GATTS_TABLE_TAG, "Window 2 (10s): open to all (no paired remote)");
    }
}

static void stop_adv_window(void)
{
    if (prefer_paired_timer != NULL) {
        xTimerStop(prefer_paired_timer, 0);
    }
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGI(GATTS_TABLE_TAG, "GAP_EVT, event %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        start_adv_window_sequence();  // Sets filter policy before advertising starts
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
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

        	// Load paired remote MAC; whitelist updated here so it's ready before advertising
        	ble_paired_remote_load_mac();
        	if (has_paired_remote) {
        	    esp_ble_gap_update_whitelist(true, paired_remote_mac, BLE_WL_ADDR_TYPE_PUBLIC);
        	}

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

                // SECURITY CHECK: Only accept commands from authenticated connections
                if (!is_authenticated) {
                    ESP_LOGW(GATTS_TABLE_TAG, "Rejecting write from unauthenticated client!");
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
    	    // Connection established — freeze the advertising window state
    	    stop_adv_window();
    	    // Save remote MAC if new or different
    	    if (!has_paired_remote ||
    	        memcmp(paired_remote_mac, p_data->connect.remote_bda, sizeof(esp_bd_addr_t)) != 0) {
    	        ble_paired_remote_save_mac(p_data->connect.remote_bda);
    	    }
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
    	    // Restart 3-window sequence on disconnect
    	    start_adv_window_sequence();  // Resets state and sets filter policy
    	    esp_ble_gap_start_advertising(&spp_adv_params);
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

    // Create task to handle advertising window transitions (avoids BLE API calls from timer context)
    xTaskCreate(prefer_paired_task, "adv_window", 2048, NULL, 5, &prefer_paired_task_handle);

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

