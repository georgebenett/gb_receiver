/*
 * bldc_interface_can.c
 *
 * CAN interface implementation for VESC communication
 * Uses ESP32 TWAI (CAN) driver
 */

#include "bldc_interface_can.h"
#include "buffer.h"
#include "crc.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "BLDC_CAN";

// Configuration
#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_500KBITS()  // 500 kbps
#define MAX_DETECTED_VESCS 4  // Maximum number of VESCs to track
#define VESC_DETECTION_TIMEOUT_MS 5000  // Consider VESC inactive after 5s without STATUS messages

// Transmit retry configuration
#define CAN_TX_MAX_RETRIES      3   // Attempts per transmit call
#define CAN_TX_RETRY_DELAY_MS   2   // Delay between retries
#define CAN_TX_FAULT_THRESHOLD  10  // Consecutive failures before fault is declared

// VESC auto-detection
typedef struct {
    uint8_t id;
    uint32_t last_seen_ms;
    bool active;
    int32_t erpm;               // Last known ERPM from STATUS packet
    uint32_t erpm_timestamp_ms; // When erpm was last updated
} detected_vesc_t;

static detected_vesc_t detected_vescs[MAX_DETECTED_VESCS] = {0};
static uint8_t primary_vesc_id = 0;
static bool primary_vesc_detected = false;
static uint8_t num_detected_vescs = 0;

// Callbacks
static void(*rx_value_func)(mc_values *values) = NULL;
static void(*rx_mcconf_temp_func)(mc_temp_config_t *conf) = NULL;
static void(*vesc_dropout_func)(uint8_t id) = NULL;

static mc_temp_config_t mc_temp_config;

// Forward declarations
static esp_err_t can_transmit_eid(uint32_t eid, const uint8_t *data, uint8_t len);
static void process_status_message(uint8_t id, uint8_t *data, uint8_t len);
static void process_status_2_message(uint8_t id, uint8_t *data, uint8_t len);
static void process_status_3_message(uint8_t id, uint8_t *data, uint8_t len);
static void process_status_4_message(uint8_t id, uint8_t *data, uint8_t len);
static void process_status_5_message(uint8_t id, uint8_t *data, uint8_t len);
static void update_and_notify_values(void);
static void detect_vesc_id(uint8_t id);
static uint8_t get_primary_vesc_id(void);
static void send_command_to_id(uint8_t vesc_id, uint8_t *data, uint16_t len);

// Maximum plausible ERPM for any supported vehicle.
// During VESC motor detection the VESC broadcasts STATUS packets with
// electrical frequencies that translate to millions of ERPM.  Any reading
// beyond this threshold is noise from detection and must be discarded.
#define MAX_VALID_ERPM 200000

// CAN transmit fault tracking
static uint32_t can_consecutive_failures = 0;
static bool can_fault_active = false;

// Accumulated status values from multiple STATUS messages
static mc_values accumulated_values = {0};
static bool status_received = false;
static bool status_2_received = false;
static bool status_3_received = false;
static bool status_4_received = false;
static bool status_5_received = false;

// Multi-frame response buffers — one per VESC to handle concurrent responses
#define RX_BUFFER_SIZE 512
#define RX_BUFFER_TIMEOUT_MS 1000
#define MAX_RX_SESSIONS 4

typedef struct {
    uint8_t  data[RX_BUFFER_SIZE];
    uint16_t len;
    uint8_t  vesc_id;
    uint32_t timeout_ms;
    bool     active;
} vesc_rx_buf_t;

static vesc_rx_buf_t vesc_rx_bufs[MAX_RX_SESSIONS];

static vesc_rx_buf_t *rx_buf_for_id(uint8_t id, bool create) {
    // Find existing slot for this VESC
    for (int i = 0; i < MAX_RX_SESSIONS; i++) {
        if (vesc_rx_bufs[i].active && vesc_rx_bufs[i].vesc_id == id) {
            return &vesc_rx_bufs[i];
        }
    }
    if (!create) {
        return NULL;
    }
    // Allocate a free slot
    for (int i = 0; i < MAX_RX_SESSIONS; i++) {
        if (!vesc_rx_bufs[i].active) {
            memset(&vesc_rx_bufs[i], 0, sizeof(vesc_rx_buf_t));
            vesc_rx_bufs[i].active  = true;
            vesc_rx_bufs[i].vesc_id = id;
            return &vesc_rx_bufs[i];
        }
    }
    // All slots busy — evict the oldest (first active slot)
    ESP_LOGW(TAG, "RX session table full, evicting slot for ID=%d", vesc_rx_bufs[0].vesc_id);
    memset(&vesc_rx_bufs[0], 0, sizeof(vesc_rx_buf_t));
    vesc_rx_bufs[0].active  = true;
    vesc_rx_bufs[0].vesc_id = id;
    return &vesc_rx_bufs[0];
}

void bldc_interface_can_init(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    // Configure TWAI (CAN) driver
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        tx_pin, rx_pin, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 5;
    g_config.rx_queue_len = 10;
    // Note: ESP_INTR_FLAG_IRAM removed - TWAI driver functions are not in IRAM

    twai_timing_config_t t_config = CAN_BAUD_RATE;

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return;
    }

    // Start TWAI driver
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        return;
    }

    // Reset detection state
    primary_vesc_id = 0;
    primary_vesc_detected = false;
    num_detected_vescs = 0;
    memset(detected_vescs, 0, sizeof(detected_vescs));

    // Reset fault state
    can_consecutive_failures = 0;
    can_fault_active = false;

    // Reset RX buffer state
    memset(vesc_rx_bufs, 0, sizeof(vesc_rx_bufs));

    ESP_LOGI(TAG, "CAN interface initialized - auto-detecting VESC ID(s)");
}

void bldc_interface_can_deinit(void) {
    twai_stop();
    twai_driver_uninstall();
}

static esp_err_t can_transmit_eid(uint32_t eid, const uint8_t *data, uint8_t len) {
    if (len > 8) {
        len = 8;
    }

    twai_message_t message;
    message.identifier = eid;
    message.flags = TWAI_MSG_FLAG_EXTD;
    message.data_length_code = len;
    memcpy(message.data, data, len);

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < CAN_TX_MAX_RETRIES; attempt++) {
        ret = twai_transmit(&message, pdMS_TO_TICKS(10));
        if (ret == ESP_OK) {
            if (can_consecutive_failures > 0) {
                ESP_LOGI(TAG, "CAN transmit recovered after %" PRIu32 " failure(s)", can_consecutive_failures);
                can_consecutive_failures = 0;
                can_fault_active = false;
            }
            return ESP_OK;
        }
        if (ret == ESP_ERR_INVALID_STATE) {
            // Driver is not running — recover and abort this transmit.
            // Do NOT retry immediately; let the caller's polling cycle retry.
            twai_status_info_t status;
            if (twai_get_status_info(&status) == ESP_OK) {
                if (status.state == TWAI_STATE_BUS_OFF) {
                    ESP_LOGW(TAG, "CAN bus-off (TEC=%" PRIu32 "), initiating recovery",
                             status.tx_error_counter);
                    twai_initiate_recovery();
                    // Poll until recovery completes (RECOVERING -> STOPPED), max 500 ms
                    for (int r = 0; r < 50; r++) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                        twai_status_info_t s2;
                        if (twai_get_status_info(&s2) == ESP_OK &&
                            s2.state == TWAI_STATE_STOPPED) {
                            break;
                        }
                    }
                    esp_err_t start_ret = twai_start();
                    if (start_ret != ESP_OK) {
                        ESP_LOGE(TAG, "CAN restart failed: %s", esp_err_to_name(start_ret));
                    }
                } else if (status.state == TWAI_STATE_STOPPED) {
                    ESP_LOGW(TAG, "CAN driver stopped, restarting");
                    twai_start();
                }
            }
            break;  // Don't retry TX — let the next polling cycle do it
        }
        if (attempt < CAN_TX_MAX_RETRIES - 1) {
            vTaskDelay(pdMS_TO_TICKS(CAN_TX_RETRY_DELAY_MS));
        }
    }

    can_consecutive_failures++;
    if (can_consecutive_failures >= CAN_TX_FAULT_THRESHOLD && !can_fault_active) {
        can_fault_active = true;
        ESP_LOGE(TAG, "CAN bus fault: %" PRIu32 " consecutive transmit failures, last error: %s",
                 can_consecutive_failures, esp_err_to_name(ret));
    } else {
        ESP_LOGW(TAG, "CAN transmit failed after %d retries: %s", CAN_TX_MAX_RETRIES, esp_err_to_name(ret));
    }

    return ret;
}

// Get primary VESC ID (first detected, or 0 if none detected)
static uint8_t get_primary_vesc_id(void) {
    if (!primary_vesc_detected) {
        // Check if any VESC is detected
        for (int i = 0; i < num_detected_vescs; i++) {
            if (detected_vescs[i].active) {
                primary_vesc_id = detected_vescs[i].id;
                primary_vesc_detected = true;
                break;
            }
        }
    }
    return primary_vesc_id;
}

// Public function to get primary VESC ID
uint8_t bldc_interface_can_get_primary_vesc_id(void) {
    return get_primary_vesc_id();
}

// Public function to get number of detected VESCs
uint8_t bldc_interface_can_get_detected_vesc_count(void) {
    uint8_t count = 0;
    for (int i = 0; i < num_detected_vescs; i++) {
        if (detected_vescs[i].active) {
            count++;
        }
    }
    return count;
}

// Detect and track VESC ID from received STATUS messages
static void detect_vesc_id(uint8_t id) {
    // Ignore broadcast ID (255)
    if (id == 255) {
        return;
    }

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Check if we already know this VESC
    for (int i = 0; i < num_detected_vescs; i++) {
        if (detected_vescs[i].id == id) {
            detected_vescs[i].last_seen_ms = now_ms;
            detected_vescs[i].active = true;
            return;
        }
    }

    // New VESC detected - add it
    if (num_detected_vescs < MAX_DETECTED_VESCS) {
        detected_vescs[num_detected_vescs].id = id;
        detected_vescs[num_detected_vescs].last_seen_ms = now_ms;
        detected_vescs[num_detected_vescs].active = true;

        // Set as primary if it's the first one
        if (!primary_vesc_detected) {
            primary_vesc_id = id;
            primary_vesc_detected = true;
            ESP_LOGI(TAG, "Primary VESC detected: ID=%d", id);
        } else {
            ESP_LOGI(TAG, "Additional VESC detected: ID=%d (primary: %d)", id, primary_vesc_id);
        }

        num_detected_vescs++;
    } else {
        ESP_LOGW(TAG, "Maximum VESC count reached, ignoring ID=%d", id);
    }
}

// Simple commands (single frame)
void bldc_interface_can_set_duty(float duty) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_current(float current) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_current_brake(float current) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_rpm(float rpm) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_int32(buffer, (int32_t)rpm, &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_RPM << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_pos(float pos) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_POS << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_current_rel(float current_rel) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_current_brake_rel(float current_rel) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_handbrake(float current) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_float32(buffer, current, 1e3, &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8);
    can_transmit_eid(eid, buffer, 4);
}

void bldc_interface_can_set_handbrake_rel(float current_rel) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        return;
    }

    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, &index);

    uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8);
    can_transmit_eid(eid, buffer, 4);
}

// Internal: send a command to a specific VESC ID (single or multi-frame as needed)
static void send_command_to_id(uint8_t vesc_id, uint8_t *data, uint16_t len) {
    uint8_t send_buffer[8];

    if (len <= 6) {
        uint32_t ind = 0;
        send_buffer[ind++] = vesc_id;  // Sender ID
        send_buffer[ind++] = 0;        // send = 0 (process packet)
        memcpy(send_buffer + ind, data, len);
        ind += len;

        uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8);
        can_transmit_eid(eid, send_buffer, ind);
    } else {
        uint16_t offset = 0;
        for (uint16_t i = 0; i < len && i < 255; i += 7) {
            uint8_t frag_len = (len - i < 7) ? (len - i) : 7;
            send_buffer[0] = (uint8_t)(i & 0xFF);
            memcpy(send_buffer + 1, data + i, frag_len);
            uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8);
            can_transmit_eid(eid, send_buffer, frag_len + 1);
            offset += frag_len;
        }

        for (uint16_t i = offset; i < len; i += 6) {
            uint8_t frag_len = (len - i < 6) ? (len - i) : 6;
            send_buffer[0] = (i >> 8) & 0xFF;
            send_buffer[1] = i & 0xFF;
            memcpy(send_buffer + 2, data + i, frag_len);
            uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8);
            can_transmit_eid(eid, send_buffer, frag_len + 2);
        }

        uint32_t ind = 0;
        send_buffer[ind++] = vesc_id;
        send_buffer[ind++] = 0;
        send_buffer[ind++] = (len >> 8) & 0xFF;
        send_buffer[ind++] = len & 0xFF;
        uint16_t crc = crc16(data, len);
        send_buffer[ind++] = (crc >> 8) & 0xFF;
        send_buffer[ind++] = crc & 0xFF;
        uint32_t eid = vesc_id | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8);
        can_transmit_eid(eid, send_buffer, ind);
    }
}

// Multi-frame command transmission — primary VESC only
void bldc_interface_can_send_command(uint8_t *data, uint16_t len) {
    if (!primary_vesc_detected) {
        return;
    }
    send_command_to_id(get_primary_vesc_id(), data, len);
}

// Send command to every active VESC independently — never relies on CAN forwarding
void bldc_interface_can_send_to_all(uint8_t *data, uint16_t len) {
    for (int i = 0; i < num_detected_vescs; i++) {
        if (detected_vescs[i].active) {
            send_command_to_id(detected_vescs[i].id, data, len);
        }
    }
}

void bldc_interface_can_send_to_id(uint8_t vesc_id, uint8_t *data, uint16_t len) {
    send_command_to_id(vesc_id, data, len);
}

// Apply regenerative braking to every active VESC — used by failsafe path
void bldc_interface_can_set_current_brake_rel_all(float current_rel) {
    uint8_t buffer[4];
    int32_t index = 0;
    buffer_append_float32(buffer, current_rel, 1e5, &index);

    for (int i = 0; i < num_detected_vescs; i++) {
        if (detected_vescs[i].active) {
            uint32_t eid = detected_vescs[i].id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8);
            can_transmit_eid(eid, buffer, 4);
        }
    }
}

// Return the worst-case |ERPM| across all active VESCs — used to gate braking intensity
int32_t bldc_interface_can_get_max_abs_erpm(void) {
    int32_t max_erpm = 0;
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < num_detected_vescs; i++) {
        if (!detected_vescs[i].active) continue;
        // Treat stale readings as unknown — use last known value conservatively
        if (now_ms - detected_vescs[i].erpm_timestamp_ms > VESC_DETECTION_TIMEOUT_MS) continue;
        int32_t abs_erpm = detected_vescs[i].erpm < 0 ? -detected_vescs[i].erpm : detected_vescs[i].erpm;
        if (abs_erpm > max_erpm) {
            max_erpm = abs_erpm;
        }
    }
    return max_erpm;
}

void bldc_interface_can_set_vesc_dropout_func(void(*func)(uint8_t id)) {
    vesc_dropout_func = func;
}

int32_t bldc_interface_can_get_erpm_at(uint8_t index) {
    if (index >= num_detected_vescs) return 0;
    return detected_vescs[index].erpm;
}

uint8_t bldc_interface_can_get_id_at(uint8_t index) {
    if (index >= num_detected_vescs) return 0;
    return detected_vescs[index].id;
}

// Wrapper functions for complex commands
void bldc_interface_can_get_values(void) {
    uint8_t cmd = COMM_GET_VALUES;
    // Request values silently (no logging needed for normal operation)
    bldc_interface_can_send_command(&cmd, 1);
}

void bldc_interface_can_get_mcconf_temp(void) {
    uint8_t vesc_id = get_primary_vesc_id();
    if (!primary_vesc_detected) {
        ESP_LOGW(TAG, "Cannot request MC config: no VESC detected yet");
        return;
    }
    uint8_t cmd = COMM_GET_MCCONF_TEMP;
    ESP_LOGI(TAG, "Requesting MC config from VESC ID=%d", vesc_id);
    bldc_interface_can_send_command(&cmd, 1);
}

void bldc_interface_can_get_mcconf(void) {
    uint8_t cmd = COMM_GET_MCCONF;
    bldc_interface_can_send_command(&cmd, 1);
}

void bldc_interface_can_get_appconf(void) {
    uint8_t cmd = COMM_GET_APPCONF;
    bldc_interface_can_send_command(&cmd, 1);
}

void bldc_interface_can_send_packet(unsigned char *data, unsigned int len) {
    bldc_interface_can_send_command((uint8_t *)data, (uint16_t)len);
}

static void process_response_packet(unsigned char *data, unsigned int len) {
    if (!len) return;

    int32_t ind = 0;
    unsigned char id = data[0];
    data++;
    len--;

    switch (id) {
    case COMM_GET_VALUES: {
        // Minimum bytes to reach v_in: temp_mos(2)+temp_motor(2)+current_motor(4)+
        // current_in(4)+id(4)+iq(4)+duty_now(2)+rpm(4)+v_in(2) = 28
        if (len < 28) break;
        int32_t ind = 0;
        ind += 2; // temp_mos
        ind += 2; // temp_motor
        ind += 4; // current_motor
        ind += 4; // current_in
        ind += 4; // id
        ind += 4; // iq
        ind += 2; // duty_now
        ind += 4; // rpm
        float v_in = (float)buffer_get_int16(data, &ind) / 10.0f;
        if (v_in > 0.1f) {
            accumulated_values.v_in = v_in;
            status_5_received = true;
            update_and_notify_values();
        }
    } break;
    case COMM_GET_MCCONF_TEMP: {
        if (len < 49) {
            ESP_LOGW(TAG, "MCCONF_TEMP packet too short (%d)", (int)len);
            break;
        }
        for (int i = 0; i < 10; i++) {
            buffer_get_float32_auto(data, &ind);
        }
        mc_temp_config.motor_poles    = data[ind++];
        mc_temp_config.gear_ratio     = buffer_get_float32_auto(data, &ind);
        mc_temp_config.wheel_diameter = buffer_get_float32_auto(data, &ind);
        mc_temp_config.valid          = true;
        if (rx_mcconf_temp_func) {
            rx_mcconf_temp_func(&mc_temp_config);
        }
    } break;
    default:
        break;
    }
}

// Callback setters
void bldc_interface_can_set_rx_value_func(void(*func)(mc_values *values)) {
    rx_value_func = func;
}

void bldc_interface_can_set_rx_mcconf_temp_func(void(*func)(mc_temp_config_t *conf)) {
    rx_mcconf_temp_func = func;
}

// Process received CAN frame
void bldc_interface_can_process_rx_frame(uint32_t eid, uint8_t *data, uint8_t len) {
    uint8_t controller_id = eid & 0xFF;
    CAN_PACKET_ID cmd = (CAN_PACKET_ID)((eid >> 8) & 0xFF);
    static uint32_t processed_count = 0;

    // Accept all VESC STATUS messages for auto-detection
    // Also accept broadcast (255) and responses from detected VESCs
    bool is_status_message = (cmd == CAN_PACKET_STATUS ||
                              cmd == CAN_PACKET_STATUS_2 ||
                              cmd == CAN_PACKET_STATUS_3 ||
                              cmd == CAN_PACKET_STATUS_4 ||
                              cmd == CAN_PACKET_STATUS_5);

    bool is_broadcast = (controller_id == 255);
    bool is_detected_vesc = false;

    // Check if this is from a detected VESC
    for (int i = 0; i < num_detected_vescs; i++) {
        if (detected_vescs[i].id == controller_id && detected_vescs[i].active) {
            is_detected_vesc = true;
            break;
        }
    }

    // Accept STATUS messages (for auto-detection), broadcast, or detected VESCs
    if (!is_status_message && !is_broadcast && !is_detected_vesc) {
        // Unknown VESC - could be another device on the bus, ignore silently
        return;
    }

    // Auto-detect VESC ID from STATUS messages
    if (is_status_message && controller_id != 255) {
        detect_vesc_id(controller_id);
    }

    processed_count++;
    // Log processed frames at debug level (can be enabled if needed)
    ESP_LOGD(TAG, "Processing frame: ID=%d cmd=%d len=%d (processed %" PRIu32 ")",
             controller_id, cmd, len, processed_count);

    switch (cmd) {
        case CAN_PACKET_STATUS: {
            // Process STATUS from every VESC — ERPM tracking is safety-critical for all motors.
            // process_status_message guards accumulated_values updates to the primary VESC only.
            ESP_LOGD(TAG, "Status message received from ID=%d", controller_id);
            process_status_message(controller_id, data, len);
            break;
        }

        case CAN_PACKET_STATUS_2: {
            if (controller_id == get_primary_vesc_id() || !primary_vesc_detected) {
                ESP_LOGD(TAG, "Status 2 message received from ID=%d", controller_id);
                process_status_2_message(controller_id, data, len);
            }
            break;
        }

        case CAN_PACKET_STATUS_3: {
            if (controller_id == get_primary_vesc_id() || !primary_vesc_detected) {
                ESP_LOGD(TAG, "Status 3 message received from ID=%d", controller_id);
                process_status_3_message(controller_id, data, len);
            }
            break;
        }

        case CAN_PACKET_STATUS_4: {
            if (controller_id == get_primary_vesc_id() || !primary_vesc_detected) {
                ESP_LOGD(TAG, "Status 4 message received from ID=%d", controller_id);
                process_status_4_message(controller_id, data, len);
            }
            break;
        }

        case CAN_PACKET_STATUS_5: {
            if (controller_id == get_primary_vesc_id() || !primary_vesc_detected) {
                ESP_LOGD(TAG, "Status 5 message received from ID=%d", controller_id);
                process_status_5_message(controller_id, data, len);
            }
            break;
        }

        case CAN_PACKET_PROCESS_SHORT_BUFFER: {
            // Short response (≤6 bytes) - this is how VESC sends responses
            // Accept responses from primary VESC or any detected VESC
            if (controller_id == get_primary_vesc_id() ||
                (controller_id != 255 && is_detected_vesc)) {
                if (len < 2) {
                    ESP_LOGW(TAG, "Short buffer too small: len=%d", len);
                    break;
                }
                // sender_id = data[0];  // Not used
                // send_type = data[1];  // Not used

                ESP_LOGD(TAG, "VESC response received from ID=%d: sender_id=%d send_type=%d len=%d cmd=0x%02X",
                         controller_id, data[0], data[1], len, (len > 2) ? data[2] : 0);

                // Process the response packet (skip sender_id and send_type)
                if (len > 2) {
                    process_response_packet(data + 2, len - 2);
                }
            }
            break;
        }

        case CAN_PACKET_FILL_RX_BUFFER: {
            if (len < 1) {
                ESP_LOGW(TAG, "FILL_RX_BUFFER too small: len=%d", len);
                break;
            }
            if (controller_id == get_primary_vesc_id() ||
                (controller_id != 255 && is_detected_vesc)) {

                uint8_t offset = data[0];
                uint8_t frag_len = len - 1;
                vesc_rx_buf_t *buf = rx_buf_for_id(controller_id, offset == 0);
                if (!buf) break;

                buf->timeout_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (offset + frag_len <= RX_BUFFER_SIZE) {
                    memcpy(buf->data + offset, data + 1, frag_len);
                    if (offset + frag_len > buf->len) {
                        buf->len = offset + frag_len;
                    }
                    ESP_LOGD(TAG, "RX frag ID=%d offset=%d frag=%d total=%d",
                             controller_id, offset, frag_len, buf->len);
                } else {
                    ESP_LOGW(TAG, "RX buffer overflow ID=%d offset=%d", controller_id, offset);
                    buf->active = false;
                }
            }
            break;
        }

        case CAN_PACKET_FILL_RX_BUFFER_LONG: {
            if (len < 2) {
                ESP_LOGW(TAG, "FILL_RX_BUFFER_LONG too small: len=%d", len);
                break;
            }
            if (controller_id == get_primary_vesc_id() ||
                (controller_id != 255 && is_detected_vesc)) {

                uint16_t offset = ((uint16_t)data[0] << 8) | data[1];
                uint8_t frag_len = len - 2;
                vesc_rx_buf_t *buf = rx_buf_for_id(controller_id, offset == 0);
                if (!buf) break;

                buf->timeout_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (offset + frag_len <= RX_BUFFER_SIZE) {
                    memcpy(buf->data + offset, data + 2, frag_len);
                    if (offset + frag_len > buf->len) {
                        buf->len = offset + frag_len;
                    }
                    ESP_LOGD(TAG, "RX long frag ID=%d offset=%d frag=%d total=%d",
                             controller_id, offset, frag_len, buf->len);
                } else {
                    ESP_LOGW(TAG, "RX buffer overflow ID=%d offset=%d", controller_id, offset);
                    buf->active = false;
                }
            }
            break;
        }

        case CAN_PACKET_PROCESS_RX_BUFFER: {
            if (len < 6) {
                ESP_LOGW(TAG, "PROCESS_RX_BUFFER too small: len=%d", len);
                break;
            }
            if (controller_id == get_primary_vesc_id() ||
                (controller_id != 255 && is_detected_vesc)) {

                uint16_t expected_len = ((uint16_t)data[2] << 8) | data[3];
                uint16_t expected_crc = ((uint16_t)data[4] << 8) | data[5];
                vesc_rx_buf_t *buf = rx_buf_for_id(controller_id, false);

                if (!buf || buf->len < expected_len) {
                    ESP_LOGW(TAG, "Buffer mismatch ID=%d: have=%d need=%d",
                             controller_id, buf ? buf->len : 0, expected_len);
                    if (buf) buf->active = false;
                    break;
                }

                uint16_t calculated_crc = crc16(buf->data, expected_len);
                if (calculated_crc == expected_crc) {
                    ESP_LOGD(TAG, "Multi-frame response complete: len=%d from ID=%d",
                             expected_len, controller_id);
                    process_response_packet(buf->data, expected_len);
                } else {
                    ESP_LOGW(TAG, "CRC mismatch ID=%d: expected=0x%04X calculated=0x%04X",
                             controller_id, expected_crc, calculated_crc);
                }
                buf->active = false;
            }
            break;
        }

        default:
            // Other status messages or unhandled packets
            ESP_LOGD(TAG, "Unhandled packet type: cmd=%d", cmd);
            break;
    }
}

static void process_status_message(uint8_t id, uint8_t *data, uint8_t len) {
    if (len < 8) return;

    int32_t ind = 0;

    int32_t raw_rpm = buffer_get_int32(data, &ind);

    // Discard ERPM values that exceed any physical limit — these occur during
    // VESC motor detection when the controller sweeps at high electrical
    // frequencies and would otherwise produce absurd speeds on the remote.
    if (raw_rpm > MAX_VALID_ERPM || raw_rpm < -MAX_VALID_ERPM) {
        ESP_LOGD(TAG, "Discarding out-of-range ERPM=%"PRId32" (motor detection?)", raw_rpm);
        raw_rpm = 0;
    }

    // Always track ERPM per-VESC — required for failsafe speed assessment across all motors
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    for (int i = 0; i < num_detected_vescs; i++) {
        if (detected_vescs[i].id == id) {
            detected_vescs[i].erpm = raw_rpm;
            detected_vescs[i].erpm_timestamp_ms = now_ms;
            break;
        }
    }

    // Only update accumulated_values (used for BLE telemetry) from the primary VESC
    if (id != get_primary_vesc_id() && primary_vesc_detected) {
        return;
    }

    // Use max-magnitude ERPM across all active VESCs so that if one motor is
    // spinning and the other is at rest the remote shows the real speed.
    int32_t display_erpm = raw_rpm;
    for (int i = 0; i < num_detected_vescs; i++) {
        if (!detected_vescs[i].active) continue;
        if (now_ms - detected_vescs[i].erpm_timestamp_ms > VESC_DETECTION_TIMEOUT_MS) continue;
        int32_t abs_i = detected_vescs[i].erpm < 0 ? -detected_vescs[i].erpm : detected_vescs[i].erpm;
        int32_t abs_d = display_erpm < 0 ? -display_erpm : display_erpm;
        if (abs_i > abs_d) {
            display_erpm = detected_vescs[i].erpm;
        }
    }
    accumulated_values.rpm = (float)display_erpm;
    accumulated_values.current_motor = (float)buffer_get_int16(data, &ind) / 10.0;
    accumulated_values.duty_now = (float)buffer_get_int16(data, &ind) / 1000.0;
    accumulated_values.vesc_id = id;
    status_received = true;

    update_and_notify_values();
}

static void process_status_2_message(uint8_t id, uint8_t *data, uint8_t len) {
    if (len < 8) return;

    int32_t ind = 0;

    accumulated_values.amp_hours = (float)buffer_get_int32(data, &ind) / 1e4;
    accumulated_values.amp_hours_charged = (float)buffer_get_int32(data, &ind) / 1e4;
    accumulated_values.vesc_id = id;
    status_2_received = true;

    update_and_notify_values();
}

static void process_status_3_message(uint8_t id, uint8_t *data, uint8_t len) {
    if (len < 8) return;

    int32_t ind = 0;

    accumulated_values.watt_hours = (float)buffer_get_int32(data, &ind) / 1e4;
    accumulated_values.watt_hours_charged = (float)buffer_get_int32(data, &ind) / 1e4;
    accumulated_values.vesc_id = id;
    status_3_received = true;

    update_and_notify_values();
}

static void process_status_4_message(uint8_t id, uint8_t *data, uint8_t len) {
    if (len < 8) return;

    int32_t ind = 0;

    accumulated_values.temp_mos = (float)buffer_get_int16(data, &ind) / 10.0;
    accumulated_values.temp_motor = (float)buffer_get_int16(data, &ind) / 10.0;
    accumulated_values.current_in = (float)buffer_get_int16(data, &ind) / 10.0;
    accumulated_values.pid_pos = (float)buffer_get_int16(data, &ind) / 50.0;
    accumulated_values.vesc_id = id;
    status_4_received = true;

    update_and_notify_values();
}

static void process_status_5_message(uint8_t id, uint8_t *data, uint8_t len) {
    if (len < 6) return;

    int32_t ind = 0;

    accumulated_values.tachometer = buffer_get_int32(data, &ind);
    float new_v_in = (float)buffer_get_int16(data, &ind) / 10.0;
    if (new_v_in > 1.0f) {
        accumulated_values.v_in = new_v_in;
    }
    accumulated_values.vesc_id = id;
    status_5_received = true;

    update_and_notify_values();
}

static void update_and_notify_values(void) {
    // Notify whenever we receive any STATUS message for responsive telemetry
    if (rx_value_func && (status_received || status_4_received || status_5_received)) {
        rx_value_func(&accumulated_values);
    }
}

// Check for inactive VESCs (haven't sent STATUS messages recently)
static void check_vesc_activity(void) {
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < num_detected_vescs; i++) {
        if (detected_vescs[i].active) {
            uint32_t time_since_last_seen = now_ms - detected_vescs[i].last_seen_ms;
            if (time_since_last_seen > VESC_DETECTION_TIMEOUT_MS) {
                ESP_LOGW(TAG, "VESC ID=%d inactive (no STATUS messages for %" PRIu32 " ms)",
                         detected_vescs[i].id, time_since_last_seen);
                detected_vescs[i].active = false;
                if (vesc_dropout_func) {
                    vesc_dropout_func(detected_vescs[i].id);
                }

                // If primary VESC went inactive, try to find another active one
                if (detected_vescs[i].id == primary_vesc_id) {
                    primary_vesc_id = 0;  // Reset primary
                    for (int j = 0; j < num_detected_vescs; j++) {
                        if (detected_vescs[j].active && detected_vescs[j].id != detected_vescs[i].id) {
                            primary_vesc_id = detected_vescs[j].id;
                            ESP_LOGI(TAG, "Switched primary VESC to ID=%d", primary_vesc_id);
                            break;
                        }
                    }
                    if (!primary_vesc_detected) {
                        ESP_LOGW(TAG, "No active VESC found");
                    }
                }
            }
        }
    }

    // Check for RX buffer timeouts (incomplete multi-frame responses)
    for (int i = 0; i < MAX_RX_SESSIONS; i++) {
        vesc_rx_buf_t *buf = &vesc_rx_bufs[i];
        if (buf->active && buf->timeout_ms > 0) {
            uint32_t elapsed = now_ms - buf->timeout_ms;
            if (elapsed > RX_BUFFER_TIMEOUT_MS) {
                ESP_LOGW(TAG, "RX buffer timeout: incomplete response from ID=%d (len=%d, elapsed=%" PRIu32 " ms)",
                         buf->vesc_id, buf->len, elapsed);
                buf->active = false;
            }
        }
    }
}

bool bldc_interface_can_has_fault(void) {
    return can_fault_active;
}

void bldc_interface_can_clear_fault(void) {
    can_consecutive_failures = 0;
    can_fault_active = false;
}

void bldc_interface_can_rx_task(void *pvParameters) {
    twai_message_t message;
    uint32_t frame_count = 0;
    uint32_t last_activity_check_ms = 0;

    ESP_LOGI(TAG, "CAN RX task started");

    while (1) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Check VESC activity every second
        if (now_ms - last_activity_check_ms >= 1000) {
            check_vesc_activity();
            last_activity_check_ms = now_ms;
        }

        if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (message.flags & TWAI_MSG_FLAG_EXTD) {
                // Extended ID frame
                frame_count++;
                // Log first frame only, periodic logging moved to DEBUG level
                if (frame_count == 1) {
                    ESP_LOGI(TAG, "RX: EID=0x%08" PRIX32 " len=%d (first frame)",
                             message.identifier, message.data_length_code);
                } else if (frame_count % 1000 == 0) {
                    ESP_LOGD(TAG, "RX: EID=0x%08" PRIX32 " len=%d (total frames: %" PRIu32 ")",
                             message.identifier, message.data_length_code, frame_count);
                }
                bldc_interface_can_process_rx_frame(message.identifier, message.data, message.data_length_code);
            } else {
                // Standard ID frame (shouldn't happen for VESC, but log it)
                ESP_LOGD(TAG, "RX: SID=0x%03" PRIX32 " (not extended, ignoring)", message.identifier);
            }
        }
    }
}
