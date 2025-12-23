#include "usb_serial.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "ble.h"
#include "throttle.h"
#include "version.h"
#include "target_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bms.h"
#include "datatypes.h"

// Accessors provided in main.c
extern mc_temp_config_t* get_stored_mc_temp_config(void);

#define TAG "USB_SERIAL"

// Binary protocol state machine variables
static TaskHandle_t usb_task_handle = NULL;
static packet_state_t rx_state = STATE_WAIT_START;
static binary_packet_t rx_packet;
static uint16_t rx_payload_index = 0;
static uint16_t rx_crc_calculated = 0;

// Streaming configuration
static stream_config_t stream_config = {
    .enabled = false,
    .rate_hz = 10,  // Default 10Hz
    .last_send_ms = 0
};

// Forward declarations - binary protocol handlers
static void usb_serial_task(void *pvParameters);
static void handle_cmd_ping(const binary_packet_t* packet);
static void handle_cmd_get_firmware_version(const binary_packet_t* packet);
static void handle_cmd_get_config(const binary_packet_t* packet);
static void handle_cmd_start_streaming(const binary_packet_t* packet);
static void handle_cmd_stop_streaming(const binary_packet_t* packet);
static void handle_cmd_set_stream_rate(const binary_packet_t* packet);

// CRC-16-CCITT calculation (polynomial: 0x1021)
uint16_t calculate_crc16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void usb_serial_init(void)
{
    ESP_LOGI(TAG, "Initializing USB Serial Handler");
    ESP_LOGI(TAG, "Target: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "USB CDC Enabled: %d", USB_CDC_ENABLED);

    if (!USB_CDC_ENABLED) {
        ESP_LOGW(TAG, "USB CDC not enabled for this target");
        return;
    }

    // Add target-specific initialization delay
    vTaskDelay(pdMS_TO_TICKS(USB_CDC_INIT_DELAY_MS));

    usb_serial_init_esp32s3();

    // Additional delay to ensure USB is fully ready
    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize packet state machine
    rx_state = STATE_WAIT_START;
    rx_payload_index = 0;
    memset(&rx_packet, 0, sizeof(rx_packet));

    ESP_LOGI(TAG, "Binary Protocol v1.0 - Packet-based communication ready");
    ESP_LOGI(TAG, "Max payload size: %d bytes", PACKET_MAX_PAYLOAD_SIZE);
}

void usb_serial_start_task(void)
{
    if (!USB_CDC_ENABLED) {
        ESP_LOGW(TAG, "USB CDC not enabled, skipping task creation");
        return;
    }

    if (usb_task_handle == NULL) {
        xTaskCreate(usb_serial_task, "usb_serial_task", 4096, NULL, 5, &usb_task_handle);
    }
}

void usb_serial_init_esp32s3(void)
{
    ESP_LOGI(TAG, "Setting up USB Serial JTAG interface for ESP32-S3");

    usb_serial_jtag_driver_config_t usb_serial_jtag_config;
    usb_serial_jtag_config.rx_buffer_size = USB_CDC_BUFFER_SIZE;
    usb_serial_jtag_config.tx_buffer_size = USB_CDC_BUFFER_SIZE;

    esp_err_t ret = ESP_OK;
    /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB serial driver: %s", esp_err_to_name(ret));
        return;
    }

    /* Note: We use the raw driver API, not VFS, to avoid conflicts with console */
    ESP_LOGI(TAG, "USB Serial JTAG (Binary Mode) initialized successfully");
}

static void usb_serial_task(void *pvParameters)
{
    ESP_LOGI(TAG, "USB Serial task started");

    uint8_t temp_crc_buffer[PACKET_MAX_PAYLOAD_SIZE + 4]; // CMD + LEN(2) + PAYLOAD
    uint8_t rx_buffer[64];
    int rx_len = 0;

    for (;;) {
        // Handle streaming if enabled
        if (stream_config.enabled) {
            uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t interval_ms = 1000 / stream_config.rate_hz;
            if ((now_ms - stream_config.last_send_ms) >= interval_ms) {
                usb_serial_send_stream_data();
                stream_config.last_send_ms = now_ms;
            }
        }

        // Read bytes from USB serial using raw driver API (non-blocking)
        rx_len = usb_serial_jtag_read_bytes(rx_buffer, sizeof(rx_buffer), 0);
        if (rx_len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(USB_CDC_TASK_DELAY_MS));
            continue;
        }

        ESP_LOGD(TAG, "Received %d bytes", rx_len);

        // Process each received byte
        for (int i = 0; i < rx_len; i++) {
            uint8_t data = rx_buffer[i];

        switch (rx_state) {
            case STATE_WAIT_START:
                if (data == PACKET_START_BYTE) {
                    rx_state = STATE_WAIT_CMD;
                    rx_payload_index = 0;
                    memset(&rx_packet, 0, sizeof(rx_packet));
                }
                break;

            case STATE_WAIT_CMD:
                rx_packet.cmd_id = data;
                rx_state = STATE_WAIT_LEN_LSB;
                break;

            case STATE_WAIT_LEN_LSB:
                rx_packet.payload_length = data;
                rx_state = STATE_WAIT_LEN_MSB;
                break;

            case STATE_WAIT_LEN_MSB:
                rx_packet.payload_length |= ((uint16_t)data << 8);

                // Validate payload length
                if (rx_packet.payload_length > PACKET_MAX_PAYLOAD_SIZE) {
                    ESP_LOGW(TAG, "Invalid payload length: %d", rx_packet.payload_length);
                    rx_state = STATE_WAIT_START;
                } else if (rx_packet.payload_length == 0) {
                    // No payload, go directly to CRC
                    rx_state = STATE_WAIT_CRC_LSB;
                } else {
                    rx_state = STATE_WAIT_PAYLOAD;
                    rx_payload_index = 0;
                }
                break;

            case STATE_WAIT_PAYLOAD:
                rx_packet.payload[rx_payload_index++] = data;
                if (rx_payload_index >= rx_packet.payload_length) {
                    rx_state = STATE_WAIT_CRC_LSB;
                }
                break;

            case STATE_WAIT_CRC_LSB:
                rx_packet.crc = data;
                rx_state = STATE_WAIT_CRC_MSB;
                break;

            case STATE_WAIT_CRC_MSB:
                rx_packet.crc |= ((uint16_t)data << 8);

                // Calculate CRC over [CMD][LEN_LSB][LEN_MSB][PAYLOAD]
                temp_crc_buffer[0] = rx_packet.cmd_id;
                temp_crc_buffer[1] = rx_packet.payload_length & 0xFF;
                temp_crc_buffer[2] = (rx_packet.payload_length >> 8) & 0xFF;
                memcpy(&temp_crc_buffer[3], rx_packet.payload, rx_packet.payload_length);

                rx_crc_calculated = calculate_crc16(temp_crc_buffer, 3 + rx_packet.payload_length);

                if (rx_crc_calculated == rx_packet.crc) {
                    // Valid packet received, process it
                    ESP_LOGD(TAG, "Valid packet: CMD=0x%02X, LEN=%d",
                             rx_packet.cmd_id, rx_packet.payload_length);
                    usb_serial_process_packet(&rx_packet);
                } else {
                    ESP_LOGW(TAG, "CRC mismatch: expected 0x%04X, got 0x%04X",
                             rx_crc_calculated, rx_packet.crc);
                    usb_serial_send_ack(rx_packet.cmd_id, ERR_CRC_MISMATCH);
                }

                rx_state = STATE_WAIT_START;
                break;

            default:
                rx_state = STATE_WAIT_START;
                break;
            }
        }

        vTaskDelay(USB_CDC_TASK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

// Send a binary response packet
void usb_serial_send_response(uint8_t cmd_id, const uint8_t* payload, uint16_t length) {
    if (length > PACKET_MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload too large: %d bytes", length);
        return;
    }

    // Build packet: [START][CMD][LEN_LSB][LEN_MSB][PAYLOAD][CRC_LSB][CRC_MSB]
    uint8_t packet[PACKET_MAX_PAYLOAD_SIZE + 10];
    uint16_t idx = 0;

    packet[idx++] = PACKET_START_BYTE;
    packet[idx++] = cmd_id;
    packet[idx++] = length & 0xFF;
    packet[idx++] = (length >> 8) & 0xFF;

    if (length > 0 && payload != NULL) {
        memcpy(&packet[idx], payload, length);
        idx += length;
    }

    // Calculate CRC over [CMD][LEN][PAYLOAD]
    uint16_t crc = calculate_crc16(&packet[1], 3 + length);
    packet[idx++] = crc & 0xFF;
    packet[idx++] = (crc >> 8) & 0xFF;

    // Send packet
    usb_serial_jtag_write_bytes((const char*)packet, idx, pdMS_TO_TICKS(100));

    ESP_LOGD(TAG, "Sent response: CMD=0x%02X, LEN=%d, CRC=0x%04X", cmd_id, length, crc);
}

// Send ACK/NACK response
void usb_serial_send_ack(uint8_t original_cmd, error_code_t error_code) {
    uint8_t payload[2];
    payload[0] = original_cmd;
    payload[1] = (uint8_t)error_code;
    usb_serial_send_response(RSP_ACK, payload, 2);
}

// Send error response with message
void usb_serial_send_error(error_code_t error_code, const char* message) {
    uint8_t payload[256];
    payload[0] = (uint8_t)error_code;

    uint16_t msg_len = 0;
    if (message != NULL) {
        msg_len = strlen(message);
        if (msg_len > 255) msg_len = 255;
        memcpy(&payload[1], message, msg_len);
    }

    usb_serial_send_response(RSP_ERROR, payload, 1 + msg_len);
}

// Process received binary packet
void usb_serial_process_packet(const binary_packet_t* packet) {
    switch (packet->cmd_id) {
        case CMD_PING:
            handle_cmd_ping(packet);
            break;
        case CMD_GET_FIRMWARE_VERSION:
            handle_cmd_get_firmware_version(packet);
            break;
        case CMD_GET_CONFIG:
            handle_cmd_get_config(packet);
            break;
        case CMD_START_STREAMING:
            handle_cmd_start_streaming(packet);
            break;
        case CMD_STOP_STREAMING:
            handle_cmd_stop_streaming(packet);
            break;
        case CMD_SET_STREAM_RATE:
            handle_cmd_set_stream_rate(packet);
            break;
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", packet->cmd_id);
            usb_serial_send_ack(packet->cmd_id, ERR_UNKNOWN_CMD);
            break;
    }
}

// ========== BINARY PROTOCOL COMMAND HANDLERS ==========

static void handle_cmd_ping(const binary_packet_t* packet) {
    // Simple ping response - just ACK
    usb_serial_send_ack(CMD_PING, ERR_OK);
}

static void handle_cmd_get_firmware_version(const binary_packet_t* packet) {
    uint8_t payload[256];
    uint16_t idx = 0;

    // Parse version string to get individual components
    uint8_t major = 0, minor = 0, patch = 0;
    sscanf(FW_VERSION, "%hhu.%hhu.%hhu", &major, &minor, &patch);
    payload[idx++] = major;
    payload[idx++] = minor;
    payload[idx++] = patch;

    // Build date string
    char build_str[64];
    snprintf(build_str, sizeof(build_str), "%s %s", BUILD_DATE, BUILD_TIME);
    uint8_t build_len = strlen(build_str);
    payload[idx++] = build_len;
    memcpy(&payload[idx], build_str, build_len);
    idx += build_len;

    // Model name
    uint8_t model_len = strlen(TARGET_NAME);
    payload[idx++] = model_len;
    memcpy(&payload[idx], TARGET_NAME, model_len);
    idx += model_len;

    // IDF version
    const char* idf_ver = esp_get_idf_version();
    uint8_t idf_len = strlen(idf_ver);
    payload[idx++] = idf_len;
    memcpy(&payload[idx], idf_ver, idf_len);
    idx += idf_len;

    usb_serial_send_response(RSP_FIRMWARE_VERSION, payload, idx);
}

static void handle_cmd_get_config(const binary_packet_t* packet) {
    uint8_t payload[256];
    uint16_t idx = 0;

    // Status flags
    uint8_t flags = 0;
    if (ble_is_connected()) flags |= 0x01;  // BLE connected
    payload[idx++] = flags;

    // Current throttle value (from BLE)
    uint16_t throttle_value = current_throttle_value;
    payload[idx++] = throttle_value & 0xFF;
    payload[idx++] = (throttle_value >> 8) & 0xFF;

    // Compact motor configuration (motor poles, gear ratio, wheel diameter)
    mc_temp_config_t* mc_temp_conf = get_stored_mc_temp_config();
    uint8_t motor_poles = 0;
    uint16_t gear_ratio_scaled = 0;
    uint16_t wheel_diameter_scaled = 0;

    if (mc_temp_conf != NULL && mc_temp_conf->valid) {
        motor_poles = mc_temp_conf->motor_poles;
        gear_ratio_scaled = (uint16_t)((mc_temp_conf->gear_ratio * 1000.0f) + 0.5f);
        wheel_diameter_scaled = (uint16_t)((mc_temp_conf->wheel_diameter * 1000.0f) + 0.5f);
    }

    payload[idx++] = motor_poles;
    payload[idx++] = gear_ratio_scaled & 0xFF;
    payload[idx++] = (gear_ratio_scaled >> 8) & 0xFF;
    payload[idx++] = wheel_diameter_scaled & 0xFF;
    payload[idx++] = (wheel_diameter_scaled >> 8) & 0xFF;

    usb_serial_send_response(RSP_CONFIG, payload, idx);
}

static void handle_cmd_start_streaming(const binary_packet_t* packet) {
    uint16_t rate_hz = 10; // Default 10Hz

    if (packet->payload_length >= 2) {
        // Payload: [rate_hz_lsb][rate_hz_msb] (little-endian)
        rate_hz = packet->payload[0] | (packet->payload[1] << 8);
    }

    // Validate rate (1Hz - 100Hz)
    if (rate_hz < 1) rate_hz = 1;
    if (rate_hz > 100) rate_hz = 100;

    usb_serial_start_streaming(rate_hz);

    ESP_LOGI(TAG, "Streaming started at %d Hz", rate_hz);
    usb_serial_send_ack(CMD_START_STREAMING, ERR_OK);
}

static void handle_cmd_stop_streaming(const binary_packet_t* packet) {
    usb_serial_stop_streaming();
    ESP_LOGI(TAG, "Streaming stopped");
    usb_serial_send_ack(CMD_STOP_STREAMING, ERR_OK);
}

static void handle_cmd_set_stream_rate(const binary_packet_t* packet) {
    if (packet->payload_length != 2) {
        usb_serial_send_ack(CMD_SET_STREAM_RATE, ERR_INVALID_PAYLOAD);
        return;
    }

    // Payload: [rate_hz_lsb][rate_hz_msb] (little-endian)
    uint16_t rate_hz = packet->payload[0] | (packet->payload[1] << 8);

    // Validate rate (1Hz - 100Hz)
    if (rate_hz < 1 || rate_hz > 100) {
        usb_serial_send_ack(CMD_SET_STREAM_RATE, ERR_OUT_OF_RANGE);
        return;
    }

    stream_config.rate_hz = rate_hz;

    ESP_LOGI(TAG, "Streaming rate changed to %d Hz", rate_hz);
    usb_serial_send_ack(CMD_SET_STREAM_RATE, ERR_OK);
}

void usb_serial_start_streaming(uint16_t rate_hz) {
    stream_config.enabled = true;
    stream_config.rate_hz = rate_hz;
    stream_config.last_send_ms = 0;
}

void usb_serial_stop_streaming(void) {
    stream_config.enabled = false;
}

void usb_serial_send_stream_data(void) {
    if (!stream_config.enabled) {
        return;
    }

    extern mc_values* get_stored_vesc_values(void);
    extern bms_values_t* get_stored_bms_values(void);

    uint8_t payload[256];  // Increased size for cell voltages
    uint16_t idx = 0;

    // Timestamp (4 bytes, little-endian)
    uint32_t timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    payload[idx++] = (timestamp_ms >> 0) & 0xFF;
    payload[idx++] = (timestamp_ms >> 8) & 0xFF;
    payload[idx++] = (timestamp_ms >> 16) & 0xFF;
    payload[idx++] = (timestamp_ms >> 24) & 0xFF;

    // Status flags
    uint8_t flags = 0;
    if (ble_is_connected()) flags |= 0x01;  // BLE connected
    payload[idx++] = flags;

    // Current throttle value from BLE (2 bytes, little-endian)
    uint16_t throttle_value = current_throttle_value;
    payload[idx++] = throttle_value & 0xFF;
    payload[idx++] = (throttle_value >> 8) & 0xFF;

    // VESC data (if available)
    mc_values* vesc_values = get_stored_vesc_values();
    if (vesc_values != NULL) {
        // Voltage (2 bytes, little-endian, scaled by 100)
        int16_t voltage = (int16_t)(vesc_values->v_in * 100);
        payload[idx++] = voltage & 0xFF;
        payload[idx++] = (voltage >> 8) & 0xFF;

        // Current motor (2 bytes, little-endian, scaled by 100)
        int16_t current_motor = (int16_t)(vesc_values->current_motor * 100);
        payload[idx++] = current_motor & 0xFF;
        payload[idx++] = (current_motor >> 8) & 0xFF;

        // Current input (2 bytes, little-endian, scaled by 100)
        int16_t current_input = (int16_t)(vesc_values->current_in * 100);
        payload[idx++] = current_input & 0xFF;
        payload[idx++] = (current_input >> 8) & 0xFF;

        // Duty cycle (2 bytes, little-endian, scaled by 1000)
        int16_t duty = (int16_t)(vesc_values->duty_now * 1000);
        payload[idx++] = duty & 0xFF;
        payload[idx++] = (duty >> 8) & 0xFF;

        // RPM (4 bytes, little-endian)
        int32_t rpm = (int32_t)vesc_values->rpm;
        payload[idx++] = rpm & 0xFF;
        payload[idx++] = (rpm >> 8) & 0xFF;
        payload[idx++] = (rpm >> 16) & 0xFF;
        payload[idx++] = (rpm >> 24) & 0xFF;

        // Temperature MOSFET (2 bytes, little-endian, scaled by 100)
        int16_t temp_mos = (int16_t)(vesc_values->temp_mos * 100);
        payload[idx++] = temp_mos & 0xFF;
        payload[idx++] = (temp_mos >> 8) & 0xFF;

        // Temperature motor (2 bytes, little-endian, scaled by 100)
        int16_t temp_motor = (int16_t)(vesc_values->temp_motor * 100);
        payload[idx++] = temp_motor & 0xFF;
        payload[idx++] = (temp_motor >> 8) & 0xFF;
    } else {
        // No VESC data available - send zeros (16 bytes total)
        for (int i = 0; i < 16; i++) {
            payload[idx++] = 0;
        }
    }

    // BMS data (if available)
    bms_values_t* bms_values = get_stored_bms_values();
    if (bms_values != NULL) {
        // Total voltage (2 bytes, little-endian, scaled by 100)
        int16_t bms_voltage = (int16_t)(bms_values->total_voltage * 100);
        payload[idx++] = bms_voltage & 0xFF;
        payload[idx++] = (bms_voltage >> 8) & 0xFF;

        // Current (2 bytes, little-endian, scaled by 100)
        int16_t bms_current = (int16_t)(bms_values->current * 100);
        payload[idx++] = bms_current & 0xFF;
        payload[idx++] = (bms_current >> 8) & 0xFF;

        // Remaining capacity (2 bytes, little-endian, scaled by 100)
        int16_t remaining_cap = (int16_t)(bms_values->remaining_capacity * 100);
        payload[idx++] = remaining_cap & 0xFF;
        payload[idx++] = (remaining_cap >> 8) & 0xFF;

        // Nominal capacity (2 bytes, little-endian, scaled by 100)
        int16_t nominal_cap = (int16_t)(bms_values->nominal_capacity * 100);
        payload[idx++] = nominal_cap & 0xFF;
        payload[idx++] = (nominal_cap >> 8) & 0xFF;

        // Number of cells (1 byte)
        payload[idx++] = bms_values->num_cells;

        // Cell voltages (2 bytes each, little-endian, scaled by 1000, up to 16 cells)
        for (int i = 0; i < 16; i++) {
            int16_t cell_voltage;
            if (i < bms_values->num_cells) {
                cell_voltage = (int16_t)(bms_values->cell_voltages[i] * 1000);
            } else {
                cell_voltage = 0;
            }
            payload[idx++] = cell_voltage & 0xFF;
            payload[idx++] = (cell_voltage >> 8) & 0xFF;
        }
    } else {
        // No BMS data available - send zeros (8 bytes for basic + 1 for num_cells + 32 for cells = 41 bytes)
        for (int i = 0; i < 41; i++) {
            payload[idx++] = 0;
        }
    }

    // Motor configuration (motor poles, gear ratio, wheel diameter)
    mc_temp_config_t* mc_temp_conf = get_stored_mc_temp_config();
    uint8_t motor_poles = 0;
    uint16_t gear_ratio_scaled = 0;
    uint16_t wheel_diameter_scaled = 0;

    if (mc_temp_conf != NULL && mc_temp_conf->valid) {
        motor_poles = mc_temp_conf->motor_poles;
        gear_ratio_scaled = (uint16_t)((mc_temp_conf->gear_ratio * 1000.0f) + 0.5f);
        wheel_diameter_scaled = (uint16_t)((mc_temp_conf->wheel_diameter * 1000.0f) + 0.5f);
    }

    payload[idx++] = motor_poles;
    payload[idx++] = gear_ratio_scaled & 0xFF;
    payload[idx++] = (gear_ratio_scaled >> 8) & 0xFF;
    payload[idx++] = wheel_diameter_scaled & 0xFF;
    payload[idx++] = (wheel_diameter_scaled >> 8) & 0xFF;

    usb_serial_send_response(RSP_STREAM_DATA, payload, idx);
}
