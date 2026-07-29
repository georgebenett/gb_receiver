
#include "bms.h"
#include "string.h"
#include "stdbool.h"
#include "esp_log.h"
#include "hw_config.h"

static const char *TAG = BMS_TAG;
static bms_values_t stored_bms_values = {0};

#define START_BYTE 0xDD
#define STOP_BYTE  0x77

// Largest BMS response is cell voltages for 16S: 4 + 32 + 3 = 39 bytes. 64 is safe.
#define BMS_RESPONSE_BUF 64

/** BMS frame checksum: two's complement of the sum over [status..payload]. */
static uint16_t bms_checksum(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)(~sum + 1);
}

static void bms_read_task(void *pvParameters);
static void print_bms_values(uint8_t *data, size_t len);
static esp_err_t send_command(uint8_t status, uint8_t cmd, const uint8_t *data, size_t data_len, uint8_t *response, size_t *response_len);

/** Return true if the buffer looks like a valid BMS frame (0xDD ... 0x77, min length, checksum). */
static bool is_valid_bms_response(const uint8_t *data, size_t len) {
    if (data == NULL || len < 6) return false;
    if (data[0] != START_BYTE || data[len - 1] != STOP_BYTE) return false;
    uint8_t data_len = data[3];
    if (len < (size_t)(4 + data_len + 3)) return false;  // status, cmd, len, payload, chk_hi, chk_lo, stop
    uint16_t expected = bms_checksum(&data[2], (size_t)data_len + 2);
    uint16_t got = ((uint16_t)data[len - 3] << 8) | data[len - 2];
    return (expected == got);
}

/** Try one pin assignment: set pins, flush, send basic info, return true if valid response. */
static bool try_pin_assignment(gpio_num_t tx_pin, gpio_num_t rx_pin, uint8_t *response, size_t *response_len) {
    uart_set_pin(BMS_UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_flush(BMS_UART_PORT);
    vTaskDelay(pdMS_TO_TICKS(50));  // let line settle
    esp_err_t err = send_command(0xA5, 0x03, NULL, 0, response, response_len);
    if (err != ESP_OK || *response_len == 0) return false;
    return is_valid_bms_response(response, *response_len);
}

esp_err_t bms_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = BMS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(BMS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BMS_UART_PORT, BMS_UART_PIN_A, BMS_UART_PIN_B, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(BMS_UART_PORT, BMS_BUF_SIZE * 2, 0, 0, NULL, 0));

    uint8_t probe[BMS_RESPONSE_BUF];
    size_t probe_len = 0;

    if (try_pin_assignment(BMS_UART_PIN_A, BMS_UART_PIN_B, probe, &probe_len)) {
        ESP_LOGI(TAG, "BMS UART pins detected: TX=GPIO%d, RX=GPIO%d", BMS_UART_PIN_A, BMS_UART_PIN_B);
    } else if (try_pin_assignment(BMS_UART_PIN_B, BMS_UART_PIN_A, probe, &probe_len)) {
        ESP_LOGI(TAG, "BMS UART pins detected: TX=GPIO%d, RX=GPIO%d", BMS_UART_PIN_B, BMS_UART_PIN_A);
    } else {
        ESP_LOGW(TAG, "BMS UART pin auto-detect: no valid response on either assignment (TX/RX may be swapped or BMS disconnected)");
    }

    xTaskCreate(bms_read_task, "bms_read_task", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "BMS UART initialized");
    return ESP_OK;
}

static esp_err_t send_command(uint8_t status, uint8_t cmd, const uint8_t *data, size_t data_len, uint8_t *response, size_t *response_len) {
    // Max frame: start(1) + status(1) + cmd(1) + len(1) + data(<=4) + chk(2) + stop(1) = 11
    uint8_t frame[16];
    size_t idx = 0;

    frame[idx++] = START_BYTE;
    frame[idx++] = status;
    frame[idx++] = cmd;
    frame[idx++] = (uint8_t)data_len;

    if (data && data_len > 0) {
        memcpy(&frame[idx], data, data_len);
        idx += data_len;
    }

    uint16_t chk = bms_checksum(&frame[2], data_len + 2);
    frame[idx++] = (chk >> 8) & 0xFF;
    frame[idx++] = chk & 0xFF;
    frame[idx++] = STOP_BYTE;

    uart_flush_input(BMS_UART_PORT);
    uart_write_bytes(BMS_UART_PORT, (const char *)frame, idx);

    int len = uart_read_bytes(BMS_UART_PORT, response, BMS_RESPONSE_BUF, pdMS_TO_TICKS(200));

    if (len <= 0) {
        return ESP_FAIL;
    }

    // Scan for start byte — stale partial frames can leave non-0xDD bytes at the front
    int start = -1;
    for (int i = 0; i < len; i++) {
        if (response[i] == START_BYTE) {
            start = i;
            break;
        }
    }
    if (start < 0) {
        return ESP_FAIL;
    }
    if (start > 0) {
        len -= start;
        memmove(response, response + start, len);
    }

    *response_len = len;
    return ESP_OK;
}

esp_err_t bms_read_basic_info(uint8_t *response, size_t *response_len) {
    return send_command(0xA5, 0x03, NULL, 0, response, response_len);
}

esp_err_t bms_read_cell_voltages(uint8_t *response, size_t *response_len) {
    return send_command(0xA5, 0x04, NULL, 0, response, response_len);
}

static void print_bms_values(uint8_t *data, size_t len) {
    if (len < 4) {
        ESP_LOGE(TAG, "Invalid packet length");
        return;
    }

    if (data[0] != START_BYTE) {
        ESP_LOGE(TAG, "Invalid start byte");
        return;
    }

    uint8_t status = data[1];
    uint8_t data_len = data[3];

    switch (status) {
        case 0x04:  // Cell Voltages
            if (len >= 6) {
                int num_cells = data_len / 2;
                if (num_cells > 16) num_cells = 16;
                stored_bms_values.num_cells = (uint8_t)num_cells;
                for (int i = 0; i < num_cells && (i * 2 + 5) < (int)len; i++) {
                    uint16_t cell_mv = (data[4 + i * 2] << 8) | data[5 + i * 2];
                    stored_bms_values.cell_voltages[i] = cell_mv / 1000.0f;
                }
            }
            break;

        case 0x03:  // Basic Info
            if (len >= 34) {
                stored_bms_values.total_voltage = (data[4] << 8 | data[5]) / 100.0f;
                stored_bms_values.current = ((int16_t)(data[6] << 8 | data[7])) / 100.0f;
                stored_bms_values.remaining_capacity = (data[8] << 8 | data[9]) / 100.0f;
                stored_bms_values.nominal_capacity = (data[10] << 8 | data[11]) / 100.0f;
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown status: 0x%02X", status);
            break;
    }
}

static void bms_read_task(void *pvParameters) {
    uint8_t response[BMS_RESPONSE_BUF];
    size_t response_len;
    uint8_t consecutive_failures = 0;
    const uint8_t MAX_FAILURES = 3;
    bool is_connected = true;

    while (1) {
        bool basic_ok = false;
        if (bms_read_basic_info(response, &response_len) == ESP_OK) {
            if (is_valid_bms_response(response, response_len)) {
                print_bms_values(response, response_len);
                consecutive_failures = 0;
                is_connected = true;
                basic_ok = true;
            } else {
                ESP_LOGW(TAG, "Basic info: invalid frame (checksum/length mismatch)");
                consecutive_failures++;
            }
        } else {
            consecutive_failures++;
        }

        if (!basic_ok) {
            if (consecutive_failures >= MAX_FAILURES) {
                if (is_connected) {
                    ESP_LOGW(TAG, "BMS disconnected");
                    is_connected = false;
                    memset(&stored_bms_values, 0, sizeof(bms_values_t));
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        if (is_connected) {
            vTaskDelay(pdMS_TO_TICKS(50));

            if (bms_read_cell_voltages(response, &response_len) == ESP_OK) {
                if (is_valid_bms_response(response, response_len)) {
                    print_bms_values(response, response_len);
                } else {
                    ESP_LOGW(TAG, "Cell voltages: invalid frame (checksum/length mismatch)");
                }
            }

            // Keeps the poll cadence unchanged now that the (unparsed) version
            // read that used to sit here is gone. The BMS is a 9600-baud device;
            // there is nothing to gain from hammering it faster.
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bms_values_t* get_stored_bms_values(void) {
    return &stored_bms_values;
}
