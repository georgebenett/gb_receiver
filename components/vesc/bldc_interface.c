/*
	Copyright 2016-2018 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * bldc_interface.c
 *
 * Compatible Firmware Versions
 * 3.39
 * 3.40
 *
 */

#include "bldc_interface.h"
#include "buffer.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "BLDC_IF";

static unsigned char send_buffer[1024];

static mc_values values;
static mc_temp_config_t mc_temp_config;

void send_packet_no_fwd(unsigned char *data, unsigned int len);

static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static void(*rx_value_func)(mc_values *values) = 0;
static void(*rx_mcconf_temp_func)(mc_temp_config_t *conf) = 0;

void bldc_interface_init(void(*func)(unsigned char *data, unsigned int len)) {
	send_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void bldc_interface_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void bldc_interface_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	int32_t ind = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_GET_VALUES:
		ESP_LOGD(TAG, "Parsing COMM_GET_VALUES response (len=%d)", len);
		ind = 0;
		values.temp_mos = buffer_get_float16(data, 1e1, &ind);
		values.temp_motor = buffer_get_float16(data, 1e1, &ind);
		values.current_motor = buffer_get_float32(data, 1e2, &ind);
		values.current_in = buffer_get_float32(data, 1e2, &ind);
		values.id = buffer_get_float32(data, 1e2, &ind);
		values.iq = buffer_get_float32(data, 1e2, &ind);
		values.duty_now = buffer_get_float16(data, 1e3, &ind);
		values.rpm = buffer_get_float32(data, 1e0, &ind);
		values.v_in = buffer_get_float16(data, 1e1, &ind);
		values.amp_hours = buffer_get_float32(data, 1e4, &ind);
		values.amp_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.tachometer = buffer_get_int32(data, &ind);
		values.tachometer_abs = buffer_get_int32(data, &ind);
		values.fault_code = (mc_fault_code)data[ind++];

		if (ind < (int)len) {
			values.pid_pos = buffer_get_float32(data, 1e6, &ind);
		} else {
			values.pid_pos = 0.0;
		}

		if (ind < (int)len) {
			values.vesc_id = data[ind++];
		} else {
			values.vesc_id = 255;
		}

		ESP_LOGD(TAG, "Parsed values: v_in=%.2fV, current_in=%.2fA, rpm=%.0f",
		         values.v_in, values.current_in, values.rpm);

		if (rx_value_func) {
			rx_value_func(&values);
		}
		break;

	case COMM_GET_MCCONF_TEMP: {
		ind = 0;

		if (len < 49) {
			ESP_LOGW("COMM_GET_MCCONF_TEMP", "packet too short (%d)", (int)len);
			break;
		}

		// Skip limit fields (10 × float32_auto = 10 reads)
		for (int i = 0; i < 10; i++) {
			buffer_get_float32_auto(data, &ind);
		}

		uint8_t motor_poles_u8 = data[ind++];
		float si_gear_ratio = buffer_get_float32_auto(data, &ind);
		float si_wheel_diameter = buffer_get_float32_auto(data, &ind);

		mc_temp_config.motor_poles = motor_poles_u8;
		mc_temp_config.gear_ratio = si_gear_ratio;
		mc_temp_config.wheel_diameter = si_wheel_diameter;
		mc_temp_config.valid = true;

		if (rx_mcconf_temp_func) {
			rx_mcconf_temp_func(&mc_temp_config);
		}
	} break;

	default:
		break;
	}
}

void bldc_interface_set_rx_value_func(void(*func)(mc_values *values)) {
	rx_value_func = func;
}

void bldc_interface_set_rx_mcconf_temp_func(void(*func)(mc_temp_config_t *conf)) {
	rx_mcconf_temp_func = func;
}

void bldc_interface_get_mcconf_temp(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_MCCONF_TEMP;
	send_packet_no_fwd(send_buffer, send_index);
}

const char* bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	default: return "Unknown fault";
	}
}

void send_packet_no_fwd(unsigned char *data, unsigned int len) {
	bldc_interface_send_packet(data, len);
}
