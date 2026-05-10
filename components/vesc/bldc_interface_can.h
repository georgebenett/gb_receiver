/*
 * bldc_interface_can.h
 *
 * CAN interface for VESC communication
 * Migrated from UART to CAN bus
 */

#ifndef BLDC_INTERFACE_CAN_H_
#define BLDC_INTERFACE_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "datatypes.h"

// CAN Packet IDs (from VESC firmware)
typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM = 3,
    CAN_PACKET_SET_POS = 4,
    CAN_PACKET_FILL_RX_BUFFER = 5,
    CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
    CAN_PACKET_PROCESS_RX_BUFFER = 7,
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
    CAN_PACKET_STATUS = 9,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
    CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
    CAN_PACKET_STATUS_2 = 14,
    CAN_PACKET_STATUS_3 = 15,
    CAN_PACKET_STATUS_4 = 16,
    CAN_PACKET_STATUS_5 = 27,
    CAN_PACKET_STATUS_6 = 58,
} CAN_PACKET_ID;

// Functions
void bldc_interface_can_init(gpio_num_t tx_pin, gpio_num_t rx_pin);
void bldc_interface_can_deinit(void);

// Multi-frame commands that solicit data back from the VESC (poll requests)
void bldc_interface_can_get_values(void);
void bldc_interface_can_get_mcconf_temp(void);
void bldc_interface_can_send_command(uint8_t *data, uint16_t len);

// Status message callbacks
void bldc_interface_can_set_rx_value_func(void(*func)(mc_values *values));
void bldc_interface_can_set_rx_mcconf_temp_func(void(*func)(mc_temp_config_t *conf));

// CAN RX task (runs in FreeRTOS task)
void bldc_interface_can_rx_task(void *pvParameters);

// VESC auto-detection queries
uint8_t bldc_interface_can_get_primary_vesc_id(void);  // Returns 0 if no VESC detected
uint8_t bldc_interface_can_get_detected_vesc_count(void);

// Per-VESC speed tracking (safety-critical: tracks all VESCs, not just primary)
int32_t bldc_interface_can_get_max_abs_erpm(void);          // Worst-case |ERPM| across all active VESCs
int32_t bldc_interface_can_get_signed_dominant_erpm(void);  // Signed ERPM of the largest-magnitude VESC
int32_t bldc_interface_can_get_erpm_at(uint8_t index);      // ERPM for detected VESC slot [index]
uint8_t bldc_interface_can_get_id_at(uint8_t index);        // VESC ID for detected slot [index]

// Multi-VESC commands — addressed to every active VESC independently, never via CAN forwarding.
// Both inputs are 0.0..1.0 fractions of each VESC's own configured |l_current_max| / |l_current_min|,
// making behavior portable across builds with different motor / battery configs.
void bldc_interface_can_set_current_rel_all(float current_rel);        // forward drive (0..1)
void bldc_interface_can_set_current_brake_rel_all(float current_rel);  // regen brake (0..1)

// Send zero-current and zero-brake to every active VESC. Use at boot or after CAN init
// to guarantee motors are in a known-quiescent state before any throttle source starts.
void bldc_interface_can_motors_safe_stop(void);

// VESC dropout callback (fired when a previously-active VESC stops sending STATUS)
void bldc_interface_can_set_vesc_dropout_func(void(*func)(uint8_t id));

// CAN bus fault status (set after CAN_TX_FAULT_THRESHOLD consecutive transmit failures)
bool bldc_interface_can_has_fault(void);
void bldc_interface_can_clear_fault(void);

// Internal: Process received CAN frame (called from CAN RX handler)
void bldc_interface_can_process_rx_frame(uint32_t eid, uint8_t *data, uint8_t len);

#endif /* BLDC_INTERFACE_CAN_H_ */
