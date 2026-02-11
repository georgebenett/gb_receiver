#pragma once

#include "esp_err.h"
#include <stdint.h>

/** Servo angle range (degrees) for PWM. */
#define SERVO_ANGLE_MIN  0
#define SERVO_ANGLE_MAX  180

/** Continuous-rotation servo: neutral (stop) and direction limits. */
#define SERVO_NEUTRAL        71   /* Angle at which the servo stops */
#define SERVO_ANGLE_MAX_CW   63   /* Max rotation clockwise */
#define SERVO_ANGLE_MAX_CCW   75   /* Max rotation counterclockwise */

/**
 * Initialize servo on GPIO 9 (SERVO_GPIO).
 * Uses LEDC timer 1, channel 2 at 50 Hz.
 * @return ESP_OK on success.
 */
esp_err_t servo_init(void);

/**
 * Set servo angle in degrees [0, 180].
 * @param angle_deg Angle in degrees (clamped to SERVO_ANGLE_MIN..SERVO_ANGLE_MAX).
 */
void servo_set_angle(uint8_t angle_deg);

/**
 * Set servo angle from throttle value [0, 255] (continuous-rotation mapping).
 * Maps: 0 -> max CW (50°), 127 -> neutral/stop (69°), 255 -> max CCW (83°).
 * @param throttle_value Throttle in range 0..255 (e.g. current_throttle_value).
 */
void servo_set_angle_from_throttle(uint16_t throttle_value);
