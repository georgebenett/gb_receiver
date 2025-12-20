#pragma once

#include "esp_err.h"

// Initialize the auxiliary output GPIO and LED PWM
esp_err_t aux_output_init(void);

// Set the auxiliary output state (0 = OFF, 1 = ON)
// The LED PWM will reflect this state, modulated by the current throttle value
void aux_output_set(uint8_t state);

// Update the PWM duty cycle based on current throttle value
// This should be called whenever the throttle value changes
void aux_output_update_pwm(void);

