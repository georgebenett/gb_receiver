#pragma once

#include "esp_err.h"

// Initialize the auxiliary output GPIO
esp_err_t aux_output_init(void);

// Set the auxiliary output state (0 = OFF, 1 = ON)
void aux_output_set(uint8_t state);

