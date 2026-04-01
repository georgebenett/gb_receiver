#pragma once

#include "esp_err.h"

esp_err_t aux_output_init(void);
void aux_output_set(uint8_t state);
void aux_output_update_pwm(void);
uint8_t aux_output_get_state(void);

