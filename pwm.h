#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "nrfx_pwm.h"

void pwm_init();
void pwm_set_duty_cycle(float duty_cycle0, float duty_cycle1, float duty_cycle2);