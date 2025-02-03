#include <stdbool.h>
#include <stdint.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "nrfx_pwm.h"
#include "pwm.h"
#include "nrf_gpio.h"

#include "microbit_v2.h"


// PWM configuration
static const nrfx_pwm_t PWM_INST = NRFX_PWM_INSTANCE(0);

// Holds duty cycle values to trigger PWM toggle
nrf_pwm_values_individual_t sequence_data;

// Sequence structure for configuring DMA
nrf_pwm_sequence_t pwm_sequence = {
    .values.p_individual = &sequence_data,
    .length = 4,
    .repeats = 0,
    .end_delay = 0,
};


void pwm_init(void) {

    nrfx_pwm_config_t config;
    config.output_pins[0] = EDGE_P13;
    config.output_pins[1] = EDGE_P14; 
    config.output_pins[2] = EDGE_P15;
    config.output_pins[3] = NRFX_PWM_PIN_NOT_USED;
    config.base_clock = NRF_PWM_CLK_500kHz;
    config.count_mode = NRF_PWM_MODE_UP; 	
    config.top_value = 4000;
    config.load_mode = NRF_PWM_LOAD_INDIVIDUAL;
    config.step_mode = NRF_PWM_STEP_AUTO;

    nrfx_pwm_init(&PWM_INST, &config, NULL);
}

void pwm_set_duty_cycle(float duty_cycle0, float duty_cycle1, float duty_cycle2) {

    sequence_data.channel_0 = (uint16_t)(duty_cycle0 * 4000);
    sequence_data.channel_1 = (uint16_t)(duty_cycle1 * 4000);
    sequence_data.channel_2 = (uint16_t)(duty_cycle2 * 4000);


    nrfx_pwm_simple_playback(&PWM_INST, &pwm_sequence, 1, NRFX_PWM_FLAG_LOOP);
}