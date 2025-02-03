// I2C sensors app
//
// Read from I2C sensors on the Microbit

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h> 

#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "nrfx_saadc.h"
#include "nrf.h"
#include "nrf_delay.h"

#include "microbit_v2.h"
#include "sensors.h"
#include "pwm.h"



#define ANALOG_MIC_IN NRF_SAADC_INPUT_AIN0

#define ADC_MIC_CHANNEL 0
#define ADC_MAX_COUNTS (16384)

#define SAMPLING_FREQUENCY 16000 
#define TIMER_TICKS (16000000 / SAMPLING_FREQUENCY)

#define BASELINE_SAMPLES 100
volatile uint16_t mic_baseline = 0; 
volatile bool baseline_ready = false; 
static int16_t adc_buffer[1]; 

NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);

APP_TIMER_DEF(movement_timer);
bool movement_detected = false;

const unsigned int ambientMin = 40;
const unsigned int ambientMax = 500;

volatile float loudness = 0.0;
volatile bool partymode = false;

float loudness_buffer[100];

static void clear_movement_detected() {
  movement_detected = false;
}

float mapBrightnessToPWM(int ambientBrightness) { 

  if (ambientBrightness < ambientMin) ambientBrightness = ambientMin;
  if (ambientBrightness > ambientMax) ambientBrightness = ambientMax;

  float normalizedBrightness = 1.0f + 9.0f * (float)(ambientBrightness - ambientMin) / (float)(ambientMax - ambientMin);

  float logBrightness = log10f(normalizedBrightness); 
  float pwmDutyCycle = 1.0f - logBrightness;

  return pwmDutyCycle;
}

// Timer interrupt handler
void TIMER4_IRQHandler(void) {

    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    NRF_TIMER4->CC[0] += TIMER_TICKS;

    nrfx_saadc_sample();
}

// SAADC callback for processing samples
static void saadc_event_callback(nrfx_saadc_evt_t const *event) {
    if (event->type == NRFX_SAADC_EVT_DONE) {
        int16_t sample = event->data.done.p_buffer[0]; 
        if (!baseline_ready) {
            // Collect initial samples to compute baseline
            static uint32_t baseline_sum = 0;
            static uint16_t sample_count = 0;
            baseline_sum += sample;
            sample_count++;
            if (sample_count >= BASELINE_SAMPLES) {
                mic_baseline = baseline_sum / sample_count;
                baseline_ready = true;
            }
        } else {
            int16_t deviation = abs(sample - mic_baseline);
            loudness = (float)deviation / ADC_MAX_COUNTS;
            if (loudness > 1.0) {
                loudness = 1.0;
            }
        }
        nrfx_saadc_buffer_convert(adc_buffer, 2);
    } else {
        printf("Unexpected SAADC event!\n");
    }
}

// SAADC initialization
static void adc_init(void) {
    // Initialize the SAADC
    nrfx_saadc_config_t saadc_config = {
        .resolution = NRF_SAADC_RESOLUTION_14BIT,
        .oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = 1,
        .low_power_mode = false,
    };
    nrfx_saadc_init(&saadc_config, saadc_event_callback);

    // Initialize the microphone ADC channel
    nrf_saadc_channel_config_t mic_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ANALOG_MIC_IN);
    mic_channel_config.gain = NRF_SAADC_GAIN1_4;
    mic_channel_config.acq_time = NRF_SAADC_ACQTIME_3US;
    nrfx_saadc_channel_init(ADC_MIC_CHANNEL, &mic_channel_config);

    // Initialize the buffer for continuous sampling
    nrfx_saadc_buffer_convert(adc_buffer, 2);
}

// Timer initialization
static void timer_init(void) {
    NRF_TIMER4->BITMODE = 3; 
    NRF_TIMER4->PRESCALER = 0;
    NRF_TIMER4->CC[0] = TIMER_TICKS;
    NRF_TIMER4->INTENSET = 1 << TIMER_INTENSET_COMPARE0_Pos;

    NVIC_ClearPendingIRQ(TIMER4_IRQn);
    NVIC_SetPriority(TIMER4_IRQn, 7);
    NVIC_EnableIRQ(TIMER4_IRQn);

    NRF_TIMER4->TASKS_CLEAR = 1;
    NRF_TIMER4->TASKS_START = 1;
}


int main(void) {
  printf("Board started!\n");

  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  // WARNING!!
  // These are NOT the correct pins for external I2C communication.
  // If you are using QWIIC or other external I2C devices, the are
  // connected to EDGE_P19 (a.k.a. I2C_QWIIC_SCL) and EDGE_P20 (a.k.a. I2C_QWIIC_SDA)
  i2c_config.scl = I2C_QWIIC_SCL;
  i2c_config.sda = I2C_QWIIC_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  amb_light_init(&twi_mngr_instance);
  presence_init(&twi_mngr_instance);
  pwm_init();

  adc_init();
  timer_init();
  nrf_gpio_cfg_input(14, NRF_GPIO_PIN_PULLUP);

  float red, green, blue, pwm = 0;

  float red_duty = .99;
  float green_duty = 0.49;
  float blue_duty = 0.0;
  float red_step = 0.001; 
  float green_step = 0.001;
  float blue_step = 0.001;

  app_timer_init();
  app_timer_create(&movement_timer, APP_TIMER_MODE_SINGLE_SHOT, clear_movement_detected);

  uint16_t light_val = 0;
  uint32_t counter = 0;
  float avg_loudness = 0.0;

  // Loop forever
  while (1) {
    
    if (partymode) {
      red_duty += red_step;
      green_duty += green_step;
      blue_duty += blue_step;

      if (red_duty <= 0.0 || red_duty >= 1.0) red_step = -red_step;
      if (green_duty <= 0.0 || green_duty >= 1.0) green_step = -green_step;
      if (blue_duty <= 0.0 || blue_duty >= 1.0) blue_step = -blue_step;

      avg_loudness = 0;
      loudness_buffer[counter % 100] = loudness;
      for (int i = 0; i < 100; i++) {
        avg_loudness += loudness_buffer[i] + 0.1;
      }
      avg_loudness = avg_loudness * 2;

      if ((loudness + 0.1) * 200 > 2 * avg_loudness) {

        red = avg_loudness * red_duty;
        green = avg_loudness * green_duty;
        blue = avg_loudness * blue_duty;

        pwm_set_duty_cycle(red, green, blue);

        nrf_delay_ms(50);
      
      }

      red = 0;
      green = 0;
      blue = 0;

      pwm_set_duty_cycle(red, green, blue);

      printf("loudness: %.2f, %.2f\n", avg_loudness, loudness);

    }
    else {
      if (motion_detect()) {
        movement_detected = true;
        app_timer_start(movement_timer, 163840, NULL);
      } 
      
      light_val = get_amb_light_val();
      //printf("light val: %d\n", light_val);

      if(movement_detected) {
        pwm = mapBrightnessToPWM(light_val);
        red = pwm;
        green = pwm;
        blue = pwm;
      } else {
        red = 0;
        green = 0;
        blue = 0;
      }

      pwm_set_duty_cycle(red, green, blue);

    }

    // Logic for switching states
    if (nrf_gpio_pin_read(14) == 0){ 
      nrf_delay_ms(200);
      if(partymode == false){
        partymode = true;  
      }else if(partymode==true){
        partymode = false;
      }
      printf("partymode state: %d\n", partymode);
    }
  }
}
