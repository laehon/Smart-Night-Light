#pragma once

#include "nrf_twi_mngr.h"

// Chip addresses for ambient light 
static const uint8_t AMB_LIGHT_ADDRESS = 0x48;
static const uint8_t PRESENCE_ADDRESS = 0x5A;

// Register definitions for Ambient Light Sensor
typedef enum {
  LIGHT_CONFIG = 0x00,
  LIGHT_ALS_HIGH = 0x04,
} amb_light_reg_t;

typedef enum {
  PRES_WHO_AM_I = 0x0F,
  FUNC_STATUS = 0x25,
  CTRL2 = 0x21,
  PAGE_RW = 0x11,
  TMOTION_H = 0x3D,
  TMOTION_L = 0x3C,
  STATUS = 0x23,
  MOTION_THS_H = 0x23,
  MOTION_THS_L = 0x22,
  HYST_MOTION = 0x26,
  CTRL1 = 0x20,
  RESET_ALGO = 0x2A,
  FUNC_CFG_ADDR = 0x08,
  FUNC_CFG_DATA = 0x09,
} presence_reg_t;

// Function prototypes

// Initialize and configure the LSM303AGR accelerometer/magnetometer
//
// i2c - pointer to already initialized and enabled twim instance
void amb_light_init(const nrf_twi_mngr_t* i2c);
void presence_init(const nrf_twi_mngr_t* i2c);

bool motion_detect();
uint16_t get_amb_light_val();


