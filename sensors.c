#include <stdbool.h>
#include <stdint.h>

#include <math.h>
#include "sensors.h"


static const nrf_twi_mngr_t* i2c_manager = NULL;

static uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf[2];
  uint16_t return_buf;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, rx_buf, 2, 0),
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C transaction failed! Error: %lX\n", result);
  }
  return_buf = ((uint16_t) rx_buf[1] << 8) | rx_buf[0];
  return return_buf;
}



static void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* data) {
  uint8_t reg_addr_data[3] = {reg_addr, data[0], data[1]};
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr_data, 3, 0)
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 1, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C transaction failed! Error: %lX\n", result);
  }
}



static uint8_t i2c_reg_read_byte(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, &rx_buf, 1, 0),
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C transaction failed! Error: %lX\n", result);
  };
  return rx_buf;
}



static void i2c_reg_write_byte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  uint8_t reg_addr_data[2] = {reg_addr, data};
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr_data, 2, 0)
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 1, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C transaction failed! Error: %lX\n", result);
  }
}

void embedded_reg_write(uint8_t address, uint8_t value) {

  i2c_reg_write_byte(PRESENCE_ADDRESS, FUNC_CFG_ADDR, address);
  i2c_reg_write_byte(PRESENCE_ADDRESS, FUNC_CFG_DATA, value);
  i2c_reg_write_byte(PRESENCE_ADDRESS, PAGE_RW, 0x40);
  i2c_reg_write_byte(PRESENCE_ADDRESS, PAGE_RW, 0x00);

}

void amb_light_init(const nrf_twi_mngr_t* i2c) {

  i2c_manager = i2c;

  uint8_t config[] = {0b00010000, 0b11000000};
  i2c_reg_write(AMB_LIGHT_ADDRESS, LIGHT_CONFIG, &config);

}

uint16_t get_amb_light_val() {

  return i2c_reg_read(AMB_LIGHT_ADDRESS, LIGHT_ALS_HIGH);

}

void presence_init(const nrf_twi_mngr_t* i2c) {

  i2c_manager = i2c;

  i2c_reg_write_byte(PRESENCE_ADDRESS, CTRL2, 0x10);

  embedded_reg_write(MOTION_THS_H, 0x01);
  embedded_reg_write(MOTION_THS_L, 0xF4);
  embedded_reg_write(HYST_MOTION, 0xFA);
  embedded_reg_write(RESET_ALGO, 0x01);
  embedded_reg_write(RESET_ALGO, 0x00);

  i2c_reg_write_byte(PRESENCE_ADDRESS, CTRL2, 0x00);

  i2c_reg_write_byte(PRESENCE_ADDRESS, CTRL1, 0x05);
}

bool motion_detect() {

  uint8_t status = i2c_reg_read_byte(PRESENCE_ADDRESS, FUNC_STATUS);
  // printf("FUNC_STATUS: %x\n", status);

  uint8_t motion_l = i2c_reg_read_byte(PRESENCE_ADDRESS, TMOTION_L);
  uint8_t motion_h = i2c_reg_read_byte(PRESENCE_ADDRESS, TMOTION_H);
  int16_t motion = (uint16_t)motion_h << 8 | motion_l;
  printf("Motion Val: %d\n", motion);

  //return ((status >> 1) & 0b1);
  return ((motion > 500) || (motion < -500));
}
