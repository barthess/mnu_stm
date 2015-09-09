#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "ms5806.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
static int32_t pressure;
static int16_t temperature;

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/**
 *
 */
void MS5806::pickle(void) {
  temperature = rxbuf[2];
  temperature = -50 + (temperature * 200) / 256;

  pressure = 0;
  pressure |= (rxbuf[0] << 8) | rxbuf[1];
}

/**
 *
 */
MS5806::MS5806(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
bool MS5806::hw_init_full(void) {
  const uint8_t PROM = 0b10100000;
  uint8_t cmd;
  msg_t status;

  for (size_t i=0; i<MS5806_CAL_WORDS; i++) {
    cmd = PROM;
    cmd |= i << 1;
    status = transmit(&cmd, 1, (uint8_t*)&C[i], 2);
    osalDbgCheck(MSG_OK == status);
  }
  //uint16_t cal[MS5806_CAL_WORDS];

  return OSAL_SUCCESS;
}

/**
 *
 */
bool MS5806::hw_init_fast(void){
  return OSAL_SUCCESS;
}

/**
 *
 */
sensor_state_t MS5806::start(void) {
  if (OSAL_SUCCESS == hw_init_full())
    state = SENSOR_STATE_READY;
  else {
    state = SENSOR_STATE_DEAD;
    osalSysHalt("Sensor dead");
  }
  return state;
}

void MS5806::stop(void){
  state = SENSOR_STATE_STOP;
}

sensor_state_t MS5806::wakeup(void) {
  state = SENSOR_STATE_READY;
  return state;
}

void MS5806::sleep(void) {
  state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t MS5806::get(void){
  osalDbgCheck(state == SENSOR_STATE_READY);

  receive(rxbuf, 3);

  this->pickle();

  return state;
}
