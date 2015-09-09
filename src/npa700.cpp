#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "npa700.hpp"

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
void NPA700::pickle(void) {
  temperature = rxbuf[2];
  temperature = -50 + (temperature * 200) / 256;

  pressure = 0;
  pressure |= (rxbuf[0] << 8) | rxbuf[1];
}

/**
 *
 */
NPA700::NPA700(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  state = SENSOR_STATE_STOP;
}

bool NPA700::hw_init_full(void){return OSAL_SUCCESS;}
bool NPA700::hw_init_fast(void){return OSAL_SUCCESS;}

/**
 *
 */
sensor_state_t NPA700::start(void){
  state = SENSOR_STATE_READY;
  return state;
}

void NPA700::stop(void){
  state = SENSOR_STATE_STOP;
}

sensor_state_t NPA700::wakeup(void) {
  state = SENSOR_STATE_READY;
  return state;
}

void NPA700::sleep(void) {
  state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t NPA700::get(void){
  osalDbgCheck(state == SENSOR_STATE_READY);

  receive(rxbuf, 3);

  this->pickle();

  return state;
}
