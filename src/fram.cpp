#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "fram.hpp"

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
fram::fram(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  state = SENSOR_STATE_STOP;
}

bool fram::hw_init_full(void){return OSAL_SUCCESS;}
bool fram::hw_init_fast(void){return OSAL_SUCCESS;}

/**
 *
 */
sensor_state_t fram::start(void){
  state = SENSOR_STATE_READY;
  return state;
}

void fram::stop(void){
  state = SENSOR_STATE_STOP;
}

sensor_state_t fram::wakeup(void) {
  state = SENSOR_STATE_READY;
  return state;
}

void fram::sleep(void) {
  state = SENSOR_STATE_SLEEP;
}

static uint8_t  pattern = 0;
static const uint16_t address = 512;

sensor_state_t fram::get(void){
  msg_t status = MSG_RESET;
  int cmp_status = -1;

  osalDbgCheck(state == SENSOR_STATE_READY);

  /* first write pattern */
  memset(txbuf, pattern, sizeof(txbuf));
  memcpy(txbuf, &address, sizeof(address));
  status = transmit(txbuf, sizeof(txbuf), nullptr, 0);
  osalDbgCheck(MSG_OK == status);
  osalThreadSleepMilliseconds(10);

  /* read back and compare */
  status = transmit(txbuf, 2, rxbuf, sizeof(rxbuf));
  osalDbgCheck(MSG_OK == status);
  cmp_status = memcmp(rxbuf, &txbuf[2], sizeof(rxbuf));
  osalDbgCheck(0 == cmp_status);

  /* change pattern */
  pattern++;

  return state;
}




