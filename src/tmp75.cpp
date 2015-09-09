#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "tmp75.hpp"

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
static int32_t millideg;
static int16_t temp_tmp75;
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
void TMP75::pickle(void) {
  temp_tmp75 = 0;
  temp_tmp75 |= (rxbuf[0] << 8) | rxbuf[1];

  millideg = ((int32_t)temp_tmp75 * 100) / 16;

  osalDbgCheck((millideg > 10000) && (millideg < 40000));
}

/**
 *
 */
TMP75::TMP75(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr)
{
  state = SENSOR_STATE_STOP;
}

bool TMP75::hw_init_full(void){return OSAL_SUCCESS;}
bool TMP75::hw_init_fast(void){return OSAL_SUCCESS;}

/**
 *
 */
sensor_state_t TMP75::start(void){
  state = SENSOR_STATE_READY;
  return state;
}

void TMP75::stop(void){
  state = SENSOR_STATE_STOP;
}

sensor_state_t TMP75::wakeup(void) {
  state = SENSOR_STATE_READY;
  return state;
}

void TMP75::sleep(void) {
  state = SENSOR_STATE_SLEEP;
}

/*
Accessing a particular register on the TMP175 and TMP75
is accomplished by writing the appropriate value to the
Pointer Register. The value for the Pointer Register is the
first byte transferred after the slave address byte with the
R/W bit LOW. Every write operation to the TMP175 and
TMP75 requires a value for the Pointer Register.

When reading from the TMP175 and TMP75, the last value
stored in the Pointer Register by a write operation is used
to determine which register is read by a read operation. To
change the register pointer for a read operation, a new
value must be written to the Pointer Register. This is
accomplished by issuing a slave address byte with the
R/W bit LOW, followed by the Pointer Register Byte. No
additional data is required.
*/
sensor_state_t TMP75::get(void){
  osalDbgCheck(state == SENSOR_STATE_READY);

  receive(rxbuf, 2);

  this->pickle();

  return state;
}
