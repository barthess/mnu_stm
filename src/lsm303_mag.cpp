#include "main.h"
#include "lsm303_mag.hpp"

#include "pack_unpack.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/* Swaps Y Z axis for specially LSM303DLHC. LSM303DLH and HMC5843 does not need
   this hack */
#define LSM_SWAP_Y_Z          TRUE

#define LSM_REG_CRA           0x00
#define LSM_REG_CRB           0x01
#define LSM_REG_MR            0x02
#define LSM_REG_MAG_OUT       0x03
#define LSM_REG_ID            0x0A
#define LSM_REG_TEMP_OUT      0x31
#define GAIN_BITS_SHIFT       5

/**
 * @brief   Magnetometer gain (LSB/Gauss)
 */
typedef enum {
  LSM_MAG_GAIN_1370 = 0,
  LSM_MAG_GAIN_1090,
  LSM_MAG_GAIN_820,
  LSM_MAG_GAIN_660,
  LSM_MAG_GAIN_440,
  LSM_MAG_GAIN_390,
  LSM_MAG_GAIN_330,
  LSM_MAG_GAIN_230
} mag_sens_t;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static const float mag_sens_array[8] = {
    1.0f / 1370,
    1.0f / 1090,
    1.0f / 820,
    1.0f / 660,
    1.0f / 440,
    1.0f / 390,
    1.0f / 330,
    1.0f / 230
};

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 * @brief   convert parrots to Gauss.
 */
float LSM303_mag::mag_sens(void) {
  return mag_sens_array[gain_current];
}

/**
 *
 */
void LSM303_mag::thermo_comp(float *result){
  (void)result;
}

/**
 *
 */
void LSM303_mag::iron_comp(float *result){
  (void)result;
}

/**
 *
 */
void LSM303_mag::pickle(float *result, int16_t *result_raw) {

  int16_t raw[3];
  float sens = this->mag_sens();

  for (size_t i=0; i<3; i++)
    raw[i] = static_cast<int16_t>(pack8to16be(&rxbuf[i*2]));

  #if LSM_SWAP_Y_Z
  float tmp = raw[2];
  raw[2] = raw[1];
  raw[1] = tmp;
  #endif

  /* */
  for (size_t i=0; i<3; i++) {
    result[i] = sens * raw[i];
    result_raw[i] = raw[i];
  }
  thermo_comp(result);
  iron_comp(result);
}

/**
 *
 */
bool LSM303_mag::hw_init_fast(void){
  return hw_init_full(); /* unimplemented */
}

/**
 *
 */
bool LSM303_mag::hw_init_full(void){

  msg_t i2cret = MSG_RESET;

  txbuf[0] = LSM_REG_ID;
  i2cret = transmit(txbuf, 1, rxbuf, 3);
  if (MSG_OK != i2cret)
    return OSAL_FAILED;
  if(0 != memcmp("H43", rxbuf, 3)) // incorrect ID
    return OSAL_FAILED;

  txbuf[0] = LSM_REG_CRA;
  //txbuf[1] = 0b10011100; /* enable thermometer and set maximum output rate */
  txbuf[1] = 0;
  /* Set gain. 001 is documented for LSM303 and 000 is for HMC5883.
   * 000 looks working for LSM303 too - lets use it. */
  txbuf[2] = gain_current << GAIN_BITS_SHIFT;
  /* single conversion mode */
  txbuf[3] = 0b00000001;

  i2cret = transmit(txbuf, 4, NULL, 0);
  if (MSG_OK != i2cret)
    return OSAL_FAILED;

  return OSAL_SUCCESS;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
LSM303_mag::LSM303_mag(I2CDriver *i2cdp, i2caddr_t addr):
I2CSensor(i2cdp, addr),
sample_cnt(0)
{
  state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t LSM303_mag::get(marg_data_t &result) {
  (void)result;
  return this->state;
}

/**
 *
 */
sensor_state_t LSM303_mag::start(void){

  msg_t status = MSG_RESET;
  status = hw_init_full();
  osalDbgCheck(MSG_OK == status);

  return SENSOR_STATE_READY;
}

/**
 *
 */
void LSM303_mag::stop(void){

  this->state = SENSOR_STATE_STOP;
}

/**
 *
 */
void LSM303_mag::sleep(void) {
  this->state = SENSOR_STATE_SLEEP;
}

/**
 *
 */
sensor_state_t LSM303_mag::wakeup(void) {
  this->state = SENSOR_STATE_READY;
  return this->state;
}
