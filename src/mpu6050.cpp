#include "main.h"

#include "mpu6050.hpp"
#include "pack_unpack.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/* offsets in received data array */
#define MPU_ACCEL_OFFSET        1
#define MPU_TEMP_OFFSET         7
#define MPU_GYRO_OFFSET         9

/* registers address */
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
  #define FIFO_DATA_BITS        0b1111000 /* accel and gyro */
  #define FIFO_MODE             (1 << 6)
#define MPUREG_INT_PIN_CFG      0x37
  #define I2C_BYPASS_EN         (1 << 1)
  #define INT_RD_CLEAR          (1 << 4) /* clear int flag in register on any read operation */
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A /* 1 status bite and 14 bytes of data */
#define MPUREG_TEMP_OUT         0x41  /* MSB. Next byte is LSB */
#define MPUREG_USER_CTRL        0x6A
  #define FIFO_EN               (1 << 6)
  #define I2C_MST_EN            (1 << 5) /* clear this bit to use I2C bypass mode */
  #define FIFO_RST            (1 << 2)
#define MPUREG_PWR_MGMT1        0x6B
  #define DEVICE_RESET          (1 << 7)
  #define DEVICE_SLEEP          (1 << 6)
#define MPUREG_PWR_MGMT2        0x6C
#define MPUREG_FIFO_CNT         0x72 /* MSB. Next byte is LSB */
#define MPUREG_FIFO_DATA        0x74
#define MPUREG_WHO_AM_I         0x75
  #define WHO_AM_I_VAL          0X68

/**
 * @brief   Gyro full scale in deg/s
 */
typedef enum {
  MPU_GYRO_FULL_SCALE_250 = 0,
  MPU_GYRO_FULL_SCALE_500,
  MPU_GYRO_FULL_SCALE_1000,
  MPU_GYRO_FULL_SCALE_2000
} gyro_sens_t;

/**
 * @brief   Accel full scale in g
 */
typedef enum {
  MPU_ACC_FULL_SCALE_2 = 0,
  MPU_ACC_FULL_SCALE_4,
  MPU_ACC_FULL_SCALE_8,
  MPU_ACC_FULL_SCALE_16
} acc_sens_t;

/* reset fifo if it contains such amount of bytes */
#define FIFO_RESET_THRESHOLD  1024

/* how many bytes in single fifo sample */
#define BYTES_IN_SAMPLE       12

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
void MPU6050::pickle_temp(float *result) {
  uint8_t *b = &rxbuf[MPU_TEMP_OFFSET];
  result[0] = static_cast<int16_t>(pack8to16be(b));
  result[0] /= 340;
  result[0] += 36.53f;
}

/**
 *
 */
void MPU6050::gyro_thermo_comp(float *result) {
  (void)result;
}

/**
 *
 */
void MPU6050::acc_egg_comp(float *result) {
  (void)result;
}

/**
 *
 */
static void toggle_endiannes16(uint8_t *data, const size_t len) {

  osalDbgCheck(0 == (len % 2));
  uint8_t tmp;

  for (size_t i=0; i<len; i+=2){
    tmp = data[i];
    data[i] = data[i+1];
    data[i+1] = tmp;
  }
}

/**
 *
 */
void MPU6050::pickle_gyr(float *result) {

  int16_t raw[3];
  uint8_t *b = &rxbuf[MPU_GYRO_OFFSET];
  float sens = this->gyr_sens();

  toggle_endiannes16(b, 6);
  memcpy(raw, b, sizeof(raw));

  for (size_t i=0; i<3; i++) {
    gyr_raw_data[i] = raw[i];
    result[i] = sens * raw[i];
  }

  gyro_thermo_comp(result);
}

/**
 *
 */
void MPU6050::pickle_acc(float *result) {

  int16_t raw[3];
  uint8_t *b = &rxbuf[MPU_ACCEL_OFFSET];
  float sens = this->acc_sens();

  raw[0] = static_cast<int16_t>(pack8to16be(&b[0]));
  raw[1] = static_cast<int16_t>(pack8to16be(&b[2]));
  raw[2] = static_cast<int16_t>(pack8to16be(&b[4]));

  for (size_t i=0; i<3; i++) {
    acc_raw_data[i] = raw[i];
    result[i] = sens * raw[i];
  }

  acc_egg_comp(result);
}

/**
 *
 */
bool MPU6050::hw_init_fast(void) {
  return hw_init_full(); /* unimplemented */
}

/**
 *
 */
msg_t MPU6050::set_gyr_fs(uint8_t fs) {
  txbuf[0] = MPUREG_GYRO_CONFIG;
  txbuf[1] = fs << 3;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t MPU6050::set_acc_fs(uint8_t fs) {
  txbuf[0] = MPUREG_ACCEL_CONFIG;
  txbuf[1] = fs << 3;
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
msg_t MPU6050::set_dlpf_smplrt(uint8_t lpf, uint8_t smplrt) {

  msg_t ret1 = MSG_OK;
  msg_t ret2 = MSG_OK;

  /* */
  txbuf[0] = MPUREG_SMPLRT_DIV;
  if (lpf > 0){
    /* sample rate. If (LPF > 0): (1000 / (val + 1))
     *                      else: (8000 / (val + 1)) */
    txbuf[1] = smplrt - 1; /* val */
    /*    Bandwidth   Delay
    DLPF      (Hz)    (ms)
    1         188     1.9
    2         98      2.8
    3         42      4.8
    4         20      8.3
    5         10      13.4
    6         5       18.6
    7   reserved*/
    txbuf[2] = lpf | FIFO_MODE; /* LPF */
  }
  else{
    txbuf[1] = 7; /* 8000 / (val + 1) */
    txbuf[2] = 0 | FIFO_MODE; /* LPF */
  }
  if (MSG_OK != transmit(txbuf, 3, NULL, 0))
    return MSG_RESET;

  /* FIFO settings */
  if (lpf > 0){
    txbuf[0] = MPUREG_FIFO_EN;
    txbuf[1] = 0;
    ret1 = transmit(txbuf, 2, NULL, 0);

    txbuf[0] = MPUREG_USER_CTRL;
    txbuf[1] = FIFO_RST;
    ret2 = transmit(txbuf, 2, NULL, 0);
  }
  else {
    txbuf[0] = MPUREG_FIFO_EN;
    txbuf[1] = FIFO_DATA_BITS;
    ret1 = transmit(txbuf, 2, NULL, 0);

    txbuf[0] = MPUREG_USER_CTRL;
    txbuf[1] = FIFO_EN;
    ret2 = transmit(txbuf, 2, NULL, 0);
  }

  /**/
  if ((MSG_OK == ret1) && (MSG_OK == ret2))
    return MSG_OK;
  else
    return MSG_RESET;
}

/**
 *
 */
msg_t MPU6050::soft_reset(void) {
  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = DEVICE_RESET; /* soft reset */
  return transmit(txbuf, 2, NULL, 0);
}

/**
 *
 */
bool MPU6050::hw_init_full(void){

  msg_t i2c_status = MSG_RESET;

  /* there is not need to call soft reset here because of POR sensor reset,
     just wait timeout. */
  osalThreadSleepMilliseconds(30);

  txbuf[0] = MPUREG_WHO_AM_I;
  i2c_status = transmit(txbuf, 1, rxbuf, 1);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  if(WHO_AM_I_VAL != rxbuf[0]) // MPU6050 wrong id
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(1);

  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = 1; /* select X gyro as clock source */
  i2c_status = transmit(txbuf, 2, NULL, 0);
  if (MSG_OK != i2c_status)
    return OSAL_FAILED;
  osalThreadSleepMilliseconds(5);

  return OSAL_SUCCESS;
}

/**
 *
 */
msg_t MPU6050::param_update(void) {

  return MSG_OK;
}

/**
 *
 */
msg_t MPU6050::acquire_simple(float *acc, float *gyr) {

  msg_t ret = MSG_RESET;

  txbuf[0] = MPUREG_INT_STATUS;
  ret = transmit(txbuf, 1, rxbuf, sizeof(rxbuf));

  this->set_lock();

  pickle_temp(&temperature);
  if (nullptr != gyr)
    pickle_gyr(gyr);
  if (nullptr != acc)
    pickle_acc(acc);
  fifo_remainder = 0;

  this->release_lock();

  return ret;
}

/**
 *
 */
time_measurement_t fir_tmu;

void MPU6050::pickle_fifo(float *acc, float *gyr, const size_t sample_cnt) {

  float sens;
  const size_t acc_fifo_offset = 0;
  const size_t gyr_fifo_offset = 3;

  for (size_t i=0; i<3; i++) {
    acc_raw_data[i] = rxbuf_fifo[acc_fifo_offset + i];
    gyr_raw_data[i] = rxbuf_fifo[gyr_fifo_offset + i];
  }

  if (sample_cnt == 10)
    chTMStartMeasurementX(&fir_tmu);
  for (size_t n=0; n<sample_cnt; n++) {
    for (size_t i=0; i<3; i++){
      size_t shift = n * BYTES_IN_SAMPLE / 2 + i;
      acc[i] = rxbuf_fifo[shift + acc_fifo_offset];
      gyr[i] = rxbuf_fifo[shift + gyr_fifo_offset];
    }
  }
  if (sample_cnt == 10)
    chTMStopMeasurementX(&fir_tmu);

  /* acc */
  sens = this->acc_sens();
  acc[0] *= sens;
  acc[1] *= sens;
  acc[2] *= sens;
  acc_egg_comp(acc);

  /* gyr */
  sens = this->gyr_sens();
  gyr[0] *= sens;
  gyr[1] *= sens;
  gyr[2] *= sens;
  gyro_thermo_comp(gyr);
}

/**
 *
 */
msg_t MPU6050::acquire_fifo(float *acc, float *gyr) {

  msg_t ret = MSG_RESET;
  size_t recvd;

  txbuf[0] = MPUREG_FIFO_CNT;
  ret = transmit(txbuf, 1, rxbuf, 2);

  recvd = pack8to16be(rxbuf);
  if (recvd >= FIFO_RESET_THRESHOLD) {
    txbuf[0] = MPUREG_USER_CTRL;
    txbuf[1] = FIFO_RST | FIFO_EN;
    return transmit(txbuf, 2, NULL, 0);
  }
  else {
    recvd = sizeof(rxbuf_fifo);
    recvd = (recvd / BYTES_IN_SAMPLE) * BYTES_IN_SAMPLE;

    txbuf[0] = MPUREG_FIFO_DATA;
    ret = transmit(txbuf, 1, (uint8_t*)rxbuf_fifo, recvd);
    toggle_endiannes16((uint8_t*)rxbuf_fifo, recvd);

    this->set_lock();
    pickle_fifo(acc, gyr, recvd/BYTES_IN_SAMPLE);
    this->release_lock();
  }

  return ret;
}

/**
 *
 */
void MPU6050::acquire_data(void) {

  msg_t ret1 = MSG_RESET;
  msg_t ret2 = MSG_RESET;

  if (SENSOR_STATE_READY == this->state) {
    ret2 = param_update();

    ret1 = acquire_fifo(acc_data, gyr_data);

    if ((MSG_OK != ret1) || (MSG_OK != ret2))
      this->state = SENSOR_STATE_DEAD;
  }
}

/**
 *
 */
void MPU6050::set_lock(void) {
  this->protect_sem.wait();
}

/**
 *
 */
void MPU6050::release_lock(void) {
  this->protect_sem.signal();
}


/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
MPU6050::MPU6050(I2CDriver *i2cdp, i2caddr_t addr) :
I2CSensor(i2cdp, addr),
protect_sem(false),
data_ready_sem(true)
{
  state = SENSOR_STATE_STOP;
  chTMObjectInit(&fir_tmu);
}

/**
 *
 */
sensor_state_t MPU6050::start(void) {

  /* init hardware */
  bool init_status = OSAL_FAILED;
  init_status = hw_init_full();
  osalDbgCheck(OSAL_SUCCESS == init_status);

  this->state = SENSOR_STATE_READY;
  return this->state;
}

/**
 * Just fire soft reset signal. After completing of reset sequence
 * device will be in sleep state.
 */
void MPU6050::stop(void) {

  this->state = SENSOR_STATE_STOP;
}

/**
 *
 */
sensor_state_t MPU6050::get(marg_data_t &result) {

  (void)result;
  return this->state;
}

/**
 *
 */
msg_t MPU6050::waitData(systime_t timeout) {
  return data_ready_sem.wait(timeout);
}

/**
 *
 */
void MPU6050::sleep(void) {
  uint8_t b;

  if (this->state == SENSOR_STATE_SLEEP)
    return;

  osalDbgAssert(this->state == SENSOR_STATE_READY, "Invalid state");

  /* stop worker thread */
  chThdTerminate(worker);
  chThdWait(worker);
  worker = nullptr;

  /* suspend sensor */
  txbuf[0] = MPUREG_PWR_MGMT1;
  if (MSG_OK != transmit(txbuf, 1, &b, 1))
    goto ERROR;
  txbuf[0] = MPUREG_PWR_MGMT1;
  txbuf[1] = b | DEVICE_SLEEP;
  if (MSG_OK != transmit(txbuf, 2, NULL, 0))
    goto ERROR;

  this->state = SENSOR_STATE_SLEEP;
  return;
ERROR:
  this->state = SENSOR_STATE_DEAD;
}

/**
 *
 */
sensor_state_t MPU6050::wakeup(void) {
  this->state = SENSOR_STATE_READY;
  return this->state;
}

/**
 *
 */
float MPU6050::dt(void) {
  return 0.01;
}

