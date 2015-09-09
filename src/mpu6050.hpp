#ifndef MPU6050_HPP_
#define MPU6050_HPP_

#include "i2c_sensor.hpp"
#include "marg_data.hpp"

#define mpu6050addr         0b1101000

#define MPU_RX_DEPTH        16  /* 1 status byte + 14 bytes of data + 1 padding */
#define MPU_TX_DEPTH        4

/**
 *
 */
class MPU6050: protected I2CSensor {
public:
  MPU6050(I2CDriver *i2cdp, i2caddr_t addr);
  msg_t waitData(systime_t timeout);
  sensor_state_t get(marg_data_t &result);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  sensor_state_t get_state(void) {return this->state;}
  void stop(void);
  void sleep(void);

private:
  friend THD_FUNCTION(Mpu6050Thread, arg);
  float dt(void);
  void acquire_data(void);
  msg_t soft_reset(void);
  msg_t acquire_simple(float *acc, float *gyr);
  msg_t acquire_fifo(float *acc, float *gyr);
  void set_lock(void);
  void release_lock(void);
  msg_t set_gyr_fs(uint8_t fs);
  msg_t set_acc_fs(uint8_t fs);
  msg_t set_dlpf_smplrt(uint8_t lpf, uint8_t smplrt);
  msg_t param_update(void);
  float gyr_sens(void);
  float acc_sens(void);
  void gyro_thermo_comp(float *result);
  void acc_egg_comp(float *result);
  void pickle_gyr(float *result);
  void pickle_fifo(float *acc, float *gyr, const size_t sample_cnt);
  void pickle_acc(float *result);
  void pickle_temp(float *result);
  bool hw_init_full(void);
  bool hw_init_fast(void);

  float temperature;
  chibios_rt::BinarySemaphore protect_sem;
  chibios_rt::BinarySemaphore data_ready_sem;
  thread_t *worker;
  float acc_data[3];
  float gyr_data[3];
  int16_t acc_raw_data[3];
  int16_t gyr_raw_data[3];

  uint16_t fifo_remainder = 0;
  int16_t rxbuf_fifo[960 / 2];
  uint8_t rxbuf[MPU_RX_DEPTH];
  uint8_t txbuf[MPU_TX_DEPTH];

  static size_t  isr_count;
  static uint8_t isr_dlpf;
  static uint8_t isr_smplrtdiv;
  static chibios_rt::BinarySemaphore isr_sem;
};

#endif /* MPU6050_HPP_ */
