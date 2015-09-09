#ifndef FRAM_HPP_
#define FRAM_HPP_

#include "i2c_sensor.hpp"

#define FRAM0_I2C_ADDR           0b1010000
#define FRAM1_I2C_ADDR           0b1010001

/* buffers depth */
#define FRAM_RX_DEPTH    16
#define FRAM_TX_DEPTH    18


class fram: private I2CSensor {
public:
  fram(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(void);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  void stop(void);
  void sleep(void);

private:
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint8_t rxbuf[FRAM_RX_DEPTH];
  uint8_t txbuf[FRAM_TX_DEPTH];
};


#endif /* FRAM_HPP_ */
