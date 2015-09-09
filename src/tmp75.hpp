#ifndef TMP75_HPP_
#define TMP75_HPP_

#include "i2c_sensor.hpp"

#define tmp75addr 0b1001000

/* buffers depth */
#define TMP75_RX_DEPTH 4
#define TMP75_TX_DEPTH 4


class TMP75: private I2CSensor {
public:
  TMP75(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(void);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  void stop(void);
  void sleep(void);

private:
  void pickle(void);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint8_t rxbuf[TMP75_RX_DEPTH];
  uint8_t txbuf[TMP75_TX_DEPTH];
};


#endif /* TMP75_HPP_ */
