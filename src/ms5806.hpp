#ifndef MS5806_HPP_
#define MS5806_HPP_

#include "i2c_sensor.hpp"

#define ms5806addr 0b1110111

/* buffers depth */
#define MS5806_RX_DEPTH   4
#define MS5806_TX_DEPTH   4

#define MS5806_CAL_WORDS  8

class MS5806: private I2CSensor {
public:
  MS5806(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(void);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  void stop(void);
  void sleep(void);

private:
  void pickle(void);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint8_t rxbuf[MS5806_RX_DEPTH];
  uint8_t txbuf[MS5806_TX_DEPTH];
  uint16_t C[MS5806_CAL_WORDS];
};

#endif /* MS5806_HPP_ */
