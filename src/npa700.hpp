#ifndef NPA700_HPP_
#define NPA700_HPP_

#include "i2c_sensor.hpp"

#define npa700addr 0b00101000

/* buffers depth */
#define NPA700_RX_DEPTH 4
#define NPA700_TX_DEPTH 4


class NPA700: private I2CSensor {
public:
  NPA700(I2CDriver *i2cdp, i2caddr_t addr);
  sensor_state_t get(void);
  sensor_state_t start(void);
  sensor_state_t wakeup(void);
  void stop(void);
  void sleep(void);

private:
  void pickle(void);
  bool hw_init_full(void);
  bool hw_init_fast(void);
  uint8_t rxbuf[NPA700_RX_DEPTH];
  uint8_t txbuf[NPA700_TX_DEPTH];
};

#endif /* NPA700_HPP_ */
