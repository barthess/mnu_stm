#ifndef IDT5_HPP_
#define IDT5_HPP_

#define idt5addr          (0xD4 >> 1)

/* buffers depth */
#define IDT5_START_ADDR   0x10
#define IDT5_END_ADDR     0x69
#define IDT5_BUF_LEN      (IDT5_END_ADDR - IDT5_START_ADDR)

class IDT5 {
public:
  IDT5(I2CDriver *i2cdp, i2caddr_t addr);
  void start(void);
  void stop(void);

  void write_prediv(const char *arg);
  void write_vcoctrl(const char *arg);
  void write_pllint(const char *arg);
  void write_pllfraq(const char *arg);
  void write_fod(const char *arg);

private:
  void device_write_reg(uint8_t reg_addr, const uint8_t *data, size_t datalen);
  uint8_t rxbuf[IDT5_BUF_LEN];
  I2CDriver *i2cdp;
  i2caddr_t addr;
};

thread_t* idt5_clicmd(int argc, const char * const * argv, SerialDriver *sdp);

#endif /* IDT5_HPP_ */
