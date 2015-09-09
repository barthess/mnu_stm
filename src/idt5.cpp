#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "idt5.hpp"
#include "cli.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define IDT5_REF_DIV        0x15
#define IDT5_VCO_CONTROL    0x16

#define IDT5_FEEDBAK_INT1   0x17
#define IDT5_FEEDBAK_INT2   0x18

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
IDT5 idt5(&I2CD_NVRAM, idt5addr);

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
IDT5::IDT5(I2CDriver *i2cdp, i2caddr_t addr):
i2cdp(i2cdp), addr(addr)
{
  return;
}

/**
 *
 */
uint8_t fraq = 0;
uint8_t fraq_increment = 16;

#define OUT_CFG_DEFAULT     0b10111011; /* CMOSD 3.3v */
#define OUT_CFG_XILINX      0b01100011; /* LVDS 1.8v */

void IDT5::start(void) {
  uint8_t reg[16];
  msg_t status = MSG_RESET;
  const systime_t tmo = MS2ST(100);
  uint8_t a;

  i2cAcquireBus(this->i2cdp);
  reg[0] = IDT5_START_ADDR;
  status = i2cMasterTransmitTimeout(i2cdp, addr, reg, 1, rxbuf, sizeof(rxbuf), tmo);

  /* set multiplier to 121 */
  reg[0] = 0x17;
  reg[1] = 0x7;
  reg[2] = 0x9 << 4;
  status = i2cMasterTransmitTimeout(i2cdp, addr, reg, 3, nullptr, 0, tmo);

  /* configure outputs */
  reg[0] = 0x60;
  reg[1] = OUT_CFG_XILINX;
  reg[2] = 1;
  reg[3] = OUT_CFG_XILINX;
  reg[4] = 1;
  reg[5] = OUT_CFG_XILINX;
  reg[6] = 1;
  reg[7] = OUT_CFG_XILINX;
  reg[8] = 1;

  status = i2cMasterTransmitTimeout(i2cdp, addr, reg, 9, nullptr, 0, tmo);
  osalThreadSleepMilliseconds(50);
  osalDbgCheck(MSG_OK == status);

  /* set divider to 10 all outputs */
  reg[1] = 5 << 4;
  a = 0x2E;
  for (size_t i=0; i<4; i++) {
    reg[0] = a + 0x10 * i;
    status = i2cMasterTransmitTimeout(i2cdp, addr, reg, 2, nullptr, 0, tmo);
    osalDbgCheck(MSG_OK == status);
    osalThreadSleepMilliseconds(50);
  }

  reg[1] = 0b10000001; /* release reset on output */
  a = 0x31; /* starting from 0x31 because 0x21 already configured correctly */
  for (size_t i=0; i<3; i++) {
    reg[0] = a + 0x10 * i;
    status = i2cMasterTransmitTimeout(i2cdp, addr, reg, 2, nullptr, 0, tmo);
    osalDbgCheck(MSG_OK == status);
    osalThreadSleepMilliseconds(50);
  }

  memset(rxbuf, 0x55, sizeof(rxbuf));
  reg[0] = IDT5_START_ADDR;
  status = i2cMasterTransmitTimeout(i2cdp, addr, reg, 1, rxbuf, sizeof(rxbuf), tmo);

  i2cReleaseBus(this->i2cdp);
}

/**
 *
 */
void IDT5::stop(void){

}

/**
 *
 */
static void print_all(void) {
  cli_println("All registers:");
}

/**
 *
 */
void IDT5::device_write_reg(uint8_t reg_addr, const uint8_t *data, size_t datalen) {
  uint8_t txbuf[16];
  msg_t status = MSG_RESET;
  systime_t tmo = MS2ST(100);

  txbuf[0] = reg_addr;
  for (size_t i=1; i<16; i++)
    txbuf[i] = data[i-1];

  status = i2cMasterTransmitTimeout(i2cdp, addr, txbuf, datalen+1, nullptr, 0, tmo);
  if (MSG_OK != status) {
    cli_println("ERROR: i2c bus fault");
    return;
  }
}

/**
 *
 */
void IDT5::write_prediv(const char *arg) {
  uint32_t val = atoi(arg);
  uint8_t data;

  if (val > 0xFF) {
    cli_println("ERROR: incorrect value");
    return;
  }
  else {
    data = val;
    device_write_reg(0x15, &data, 1);
  }
}

/**
 *
 */
void IDT5::write_vcoctrl(const char *arg) {
  uint32_t val = atoi(arg);
  uint8_t data;

  if (val > 0xFF) {
    cli_println("ERROR: incorrect value");
    return;
  }
  else {
    data = val;
    device_write_reg(0x16, &data, 1);
  }
}

/**
 *
 */
void IDT5::write_pllint(const char *arg) {
  uint32_t val = atoi(arg);
  uint8_t data[2] = {0xFF, 0xFF};

  if (val > 4095) {
    cli_println("ERROR: incorrect value");
    return;
  }
  else {
    data[0] = val >> 4;
    data[1] = (val & 0xF) << 4;
    device_write_reg(0x17, data, 2);
  }
}

/**
 *
 */
void IDT5::write_pllfraq(const char *arg) {
  uint32_t val = atoi(arg);
  uint8_t data[3];

  if (val > 16777215) {
    cli_println("ERROR: incorrect value");
    return;
  }
  else {
    data[0] = (val >> 16) & 0xFF;
    data[1] = (val >> 8) & 0xFF;
    data[2] = val & 0xFF;
    device_write_reg(0x19, data, 3);
  }
}

/**
 *
 */
void IDT5::write_fod(const char *arg) {

  uint32_t val = atoi(arg);
  uint8_t data[2];
  uint8_t a;

  /* set divider to all outputs */
  data[0] = val >> 4;
  data[1] = (val & 0xF) << 4;

  a = 0x2D;
  for (size_t i=0; i<4; i++)
    device_write_reg(a + 0x10 * i, data, 2);
}

/**
 *
 */
thread_t* idt5_clicmd(int argc, const char * const * argv, SerialDriver *sdp) {
  (void)sdp;

  /* no arguments */
  if (argc == 0)
    print_all();

  /* one argument */
  else if (argc == 1) {
    cli_println("ERROR: too few arguments.");
  }

  /* two arguments */
  else if (argc == 2) {
    if      (strcmp(*argv, "prediv") == 0)
      idt5.write_prediv(argv[1]);
    else if (strcmp(*argv, "vcoctrl") == 0)
      idt5.write_vcoctrl(argv[1]);
    else if (strcmp(*argv, "pllint") == 0)
      idt5.write_pllint(argv[1]);
    else if (strcmp(*argv, "pllfraq") == 0)
      idt5.write_pllfraq(argv[1]);
    else if (strcmp(*argv, "fod") == 0)
      idt5.write_fod(argv[1]);
    else {
      cli_println("ERROR: unknown cmd.");
    }
  }
  else{
    cli_println("ERROR: bad arguments.");
  }

  /* stub */
  return NULL;
}

