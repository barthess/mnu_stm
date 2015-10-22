#include "main.h"
#include "mul_test.hpp"

using namespace chibios_rt;

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
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

double op1 = 3.71;
double op2 = 1.3;
volatile double result;
const double step = 0.13;
size_t wait_cycles = 0;
double delta;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void fpga_mul_test(MtrxMul *mulp) {



  volatile double *ptr = (double *)memtest_struct.start;

  ptr[1] = op1;
  ptr[2] = op2;

  // wait until data flushed from internal FSMC's fifo
  while (! FSMCDataFlushed());

  // notify FPGA
  palSetPad(GPIOB, GPIOB_FPGA_IO2);

  // wait ready pin
  while (PAL_HIGH != palReadPad(GPIOB, GPIOB_FPGA_IO1));

  // collect result
  result = ptr[3];

  // reset FPGA state machine to IDLE state
  palClearPad(GPIOB, GPIOB_FPGA_IO2);

  // wait FPGA reset
  while (PAL_LOW != palReadPad(GPIOB, GPIOB_FPGA_IO1));

  delta = fabs(result - (op1*op2));
  if (delta > (double)0.0001)
    red_led_on();

  wait_cycles = 0;
  op1 += step;
  op2 += step + (double)0.0171;
  if (op1 > 100)
    op1 = -100;
  if (op2 > 100)
    op2 = -100;
}
