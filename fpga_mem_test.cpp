#include <cmath>

#include "main.h"
#include "pads.h"

#include "memtest.h"
#include "fpga_mul.h"
#include "fpga_mem_test.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define BRAM_DEPTH    (65536 * 1)
#define BRAM_WIDTH    2 // width in bytes

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
static void mem_error_cb(memtest_t *memp, testtype type, size_t index,
                         size_t width, uint32_t got, uint32_t expect);
/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static const size_t sram_size = BRAM_DEPTH * BRAM_WIDTH;

static memtest_t memtest_struct = {
    nullptr,
    sram_size,
    MEMTEST_WIDTH_64 | MEMTEST_WIDTH_32 | MEMTEST_WIDTH_16,
    //MEMTEST_WIDTH_8,
    mem_error_cb
};

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 *
 */
void FPGAMemFill(bool enable) {
  if (enable)
    palSetPad(GPIOF, GPIOF_FPGA_IO6);
  else
    palClearPad(GPIOF, GPIOF_FPGA_IO6);
}

/*
 *
 */
static void mem_error_cb(memtest_t *memp, testtype type, size_t index,
                          size_t width, uint32_t got, uint32_t expect) {
  (void)memp;
  (void)type;
  (void)index;
  (void)width;
  (void)got;
  (void)expect;

  green_led_off();
  orange_led_off();
  red_led_on();
  osalSysHalt("Memory broken");
}

/*
 *
 */
static void memtest(memtest_t *testp) {
  memtest_run(testp, MEMTEST_RUN_ALL);
}

/*
 *
 */
static void running_lights(memtest_t *testp) {

  FPGAMemFill(false);
  osalThreadSleep(1);

  memtest(testp);
  green_led_toggle();
  memtest(testp);
  red_led_toggle();
  memtest(testp);
  orange_led_toggle();
}

/*
 *
 */
static void zero_addr_match(memtest_t *testp) {
  uint16_t *mem_array = (uint16_t *)testp->start;

  FPGAMemFill(false);
  osalThreadSleep(1);

  mem_array[0] = 0x55AA;
  osalThreadSleep(1);
  osalDbgCheck(true == FPGAMulReady());

  mem_array[0] = 0x55AB;
  osalThreadSleep(1);
  osalDbgCheck(false == FPGAMulReady());
}

/*
 *
 */
static void fpga_own_addr(memtest_t *testp) {
  uint16_t *mem_array = (uint16_t *)testp->start;
  uint16_t read;

  FPGAMemFill(true);
  osalThreadSleepMilliseconds(10); // wait until FPGA fills all BRAM array

  for (size_t i=0; i<BRAM_DEPTH; i++) {
    read = mem_array[i];
    osalDbgCheck(i == read);
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void fpga_memtest(FPGADriver *fpgap, size_t turns) {

  osalDbgCheck(fpgap->state == FPGA_READY);

  memtest_struct.start = fpgap->memspace;

  while (turns--) {
    /* general memtest without FPGA participating */
    running_lights(&memtest_struct);

    /* FPGA assisted tests */
    zero_addr_match(&memtest_struct);
    fpga_own_addr(&memtest_struct);
  }

  green_led_off();
  red_led_off();
  orange_led_off();
}



typedef uint16_t memword;
volatile memword pattern = 0xFF00;
volatile memword devnull = 0;
void fpga_memtest_oscillo_probe(FPGADriver *fpgap) {
  volatile memword *ptr = (memword *)fpgap->memspace;

  FPGAMemFill(false);
  osalThreadSleep(1);

  while (true) {
    osalSysLock();
    ptr[0] = pattern;
    ptr[1] = ~pattern;
    port_rt_get_counter_value();
    devnull = ptr[0];
    devnull = ptr[1];
    osalSysUnlock();

    osalThreadSleep(1);
  }
}

