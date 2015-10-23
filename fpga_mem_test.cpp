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

static const size_t sram_size = FPGA_MTRX_SIZE * FPGA_MTRX_CNT * sizeof(double);

static memtest_t memtest_struct = {
    nullptr,
    sram_size,
    //MEMTEST_WIDTH_64 | MEMTEST_WIDTH_32 | MEMTEST_WIDTH_16,
    MEMTEST_WIDTH_64,
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
static void memtest(void) {
  //memtest_run(&memtest_struct, MEMTEST_WALKING_ZERO);
  memtest_run(&memtest_struct, MEMTEST_RUN_ALL);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void fpga_mem_test(FPGADriver *fpgap, size_t turns) {

  memtest_struct.start = fpgap->memspace->mtrx;

  while (turns--) {
    memtest();
    green_led_toggle();
    memtest();
    red_led_toggle();
    memtest();
    orange_led_toggle();
  }

  green_led_off();
  red_led_off();
  orange_led_off();
}






