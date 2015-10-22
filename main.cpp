/*
    ChibiOS/RT - Copyright (C) 2013-2014 Uladzimir Pylinsky aka barthess

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "fsmc_sram.h"
#include "pads.h"
#include "memtest.h"
#include "msno.hpp"
#include "idt5.hpp"
#include "ui.hpp"
#include "fpga.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define USE_INFINITE_MEMTEST    TRUE

#define MEMTEST_RUN_ALL_FAST  (MEMTEST_WALKING_ONE              | \
                               MEMTEST_WALKING_ZERO             | \
                               MEMTEST_OWN_ADDRESS              | \
                               MEMTEST_MOVING_INVERSION_ZERO    | \
                               MEMTEST_MOVING_INVERSION_55AA)

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

static const size_t sram_size = 8*8 * 1024;//128 * 1024;
static double test_buf_mtrx[33*33];

// sync
static const SRAMConfig sram_cfg = {
    (FSMC_BCR_MWID_16 | FSMC_BCR_MTYP_PSRAM | FSMC_BCR_BURSTEN | FSMC_BCR_WREN | FSMC_BCR_CBURSTRW | FSMC_BCR_WAITPOL),

    // BTR
    (0 << 24) | // DATLAT
    (2 << 20) | // CLKDIV (0 value not supported, 15 - max)
    (0 << 16),  // BUSTURN

    // BWTR
    (0 << 24) | // DATLAT
    (2 << 20) | // CLKDIV (0 value not supported, 15 - max)
    (0 << 16),  // BUSTURN
};

// async
//static const SRAMConfig sram_cfg = {
//    (FSMC_BCR_MWID_16 | FSMC_BCR_MTYP_SRAM | FSMC_BCR_WREN | FSMC_BCR_EXTMOD),
//
//    // BTR
//    (6 << 16) | // BUSTURN
//    (12 << 8) |  // DATAST
//    (0 << 4) |  // ADDHLD
//    (0 << 0),   // ADDSET
//
//    // BWTR
//    (6 << 16) | // BUSTURN
//    (9 << 8) |  // DATAST
//    (0 << 4) |  // ADDHLD
//    (0 << 0),   // ADDSET
//};

static memtest_t memtest_struct = {
    (void *)FSMC_Bank1_1_MAP,
    sram_size,
    MEMTEST_WIDTH_64 | MEMTEST_WIDTH_32 | MEMTEST_WIDTH_16,
    //MEMTEST_WIDTH_8,
    mem_error_cb
};

time_measurement_t mem_tmu_w;
time_measurement_t mem_tmu_r;

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
  memtest_run(&memtest_struct, MEMTEST_RUN_ALL);
}

/*
 *
 */
static void membench(void) {

  chTMObjectInit(&mem_tmu_r);
  chTMObjectInit(&mem_tmu_w);
  double *ptr = (double *)memtest_struct.start;

  double pattern = 1.1;
  for (size_t i=0; i<33*33; i++) {
    test_buf_mtrx[i] = pattern;
    pattern += pattern;
  }

  chTMStartMeasurementX(&mem_tmu_w);
  for (size_t i=0; i<1000; i++) {
    for (size_t i=0; i<33*33; i++) {
      ptr[i] = test_buf_mtrx[i];
    }
    //memcpy(memtest_struct.start, test_buf_mtrx, sizeof(test_buf_mtrx));
  }
  chTMStopMeasurementX(&mem_tmu_w);

  chTMStartMeasurementX(&mem_tmu_r);
  for (size_t i=0; i<1000; i++) {
    for (size_t i=0; i<33*33; i++) {
      test_buf_mtrx[i] = ptr[i];
    }
    //memcpy(test_buf_mtrx, memtest_struct.start, sizeof(test_buf_mtrx));
  }
  chTMStopMeasurementX(&mem_tmu_r);
}



typedef uint16_t memword;
static volatile memword pattern = 0xFF00;
static volatile memword devnull = 0;
void mem_oscillo_probe_dbg(void) {
  volatile memword *ptr = (memword *)memtest_struct.start;

  osalSysLock();
  ptr[0] = pattern;
//  ptr[1] = ~pattern;
//  port_rt_get_counter_value();
//  devnull = ptr[0];
//  devnull = ptr[1];
  osalSysUnlock();
}


/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/* heap for temporarily threads */
memory_heap_t ThdHeap;
static uint8_t link_thd_buf[THREAD_HEAP_SIZE + sizeof(stkalign_t)];

/*
 * Application entry point.
 */
int main(void) {

 /*
  * System initializations.
  * - HAL initialization, this also initializes the configured device drivers
  *   and performs the board-specific initializations.
  * - Kernel initialization, the main() function becomes a thread and the
  *   RTOS is active.
  */
  halInit();
  chSysInit();

  // Enable special "compensation cell" for IO working on 100MHz.
  // Looks like FSMC works slower when it enabled
//  rccEnableAPB2(RCC_APB2ENR_SYSCFGEN, false);
//  SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;
//  while (! SYSCFG->CMPCR & SYSCFG_CMPCR_READY)
//    ;

  chHeapObjectInit(&ThdHeap, (uint8_t *)MEM_ALIGN_NEXT(link_thd_buf), THREAD_HEAP_SIZE);

  osalThreadSleepMilliseconds(100);

  fpgaObjectInit(&FPGAD1);
  fpgaStart(&FPGAD1);

//  while (true) {
//    fpga_mul_test();
//  }

//  membench();

//  while (true) {
//    mem_oscillo_probe_dbg();
//  }

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {

    memtest();
    green_led_toggle();
    memtest();
    red_led_toggle();
    memtest();
    orange_led_toggle();
  }
}


