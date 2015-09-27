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

#include "ch.hpp"
#include "hal.h"

#include "main.h"
#include "fsmc_sram.h"
#include "pads.h"
#include "memtest.h"
#include "i2c_local.hpp"
#include "tmp75.hpp"
#include "lsm303_mag.hpp"
#include "mpu6050.hpp"
#include "npa700.hpp"
#include "ms5806.hpp"
#include "fram.hpp"
#include "adc_local.hpp"
#include "it530.hpp"
#include "msno.hpp"
#include "idt5.hpp"
#include "ui.hpp"
#include "spi_fpga.hpp"

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

static const size_t sram_size = 128 * 1024;//128 * 1024;
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
    MEMTEST_WIDTH_32,
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
  //memtest_run(&memtest_struct, MEMTEST_RUN_ALL_FAST);
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

double op1 = 0;
double op2 = 0;
volatile double result;
const double step = 0.1;
size_t wait_cycles = 0;
double delta;
void fpga_mul_test(void) {
  volatile double *ptr = (double *)memtest_struct.start;
  volatile uint64_t *ptr_u64 = (uint64_t *)memtest_struct.start;

  ptr[1] = op1;
  ptr[2] = op2;
  ptr_u64[0] = 1;

  while (ptr_u64[0] != 0) {
    wait_cycles++;
  }

  osalThreadSleep(1);
  result = ptr[3];

  delta = fabs(result - op1*op2);
  if (delta > (double)0.0001)
    red_led_on();

  wait_cycles = 0;
  op1 += step;
  op2 += step;
  if (op1 > 10)
    op1 = -10;
  if (op2 > 10)
    op2 = -10;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

static TMP75 tmp75(&I2CD_SENSORS, tmp75addr);
static LSM303_mag lsm303mag(&I2CD_SENSORS, lsm303magaddr);
static MPU6050 mpu6050(&I2CD_SENSORS, mpu6050addr);
static NPA700 npa700(&I2CD_SENSORS, npa700addr);
static MS5806 ms5806(&I2CD_SENSORS, ms5806addr);
static fram fram0(&I2CD_NVRAM, FRAM0_I2C_ADDR);
static fram fram1(&I2CD_NVRAM, FRAM1_I2C_ADDR);
static gps::gps_data_t gps_data;
extern IDT5 idt5;

/* heap for temporarily threads */
memory_heap_t ThdHeap;
static uint8_t link_thd_buf[THREAD_HEAP_SIZE + sizeof(stkalign_t)];



enum GNSSReceiver {
  navi = 0,
  navi_nmea,
  ublox,
  unused,
};

static void gnss_select(GNSSReceiver receiver) {
  switch(receiver) {
  case GNSSReceiver::navi:
    palClearPad(GPIOB, GPIOB_FPGA_IO1);
    palClearPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  case GNSSReceiver::navi_nmea:
    palSetPad(GPIOB, GPIOB_FPGA_IO1);
    palClearPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  case GNSSReceiver::ublox:
    palClearPad(GPIOB, GPIOB_FPGA_IO1);
    palSetPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  case GNSSReceiver::unused:
    palSetPad(GPIOB, GPIOB_FPGA_IO1);
    palSetPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  }
}

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
  //uiInit();
  osalThreadSleepMilliseconds(100);

  fsmcSramInit();
  fsmcSramStart(&SRAMD1, &sram_cfg);

//  I2CInitLocal();
  ADCInitLocal();
//  tmp75.start();
//  lsm303mag.start();
//  npa700.start();
//  ms5806.start();
//
//  mpu6050_power_on();
//  mpu6050.start();
//
//  nvram_power_on();
//  fram0.start();
//  fram1.start();

//  idt5.start();

  while (!FPGAReady()) {
    orange_led_on();
    osalThreadSleepMilliseconds(30);
    orange_led_off();
    osalThreadSleepMilliseconds(70);
  }

  while (true) {
    fpga_mul_test();
    //osalThreadSleepSeconds(2);
  }

  membench();
  gnss_select(GNSSReceiver::ublox);
  msnoInit();

  //spi_fpga_test();

//  while (true) {
//    mem_oscillo_probe_dbg();
//  }

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (TRUE) {
//    osalThreadSleepMilliseconds(50);
//    green_led_toggle();
//
//    osalThreadSleepMilliseconds(50);
//    red_led_toggle();
//
//    osalThreadSleepMilliseconds(50);
//    orange_led_toggle();

//    tmp75.get();
//    npa700.get();
//    fram0.get();
//    fram1.get();
//    ADCgetBoardVoltage();
    msnoTest(gps_data);


    memtest();
    green_led_toggle();
    memtest();
    red_led_toggle();
    memtest();
    orange_led_toggle();
  }
}


