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

#include "main.h"
#include "pads.h"

#include "fpga.h"
#include "fpga_pwm.h"
#include "fpga_icu.h"
#include "fpga_mul_test.hpp"
#include "fpga_mem_test.hpp"

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

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

//typedef uint16_t memword;
//volatile memword pattern = 0xFF00;
//volatile memword devnull = 0;
//void mem_oscillo_probe_dbg(void) {
//  volatile memword *ptr = (memword *)memtest_struct.start;
//
//  osalSysLock();
//  ptr[0] = pattern;
////  ptr[1] = ~pattern;
////  port_rt_get_counter_value();
////  devnull = ptr[0];
////  devnull = ptr[1];
//  osalSysUnlock();
//}

void fpga_addr_test(void) {
  fpgapwmSet(&FPGAPWMD1, 0xFFFF, 2);
  osalThreadSleepMilliseconds(1);
  osalDbgCheck(PAL_HIGH == palReadPad(GPIOD, GPIOD_FPGA_IO5));
  fpgapwmSet(&FPGAPWMD1, 0, 2);
  osalThreadSleepMilliseconds(1);
  osalDbgCheck(PAL_LOW == palReadPad(GPIOD, GPIOD_FPGA_IO5));
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

  fpga_mem_test(&FPGAD1, 6);

  mulObjectInit(&MTRXMULD1);
  mulStart(&MTRXMULD1, &FPGAD1);

  fpgapwmObjectInit(&FPGAPWMD1);
  fpgapwmStart(&FPGAPWMD1, &FPGAD1);

  fpgaicuObjectInit(&FPGAICUD1);
  fpgaicuStart(&FPGAICUD1, &FPGAD1);

  fpgacmd_t pwm_val = 0;
  fpgacmd_t icu_val = 0;

//  while (true) {
//    fpga_addr_test();
//  }


  while (true) {
    for (size_t i=0; i<16; i++) {
      fpgapwmSet(&FPGAPWMD1, 1000 * (i+1), i);
      //icu_val = fpgaicuRead(&FPGAICUD1, 0);
    }
    pwm_val++;
    if (pwm_val > 1000)
      pwm_val = 0;

    osalThreadSleepMilliseconds(1);

    //icu_val = fpgaicuRead(&FPGAICUD1, 0);
  }


//  while (true) {
//    fpga_mul_test(&MTRXMULD1);
//    orange_led_toggle();
//    osalThreadSleepMilliseconds(50);
//  }
}


