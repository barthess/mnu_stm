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
#include "test/fpga_mem_test.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define BRAM_DEPTH
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

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

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

  osalThreadSleepMilliseconds(100);

  fpgaObjectInit(&FPGAD1);
  fpgaStart(&FPGAD1);

  fpga_memtest(&FPGAD1, true, -1, FPGA_WB_SLICE_MEMTEST, 32768);

  while (true) {
    osalThreadSleepMilliseconds(100);
    orange_led_toggle();
  }
}


