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

  fpgapwmObjectInit(&FPGAPWMD1);
  fpgapwmStart(&FPGAPWMD1, &FPGAD1);

  fpgaword_t pwm_val = 0;
  fpgaword_t icu_val[5];

  while (true) {
    //fpgapwmSet(&FPGAPWMD1, pwm_val, 1);
//    for (size_t i=0; i<16; i++) {
//      fpgapwmSet(&FPGAPWMD1, 1000*i + 1, i);
//      fpgapwmSet(&FPGAPWMD1, pwm_val, i);
      //icu_val = fpgaicuRead(&FPGAICUD1, 0);
//    }
//    osalThreadSleepMilliseconds(1);
//    pwm_val++;
//    if (pwm_val > 2000)
//      pwm_val = 0;

//    fpgapwmSet(&FPGAPWMD1, 1500, 0);
//    osalThreadSleepMilliseconds(500);
//    fpgapwmSet(&FPGAPWMD1, 1600, 0);
//    osalThreadSleepMilliseconds(500);
//    fpgapwmSet(&FPGAPWMD1, 1400, 0);
//    osalThreadSleepMilliseconds(500);

    icu_val[0] = FPGAPWMD1.pwm[0];
    icu_val[1] = FPGAPWMD1.pwm[1];
    icu_val[2] = FPGAPWMD1.pwm[2];
    icu_val[3] = FPGAPWMD1.pwm[3];
    icu_val[4] = FPGAPWMD1.pwm[4];

    green_led_toggle();
  }
}


