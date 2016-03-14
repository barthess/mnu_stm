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
#include "fpga_uart.h"

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

static const FPGAUARTConfig cfg = {
    9600,
    FPGAUART_HW_FLOW_NONE
};

static FPGAUARTDriver *uartp = &FPGAUARTBridge.FPGAUARTD[0];

static const uint8_t test_str[] = "UUUUUUUUUUU";

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

  fpgaObjectInit(&FPGAD1);
  fpgaStart(&FPGAD1);




  fpgaUartObjectInit(&FPGAUARTBridge, &FPGAD1);
  fpgaUartBridgeStart(&FPGAUARTBridge);
  fpgaUartStart(uartp, &cfg);

  while (true) {
    osalThreadSleepMilliseconds(50);
    fpgaUartWrite(uartp, sizeof(test_str), test_str);
  }
}


