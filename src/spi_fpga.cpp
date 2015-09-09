#include "main.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define SPI_BUF_LEN       256

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */


/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 *
 */
static const SPIConfig spi_cfg = {
    NULL,
    GPIOI,
    GPIOI_SPI_NSS,
    0//SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2
};

static uint8_t ref[SPI_BUF_LEN];
static uint8_t buf[SPI_BUF_LEN];
static uint8_t counter = 0;

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

void spi_fpga_test(void) {

  spiStart(&SPID2, &spi_cfg);

  for (size_t i=0; i<SPI_BUF_LEN; i++) {
    ref[i] = i;
  }


  while (true) {
    spiSelect(&SPID2);
    spiPolledExchange(&SPID2, counter);
    spiUnselect(&SPID2);

    counter++;
    osalThreadSleepMilliseconds(150);
  }


  spiStop(&SPID2);
}
