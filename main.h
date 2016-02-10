#ifndef MAIN_H_
#define MAIN_H_

/* chibios includes */
#ifdef __cplusplus
  #include "ch.hpp"
#else
  #include "ch.h"
#endif

#include "hal.h"

#define __CCM__ __attribute__((section(".ram4")))

#endif /* MAIN_H_ */
