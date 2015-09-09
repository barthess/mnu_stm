#ifndef MSNO_HPP_
#define MSNO_HPP_

#include "it530.hpp"

extern chibios_rt::EvtSource event_gps;

void msnoInit(void);
void msnoTest(gps::gps_data_t &result);

#endif /* MSNO_HPP_ */
