#include "main.h"

#include "kalman.hpp"

using namespace matrix;

static uint64_t matrix_total_time = 0;

static const klmfp init[] = {1,2,3, 4,5,6, 7,8,9, 10,11,12, 10,11,12,13,
    1,2,3, 4,5,6, 7,8,9, 10,11,12, 10,11,12,13};

/**
 *
 */
Kalman::Kalman(void) :
  R(location::ram, init_type::set, 0.8),
  F(location::ram, init_type::set, 0.8),
  T(location::ram, init_type::set, 0.8),
  J(location::ram, init_type::dia, 1),
  Ttest(location::ram, init_type::set, 2.3),
  RTtest(location::ram, init_type::set, 0)
{
  return;
}

klmfp scale;

float Kalman::run(void) {

  Matrix<klmfp, 3, 3> Patch(location::ram);

  for(size_t i=0; i<100; i++){
    uint32_t start = chSysGetRealtimeCounterX();
    //R = T * J * F * ~(T*T);
    R = R * F;
//    J = R + T;
//    patch(F, Patch, 3, 1);
//    patch(F, row(Patch, 0), 10, 11);
//    RTtest = !J * ~Ttest;
//    RTtest = J * ~Ttest;
//    scalar = Ttest * RTtest;

    matrix_total_time += chSysGetRealtimeCounterX() - start;
  }
  chThdSleep(1); /* hack to enforce stack check by chibios */

  matrix_total_time = 0;

  return 0;
}


