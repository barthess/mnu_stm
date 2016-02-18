#ifndef KALMAN_HPP_
#define KALMAN_HPP_

#include "embmatrix2/matrix.hpp"

typedef double klmfp; /* kalman floating point type */

class Kalman{
public:
  Kalman(void);
  ~Kalman(void){
    return;
  }
  float run(void);
#if defined(_CHIBIOS_RT_)
  void *operator new(size_t size){
    matrixDbgCheck(sizeof(Kalman) == size);
    return chCoreAlloc(size);
  }
  void operator delete (void *mem){
    (void)mem;
    matrixDbgPanic("Can not delete object because allocated memory can not be deallocate");
  }
#endif
private:
  matrix::Matrix<klmfp, 32, 32> R, F, T, J;
  matrix::Matrix<klmfp, 1, 32> Ttest;
  matrix::Matrix<klmfp, 32, 1> RTtest;
};


#endif /* KALMAN_HPP_ */
