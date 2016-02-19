#ifndef MATRIX_DOT_DOUBLE_HPP_
#define MATRIX_DOT_DOUBLE_HPP_

#include "matrix_class.hpp"

namespace matrix {

/**
 *
 */
template <size_t m, size_t p, size_t n>
location strategy_dot(const Matrix<double, m, p> &A, const Matrix<double, p, n> &B) {

  uint32_t t_fpga = 0;
  uint32_t t_ram  = 0;
  uint32_t ram2fpga_speed = 20 * 6; // 20 ticks * 6ns уточнить при старте умножителя
  uint32_t fpga2ram_speed = 40 * 6; // 40 ticks * 6ns уточнить при старте умножителя
  uint32_t soft_dot_speed = 0;
  uint32_t fpga_dot_lat = 0;
  uint32_t fpga_speed = 0;

  if (location::ram == A.loc)
    t_fpga += m*p * ram2fpga_speed;
  else
    t_ram  += m*p * fpga2ram_speed;

  if (location::ram == B.loc)
    t_fpga += p*n * ram2fpga_speed;
  else
    t_ram  += p*n * fpga2ram_speed;

  // add (possible) fpga->ram eviction time
  t_fpga += fpga_evict_prediction() * fpga2ram_speed;

  // сложность операции будем считать пропорциональной произведению размерностей
  t_ram  += m*p*n * soft_dot_speed;
  t_fpga += m*p*n * fpga_speed + fpga_dot_lat;

  if (t_fpga > t_ram) {
    return location::ram;
  }
  else {
    return location::fpga;
  }
}

/**
 *
 */
template <size_t m, size_t p, size_t n>
Matrix<double, m, n> matrix_dot_engine(Matrix<double, m, p> &A,
                                       Matrix<double, p, n> &B,
                                       location loc) {

  if (location::fpga == loc){
    if (A.loc != location::ram) {
      fpga_settle(&A.M, m, n, &A.ram_idx);
    }
    if (B.loc != location::ram) {
      fpga_settle(&B.M, m, n, &B.ram_idx);
    }


    Matrix<double, m, n> C(loc);
    fpgaMtrxDot(&MTRXD1, m, p, n, A.fpga_idx, B.fpga_idx, C.fpga_idx);
  }
  else {
    if (A.loc != location::fpga) {
      fpga_evict(A.fpga_idx);
    }
    if (B.loc != location::fpga) {
      fpga_evict(B.fpga_idx);
    }

    matrix_soft_dot(m, p, n, A.M, B.M, C.M);
  }
}

/**
 * Multiplication operator (double precision)
 */
template <size_t m, size_t p, size_t n>
Matrix<double, m, n> operator * (Matrix<double, m, p> &A,
                                 Matrix<double, p, n> &B) {
  matrixDbgPrint("Matrix multiply operator double\n");

  return matrix_dot_engine(A, B, C, strategy_dot(A, B));
}

} /* namespace matrix */

#endif /* MATRIX_DOT_DOUBLE_HPP_ */

