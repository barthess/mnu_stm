#ifndef MATRIX_SUB_SINGLE_HPP_
#define MATRIX_SUB_SINGLE_HPP_

#include "matrix_class.hpp"

namespace matrix {

/**
 *
 */
template <size_t m, size_t n>
Matrix<float, m, n> operator - (const Matrix<float, m, n> &A,
                                const Matrix<float, m, n> &B) {
  matrixDbgPrint("Matrix - operator\n");
  Matrix<float, m, n> C(location::ram);

  if (!A.tr && !B.tr)
    matrix_soft_sub(m, n, A.M, B.M, C.M);
  else if (A.tr && !B.tr)
    matrix_soft_sub_TA(m, n, A.M, B.M, C.M);
  else if (!A.tr && B.tr)
    matrix_soft_sub_TB(m, n, A.M, B.M, C.M);
  else
    matrix_soft_sub_TAB(m, n, A.M, B.M, C.M);

  return C;
}

} /* namespace matrix */

#endif /* MATRIX_SUB_SINGLE_HPP_ */

