#ifndef MATRIX_ADD_SINGLE_HPP_
#define MATRIX_ADD_SINGLE_HPP_

#include "matrix_class.hpp"

namespace matrix {

/**
 * @brief   Addition operator (single precision)
 */
template <size_t m, size_t n>
Matrix<float, m, n> operator + (const Matrix<float, m, n> &A,
                                const Matrix<float, m, n> &B) {
  matrixDbgPrint("Matrix + operator\n");
  Matrix<float, m, n> C(location::ram);

  if (!A.tr && !B.tr)
    matrix_soft_add(m, n, A.M, B.M, C.M);
  else if (A.tr && !B.tr)
    matrix_soft_add_TA(m, n, A.M, B.M, C.M);
  else if (!A.tr && B.tr)
    matrix_soft_add_TB(m, n, A.M, B.M, C.M);
  else
    matrix_soft_add_TAB(m, n, A.M, B.M, C.M);

  return C;
}

/**
 *
 */
template <size_t m, size_t n>
Matrix<float, m, n> operator + (const Matrix<float, m, n> &A,
                                      Matrix<float, m, n> &&B) {
  matrixDbgPrint("Matrix + rvalue operator\n");
  Matrix<float, m, n> C(std::move(B));
  C += A;
  return C;
}

/**
 *
 */
template <size_t m, size_t n>
Matrix<float, m, n> operator + (Matrix<float, m, n> &&A,
                          const Matrix<float, m, n> &B) {
  matrixDbgPrint("Matrix + rvalue operator\n");
  Matrix<float, m, n> C(std::move(A));
  C += B;
  return C;
}

/**
 *
 */
template <size_t m, size_t n>
Matrix<float, m, n> operator + (Matrix<float, m, n> &&A,
                                Matrix<float, m, n> &&B) {
  matrixDbgPrint("Matrix + rvalue operator\n");
  Matrix<float, m, n> C(std::move(A));
  C += B;
  return C;
}

} /* namespace matrix */

#endif /* MATRIX_ADD_SINGLE_HPP_ */

