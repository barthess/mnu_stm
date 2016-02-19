#ifndef MATRIX_DOT_SINGLE_HPP_
#define MATRIX_DOT_SINGLE_HPP_

#include "matrix_class.hpp"

namespace matrix {

/**
 * Multiplication operator (single precision)
 */
template <size_t m, size_t p, size_t n>
Matrix<float, m, n> operator * (const Matrix<float, m, p> &A,
                                const Matrix<float, p, n> &B) {
  matrixDbgPrint("Matrix multiply operator single\n");
  Matrix<float, m, n> C(location::ram);
  matrix_soft_dot(m, p, n, A.M, B.M, C.M);
  return C;
}

} /* namespace matrix */

#endif /* MATRIX_DOT_SINGLE_HPP_ */

