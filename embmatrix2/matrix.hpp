#ifndef MATRIX_HPP_
#define MATRIX_HPP_

#include <utility> // for std::move
#include <cstring> // memcpy, memset

#include "matrix_osal.hpp"
#include "matrix_soft.hpp"

#define MATRIX_COPY_CTOR_ENABLED 1
#define MATRIX_COPY_OPERATOR_ENABLED 1

namespace matrix {

template<typename T, size_t r, size_t c>
class Matrix {

public:
  T *M;
  bool tr;    /* transpose flag */
  size_t idx; /* pool index cache for faster free() operation */

public:
  ~Matrix(void) {
    matrix_free(idx, M);
  }

  /**
   *
   */
  Matrix(void) {
    matrixDbgPrint("Matrix default constructor\n");
    _default_ctor();
  }

  /**
   * @brief   Copy constructor. Keep it for testing
   */
#if MATRIX_COPY_CTOR_ENABLED
  Matrix(const Matrix &src){
    matrixDbgPrint("Matrix copy constructor\n");
    _default_ctor();
    memcpy(this->M, src.M, msize());
    this->tr = src.tr;
  }
#else
  Matrix(const Matrix &src) = delete; /* Copy forbidden */
#endif /* MATRIX_COPY_CTOR_FORBIDDEN */

  /**
   * @brief Copy operator
   */
#if MATRIX_COPY_OPERATOR_ENABLED
  Matrix& operator = (const Matrix &src) {
    matrixDbgPrint("Matrix copy operator\n");
    if (this == &src) {
      return *this;
    } else {
      memcpy(this->M, src.M, msize());
      this->tr = src.tr;
      return *this;
    }
  }
#else
  void operator = (const Matrix &src) = delete; /* Assign forbidden */
#endif /* MATRIX_COPY_OPERATOR_ENABLED */

  /**
   *
   */
  Matrix(Matrix &&src) {
    matrixDbgPrint("Matrix move constructor\n");
    tr = src.tr;
    M = src.M;
    src.M = nullptr;
  }

  /**
   *
   */
  Matrix(T pattern) {
    matrixDbgPrint("Matrix pattern constructor\n");
    _default_ctor();
    for (size_t i=0; i<(c*r); i++)
      M[i] = pattern;
  }

  /**
   *
   */
  Matrix(const T *array, size_t arraysize) {
    matrixDbgPrint("Matrix const array constructor\n");
    matrixDbgCheck(msize() == arraysize); /* sizes mismatch */
    _default_ctor();
    memcpy(M, array, arraysize);
  }

  /**
   *
   */
  void transpose_hack(void) {
    this->tr = !this->tr;
  }

  /**
   *
   */
  bool is_transposed(void) const {
    return this->tr;
  }

  /**
   * @brief Move operator
   */
  Matrix& operator = (Matrix &&src) {
    matrixDbgPrint("Matrix move operator\n");

    if (this == &src) {
      return *this;
    }
    else {
      matrix_free(idx, this->M);
      this->tr  = src.tr;
      this->M   = src.M;
      this->idx = src.idx;
      src.M   = nullptr;
      src.idx = ~0;
      return *this;
    }
  }

  /**
   * @brief Subindex for Matrix elements assignation.
   * @param row
   * @param col
   * @return reference to the element.
   */
  T& operator() (size_t row, size_t col) {
    return M[calc_subindex(row, col)];
  }

  /**
   * @brief Subindex for Matrix element.
   * @param r
   * @param c
   * @return the element.
   */
  T operator() (size_t row, size_t col) const {
    return M[calc_subindex(row, col)];
  }

  /**
   * @brief Return matrix size in bytes
   */
  constexpr size_t msize(void) {
    return sizeof(T) * r * c;
  }

private:

  /**
   * @brief Determine closest suitable pool size
   */
  constexpr size_t get_pool_index(void) {
    size_t firstone = sizeof(size_t) * 8 - 1;
    size_t size = msize();

    if (0 == size)
      return 0;

    while(firstone-- > 0){
      if ((size & (1 << firstone)) > 0)
        break;
    }

    if (0 == firstone)
      return 0;

    if ((size & ((1 << firstone) - 1)) == 0)
      return firstone - 4;
    else
      return firstone - 3;
  }

  void _default_ctor(void) {
    static_assert((c>0) && (r>0), "Zero size forbidden");
    //this->M = static_cast<T *>(matrix_malloc(msize()));
    this->M = static_cast<T *>(matrix_malloc(pool_index(), msize()));
  }

  /**
   * @brief calculate subindex from row and col values
   */
  size_t calc_subindex(size_t row, size_t col) const {
    matrixDbgCheck((c > col) && (r > row)); /* overflow */
    return row*c + col;
  }
};

/**
 * Multiplication operator
 */
template <typename T, size_t m, size_t n, size_t p>
Matrix<T, m, p> operator * (const Matrix<T, m, n> &left,
                            const Matrix<T, n, p> &right) {
  Matrix<T, m, p> ret;
  matrixDbgPrint("Matrix multiply operator\n");

  if (!left.tr && !right.tr)
    matrix_multiply(m, n, p, left.M, right.M, ret.M);
  else if (left.tr && !right.tr)
    matrix_multiply_TA(m, n, p, left.M, right.M, ret.M);
  else if (!left.tr && right.tr)
    matrix_multiply_TB(m, n, p, left.M, right.M, ret.M);
  else
    matrix_multiply_TAB(m, n, p, left.M, right.M, ret.M);

  return ret;
}

/**
 * Scale operator
 */
template <typename T, size_t m, size_t n>
Matrix<T, m, n> operator * (const Matrix<T, m, n> &left, T scale) {
  Matrix<T, m, n> ret;
  matrix_scale(ret.M, left.M, scale, m*n);
  return ret;
}


} /* namespace matrix */
#endif /* MATRIX_HPP_ */
