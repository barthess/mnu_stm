#ifndef MATRIX_CLASS_HPP_
#define MATRIX_CLASS_HPP_

#include <utility> // for std::move
#include <cstring> // memcpy, memset

#include "matrix_mem_pool.hpp"
#include "matrix_mem_mgr.hpp"
#include "matrix_osal.hpp"
#include "matrix_soft_engine.hpp"
#include "fpga_mtrx.h"

#define MATRIX_COPY_CTOR_ENABLED      1
#define MATRIX_COPY_OPERATOR_ENABLED  1

#define FPGA_PRESENT          1
#define FPGA_SUITABLE_LEN     16 // number of elements of type double

namespace matrix {

/**
 *
 */
enum class init_type {
  none,
  set,
  dia
};

/**
 *
 */
enum class location {
  uninit,
  ram,
  fpga
};

/**
 *
 */
template<typename T, size_t r, size_t c>
class Matrix {

public:
  /* raw pointer */
  T *M;
  /* pool indices for faster malloc/free */
  size_t fpga_idx = ~0;
  size_t ram_idx  = get_pool_index();
  location loc = location::uninit;

public:

  static_assert((c>0) && (r>0), "Zero size forbidden");

  /**
   *
   */
  ~Matrix(void) {
    this->mem_free();
  }

  /**
   *
   */
  Matrix(void) {
    matrixDbgPrint("Matrix default constructor\n");
    default_ctor(location::ram, init_type::none, 0);
  }

  /**
   *
   */
  Matrix(location loc) {
    matrixDbgPrint("Matrix default constructor\n");
    default_ctor(loc, init_type::none, 0);
  }

  /**
   *
   */
  Matrix(init_type it, T pattern) {
    matrixDbgPrint("Matrix pattern constructor\n");
    default_ctor(location::ram, it, pattern);
  }

  /**
   *
   */
  Matrix(location loc, init_type it, T pattern) {
    matrixDbgPrint("Matrix pattern constructor\n");
    default_ctor(loc, it, pattern);
  }

  /**
   * @brief   Copy constructor forbidden
   */
  Matrix(const Matrix &src) = delete;

  /**
   * @brief   Assignment forbidden
   */
  void operator = (const Matrix &src) = delete;

  /**
   * @brief Move constructor.
   */
  Matrix(Matrix &&src) {
    matrixDbgPrint("Matrix move constructor\n");
    this->M = src.M;
    this->fpga_idx = src.fpga_idx;
    this->ram_idx = src.ram_idx;
    this->loc = src.loc;
    src.M = nullptr;
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
      this->mem_free();
      this->M = src.M;
      this->fpga_idx = src.fpga_idx;
      this->ram_idx = src.ram_idx;
      this->loc = src.loc;
      src.M = nullptr;
      return *this;
    }
  }

  /**
   *
   */
  location get_location(void) const {
    return this->loc;
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
  constexpr size_t msize(void) const {
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

  /**
   *
   */
  void default_ctor(location loc, init_type it, double val) {
    switch(loc) {
    case (location::fpga):
      fpga_ctor(it, val);
      break;
    case(location::ram):
      ram_ctor(it, val);
      break;
    default:
      osalSysHalt("Unhandled case");
      break;
    }
  }

  /**
   *
   */
  void fpga_ctor(init_type it, double val) {

    matrixDbgCheck(FPGA_PRESENT && (sizeof(T) == sizeof(double)));

    this->M = static_cast<T *>(fpga_malloc(&this->fpga_idx, this->msize()));

    switch(it) {
    case init_type::none:
      (void)val;
      break;
    case init_type::set:
      fpgaMtrxSet(&MTRXD1, r, c, this->fpga_idx, val);
      break;
    case init_type::dia:
      matrixDbgCheck(r == c);
      fpgaMtrxDia(&MTRXD1, r, this->fpga_idx, val);
      break;
    }

    this->loc = location::fpga;
  }

  /**
   *
   */
  void ram_ctor(init_type it, double val) {

    this->M = static_cast<T *>(pool_malloc(this->ram_idx, this->msize()));

    switch(it) {
    case init_type::none:
      (void)val;
      break;
    case init_type::set:
      this->soft_pattern_fill(val);
      break;
    case init_type::dia:
      matrixDbgCheck(r == c);
      this->soft_set_diag(val);
      break;
    }

    this->loc = location::ram;
  }

  /**
   *
   */
  void soft_pattern_fill(T pattern) {
    for (size_t i=0; i<(c*r); i++) {
      M[i] = pattern;
    }
  }

  /**
   *
   */
  void soft_set_diag(T val) {
    size_t i = 0;

    soft_pattern_fill(0);

    while (i < (c*r)) {
      this->M[i] = val;
      i += r+1;
    }
  }

  /**
   *
   */
  void mem_free(void) {
    if (location::fpga == get_location()) {
      fpga_free(M, fpga_idx);
    }
    else {
      pool_free(M, ram_idx);
    }
  }

  /**
   * @brief calculate subindex from row and col values
   */
  size_t calc_subindex(size_t row, size_t col) const {
    matrixDbgCheck((c > col) && (r > row)); /* overflow */
    return row*c + col;
  }
};

} /* namespace matrix */

#endif /* MATRIX_CLASS_HPP_ */

