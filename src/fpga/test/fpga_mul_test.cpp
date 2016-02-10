//#pragma GCC optimize "-ffast-math"
//#pragma GCC optimize "-funroll-loops"


#include <cmath>
#include <cstdlib>
#include <cstring>

#include "main.h"
#include "pads.h"

#include "fpga_mul_test.hpp"

#include "matrix_primitives.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

static const size_t CTL_OP = 0;
static const size_t CTL_SIZES = 1;

#define RAND_POOL_LEN        (65536 / sizeof(double))
#define RAND_POOL_ADR_MASK   (RAND_POOL_LEN - 1)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static double *fpga_pool[8]; // will be initialized later

static double mtrx_pool[FPGA_MTRX_BRAMS_CNT]
                       [(1 << FPGA_MTRX_INDEX_WIDTH) * (1 << FPGA_MTRX_INDEX_WIDTH)];

__CCM__ static double rand_pool[RAND_POOL_LEN];

static time_measurement_t tmu_soft;
static time_measurement_t tmu_hard;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
double rand_double(void) {
  const double MAX_MTRX_NUM = 10000;
  const double MIN_MTRX_NUM = .0001;
  double r = MAX_MTRX_NUM + 1;
  double a, b;

  while ((r > MAX_MTRX_NUM) or (r < MIN_MTRX_NUM)) {
    a = rand();
    b = rand();
    r = a / b;
  }

  if ((rand() & 1) == 1)
    return -r;
  else
    return r;
}

/**
 *
 */
void init_rand_pool(void) {
  for (size_t i=0; i<RAND_POOL_LEN; i++) {
    rand_pool[i] = rand_double();
  }
}

/**
 *
 */
double fast_rand_double(void) {
  return rand_pool[rand() & RAND_POOL_ADR_MASK];
}

/**
 *
 */
double pyramid_accumulate(const double *d, size_t len) {
  size_t N;

  if (len % 2 == 1) {
    N = (len + 1) / 2;
  }
  else {
    N = len / 2;
  }

  double buf[N];
  size_t i = 0, j = 0;
  while (len > 1) {
    buf[i] = d[j] + d[j+1];
    i++;
    j += 2;
    len -= 2;
  }

  // copy last element as is when len is odd
  if (len == 1) {
    buf[i] = d[j];
  }

  if (1 == N)
    return buf[0];
  else
    return(pyramid_accumulate(buf, N)); // recursive call
}

/**
 * @brief   multiply matrix A(m x p) by  B(p x n), put result in C(m x n)
 */
void matrix_multiply_fpga_like(size_t m, size_t p, size_t n,
                               const double *A, const double *B, double *C) {
  size_t i, j, k;
  double tmp[p];

  for(i=0; i<m; i++) {      //each row in A
    for(j=0; j<n; j++) {    //each column in B
      for(k=0; k<p; k++) {  //each element in row A & column B
        tmp[k] = A[i*p + k] * B[k*n + j];
      }
      *C++ = pyramid_accumulate(tmp, p);
    }
  }
}

/**
 * @brief   multiply matrix A(m x p) by  B(p x n), put result in C(m x n)
 */
template <typename T>
void matrix_multiply(size_t m, size_t p, size_t n,
                     const T *A, const T *B, T *C) {
  size_t i, j, k;
  T tmp;

  for(i=0; i<m; i++) {     //each row in A
    for(j=0; j<n; j++) {   //each column in B
      tmp = 0;
      for(k=0; k<p; k++)  //each element in row A & column B
        tmp += A[i*p + k] * B[k*n + j];
      //C[i*n + j] = tmp;
      *C++ = tmp;
    }
  }
}

/**
 *
 */
template <typename T>
static bool mtrx_compare_approx(const T *soft_dat, const T *fpga_dat, size_t m, size_t n) {
  T tmp1, tmp2;
  m++;
  n++;
  for(size_t i=0; i<m*n; i++) {
    tmp1 = soft_dat[i];
    tmp2 = fpga_dat[i];
    if (fabsf(tmp1 - tmp2) > 0.1) {
      return false;
    }
  }
  return true;
}

/**
 *
 */
template <typename T>
static bool mtrx_compare_exact(const T *soft_dat, const T *fpga_dat, size_t m, size_t n) {
  T tmp1, tmp2;
  m++;
  n++;
  for(size_t i=0; i<m*n; i++) {
    tmp1 = soft_dat[i];
    tmp2 = fpga_dat[i];
    if (fabsf(tmp1 - tmp2) > 0) {
      return false;
    }
  }
  return true;
}

/**
 *
 */
static void fpga_mtrx_wait_polling(void) {

  orange_led_on();

  while (! FSMCDataFlushed())
    ;

  while(! FPGAMulRdy())
    ;

  orange_led_off();
}

/**
 *
 */
fpgaword_t fill_sizes(size_t m, size_t p, size_t n) {

  fpgaword_t ret;

  osalDbgCheck((m <= FPGA_MTRX_MAX_INDEX) &
               (p <= FPGA_MTRX_MAX_INDEX) &
               (n <= FPGA_MTRX_MAX_INDEX));

  ret   = m << FPGA_MTRX_INDEX_WIDTH * 0;
  ret  |= p << FPGA_MTRX_INDEX_WIDTH * 1;
  ret  |= n << FPGA_MTRX_INDEX_WIDTH * 2;

  return ret;
}

/**
 *
 */
fpgaword_t fill_sizes(size_t m, size_t n) {
  fpgaword_t ret;

  osalDbgCheck((m <= FPGA_MTRX_MAX_INDEX) &
               (n <= FPGA_MTRX_MAX_INDEX));

  ret   = m << FPGA_MTRX_INDEX_WIDTH * 0;
  ret  |= n << FPGA_MTRX_INDEX_WIDTH * 2;

  return ret;
}


/**
 *
 */
fpgaword_t fill_blk_adr(fpgaword_t A, fpgaword_t B, fpgaword_t C, fpgaword_t opcode) {
  fpgaword_t ret;

  osalDbgCheck((A != B) & (B != C) & (C != A));
  osalDbgCheck((A < FPGA_MTRX_BRAMS_CNT) &
               (B < FPGA_MTRX_BRAMS_CNT) &
               (C < FPGA_MTRX_BRAMS_CNT));

  ret  = A << FPGA_MTRX_BRAMS_CNT_BITS * 0;
  ret |= B << FPGA_MTRX_BRAMS_CNT_BITS * 1;
  ret |= C << FPGA_MTRX_BRAMS_CNT_BITS * 2;
  ret |= opcode << FPGA_MTRX_BRAMS_CNT_BITS * 3;
  ret |= 1 << FPGA_MTRX_DV_BIT;

  return ret;
}

/**
 *
 */
fpgaword_t fill_blk_adr(fpgaword_t A, fpgaword_t C, fpgaword_t opcode) {
  fpgaword_t ret;

  osalDbgCheck(C != A);
  osalDbgCheck((A < FPGA_MTRX_BRAMS_CNT) &
               (C < FPGA_MTRX_BRAMS_CNT));

  ret  = A << FPGA_MTRX_BRAMS_CNT_BITS * 0;
  ret |= C << FPGA_MTRX_BRAMS_CNT_BITS * 2;
  ret |= opcode << FPGA_MTRX_BRAMS_CNT_BITS * 3;
  ret |= 1 << FPGA_MTRX_DV_BIT;
  return ret;
}

/**
 *
 */
fpgaword_t fill_blk_adr(fpgaword_t C, fpgaword_t opcode) {
  fpgaword_t ret;

  osalDbgCheck(C < FPGA_MTRX_BRAMS_CNT);

  ret  = C << FPGA_MTRX_BRAMS_CNT_BITS * 2;
  ret |= opcode << FPGA_MTRX_BRAMS_CNT_BITS * 3;
  ret |= 1 << FPGA_MTRX_DV_BIT;

  return ret;
}

/**
 *
 */
void fill_constant(const double val, fpgaword_t *ctl) {
  memcpy(&ctl[4], &val, sizeof(val));
}

/*****************************************************************************************
 * Hardware API
 *****************************************************************************************/
/**
 * функции принимают размеры, пригодные для прямой записи в регистры,
 * т.е. 0х0 - это вырожденная матрица 1х1
 */
void fpga_mtrx_dot(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes(m, p, n);
  ctl[CTL_OP]    = fill_blk_adr(A, B, C, MATH_OP_DOT);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_add(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(A, B, C, MATH_OP_ADD);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_sub(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(A, B, C, MATH_OP_SUB);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_mul(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(A, B, C, MATH_OP_MUL);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_scale(size_t m,           size_t n,
                     size_t A,           size_t C,
                     fpgaword_t *ctl, double scale) {

  fill_constant(scale, ctl);

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(A, C, MATH_OP_SCALE);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_cpy(size_t m,           size_t n,
                   size_t A,           size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(A, C, MATH_OP_CPY);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_trn(size_t m,           size_t n,
                   size_t A,           size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(A, C, MATH_OP_TRN);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_set(size_t m,           size_t n,
                                       size_t C,
                   fpgaword_t *ctl, double set_val) {

  fill_constant(set_val, ctl);

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(C, MATH_OP_SET);

  fpga_mtrx_wait_polling();
}

/**
 *
 */
void fpga_mtrx_eye(size_t m,           size_t n,
                                       size_t C,
                   fpgaword_t *ctl, double set_val) {

  osalDbgCheck(m == n);

  fill_constant(set_val, ctl);

  ctl[CTL_SIZES] = fill_sizes(m, n);
  ctl[CTL_OP]    = fill_blk_adr(C, MATH_OP_EYE);

  fpga_mtrx_wait_polling();
}







/*****************************************************************************************
 * Software variants
 *****************************************************************************************/
/**
 *
 */
void soft_mtrx_dot(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  p++;
  n++;
  matrix_multiply_fpga_like(m, p, n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
  //matrix_multiply(m, p, n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_add(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  n++;
  matrix::matrix_add(m*n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_sub(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  n++;
  matrix::matrix_substract(m*n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_mul(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    mtrx_pool[C][i] = mtrx_pool[A][i] * mtrx_pool[B][i];
  }
}

/**
 *
 */
void soft_mtrx_scale(size_t m,           size_t n,
                     size_t A,           size_t C,
                     double scale) {
  m++;
  n++;
  matrix::matrix_scale(mtrx_pool[C], mtrx_pool[A], scale, m*n);
}

/**
 *
 */
void soft_mtrx_cpy(size_t m,           size_t n,
                   size_t A,           size_t C) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    mtrx_pool[C][i] = mtrx_pool[A][i];
  }
}

/**
 *
 */
void soft_mtrx_set(size_t m,           size_t n,
                                       size_t C,
                   double set_val) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    mtrx_pool[C][i] = set_val;
  }
}

/**
 *
 */
void soft_mtrx_trn(size_t m,           size_t n,
                   size_t A,           size_t C) {
  m++;
  n++;
  matrix::matrix_deep_transpose(m, n, mtrx_pool[A], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_eye(size_t m,           size_t n,
                                       size_t C,
                   double set_val) {
  size_t i = 0;

  osalDbgCheck(m == n);

  m++;
  n++;
  for (i=0; i<m*n; i++) {
    mtrx_pool[C][i] = set_val;
  }

  i = 0;
  while (i < m*n) {
    mtrx_pool[C][i] = 1;
    i += m+1;
  }
}


/*******************************************************************************
 * manual fill of arrays in FPGA
 *******************************************************************************/
void manual_fill_pattern(double *ptr, double pattern, size_t m, size_t n) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    ptr[i] = pattern;
  }
}

void manual_fill_copy(double *ptr, const double *data, size_t m, size_t n) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    ptr[i] = data[i];
  }
}

void manual_fill_rand(double *ptr, size_t m, size_t n) {

  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    ptr[i] = fast_rand_double();
  }
}

/*******************************************************************************
 * Test wrappers
 *******************************************************************************/

/**
 *
 */
void fpga_dot_test(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], m, n);

  soft_mtrx_dot(m, p, n, A, B, C);

  chTMStartMeasurementX(&tmu_hard);
  fpga_mtrx_dot(m, p, n, A, B, C, ctl);
  chTMStopMeasurementX(&tmu_hard);

  if (false == mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n)){
    osalSysHalt("");
  }
}

/**
 *
 */
void fpga_add_test(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl, bool add_not_sub) {

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], m, n);

  if (add_not_sub) {
    soft_mtrx_add(m, n, A, B, C);
    fpga_mtrx_add(m, n, A, B, C, ctl);
  }
  else {
    soft_mtrx_sub(m, n, A, B, C);
    fpga_mtrx_sub(m, n, A, B, C, ctl);
  }

  if (false == mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n)){
    osalSysHalt("");
  }
}

/**
 *
 */
void fpga_mul_test(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl, bool mul_not_scale) {

  double scale = fast_rand_double();

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], m, n);

  if (mul_not_scale) {
    soft_mtrx_mul(m, n, A, B, C);
    fpga_mtrx_mul(m, n, A, B, C, ctl);
  }
  else {
    soft_mtrx_scale(m, n, A, C, scale);
    fpga_mtrx_scale(m, n, A, C, ctl, scale);
  }

  if (false == mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n)){
    osalSysHalt("");
  }
}

/**
 *
 */
void fpga_mov_test(size_t m,           size_t n,
                   size_t A,           size_t C,
                   fpgaword_t *ctl) {

  double set_val;

//  manual_fill_rand(mtrx_pool[A], m, n);
//  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
//  soft_mtrx_cpy(m, n, A, C);
//  fpga_mtrx_cpy(m, n, A, C, ctl);
//  if (false == mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n)){
//    osalSysHalt("");
//  }

  set_val = fast_rand_double();
  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  soft_mtrx_set(m, n, C, set_val);
  fpga_mtrx_set(m, n, C, ctl, set_val);
  if (false == mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n)){
    osalSysHalt("");
  }

//  manual_fill_rand(mtrx_pool[A], m, n);
//  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
//  soft_mtrx_trn(m, n, A, C);
//  fpga_mtrx_trn(m, n, A, C, ctl);
//  if (false == mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n)){
//    osalSysHalt("");
//  }

  // Eye works correctly only with square matrices
  if (m == n) {
    set_val = fast_rand_double();
    manual_fill_rand(mtrx_pool[A], m, n);
    manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
    soft_mtrx_eye(m, n, C, set_val);
    fpga_mtrx_eye(m, n, C, ctl, set_val);
    if (false == mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n)){
      osalSysHalt("");
    }
  }
}

/**
 *
 */
void fpga_dot_test_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;

  size_t m, p, n;

  m = 31;
  p = 31;
  n = 31;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 0;
  p = 0;
  n = 0;
  fpga_dot_test(m, p, n, A, B, C, ctl);

//  const size_t MAX_BRUTE_SIZE = 3;
//  m = MAX_BRUTE_SIZE;
//  p = MAX_BRUTE_SIZE;
//  n = MAX_BRUTE_SIZE;
//
//  while(m--) {
//    while(p--) {
//      while(n--) {
//        fpga_dot_test(m, p, n, A, B, C, ctl);
//      }
//      n = MAX_BRUTE_SIZE;
//    }
//    p = MAX_BRUTE_SIZE;
//  }
}

/**
 *
 */
void fpga_add_test_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;
  //const size_t MAX_BRUTE_SIZE = 2;

  size_t m;
  size_t n;

  m = 0;
  n = 0;
  fpga_add_test(m, n, A, B, C, ctl, true);
  fpga_add_test(m, n, A, B, C, ctl, false);

  m = 0;
  n = 1;
  fpga_add_test(m, n, A, B, C, ctl, true);
  fpga_add_test(m, n, A, B, C, ctl, false);

  m = 1;
  n = 0;
  fpga_add_test(m, n, A, B, C, ctl, true);
  fpga_add_test(m, n, A, B, C, ctl, false);

  m = 1;
  n = 1;
  fpga_add_test(m, n, A, B, C, ctl, true);
  fpga_add_test(m, n, A, B, C, ctl, false);

  m = 0;
  n = 31;
  fpga_add_test(m, n, A, B, C, ctl, true);
  fpga_add_test(m, n, A, B, C, ctl, false);

  m = 31;
  n = 0;
  fpga_add_test(m, n, A, B, C, ctl, true);
  fpga_add_test(m, n, A, B, C, ctl, false);

  m = 31;
  n = 31;
  fpga_add_test(m, n, A, B, C, ctl, true);
  fpga_add_test(m, n, A, B, C, ctl, false);
}

/**
 *
 */
void fpga_mul_test_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;
  size_t m, n;

  m = 0;
  n = 0;
  fpga_mul_test(m, n, A, B, C, ctl, true);
  fpga_mul_test(m, n, A, B, C, ctl, false);

  m = 0;
  n = 1;
  fpga_mul_test(m, n, A, B, C, ctl, true);
  fpga_mul_test(m, n, A, B, C, ctl, false);

  m = 1;
  n = 0;
  fpga_mul_test(m, n, A, B, C, ctl, true);
  fpga_mul_test(m, n, A, B, C, ctl, false);

  m = 1;
  n = 1;
  fpga_mul_test(m, n, A, B, C, ctl, true);
  fpga_mul_test(m, n, A, B, C, ctl, false);

  m = 0;
  n = 31;
  fpga_mul_test(m, n, A, B, C, ctl, true);
  fpga_mul_test(m, n, A, B, C, ctl, false);

  m = 31;
  n = 0;
  fpga_mul_test(m, n, A, B, C, ctl, true);
  fpga_mul_test(m, n, A, B, C, ctl, false);

  m = 31;
  n = 31;
  fpga_mul_test(m, n, A, B, C, ctl, true);
  fpga_mul_test(m, n, A, B, C, ctl, false);
}

/**
 *
 */
void fpga_mov_test_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t C = 2;
  size_t m, n;

  m = 0;
  n = 0;
  fpga_mov_test(m, n, A, C, ctl);

//  m = 0;
//  n = 1;
//  fpga_mov_test(m, n, A, C, ctl);

//  m = 1;
//  n = 0;
//  fpga_mov_test(m, n, A, C, ctl);
//
//  m = 1;
//  n = 1;
//  fpga_mov_test(m, n, A, C, ctl);

//  m = 0;
//  n = 31;
//  fpga_mov_test(m, n, A, C, ctl);

//  m = 31;
//  n = 0;
//  fpga_mov_test(m, n, A, C, ctl);

  m = 31;
  n = 31;
  fpga_mov_test(m, n, A, C, ctl);
}


/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
volatile fpgaword_t tmp;
volatile double distance;
void fpga_mul_test(FPGADriver *fpgap, size_t turns) {

  osalDbgCheck(fpgap->state == FPGA_READY);

  chTMObjectInit(&tmu_soft);
  chTMObjectInit(&tmu_hard);

  srand(chSysGetRealtimeCounterX());

  for (size_t i=0; i<8; i++) {
    fpga_pool[i] = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF0 + i);
  }

  fpgaword_t *ctl = fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_CTL);

  init_rand_pool();

  while(turns--) {
//    fpga_add_test_corner(ctl);
//    fpga_dot_test_corner(ctl);
//    fpga_mul_test_corner(ctl);
    fpga_mov_test_corner(ctl);


//    green_led_off();
//    red_led_off();
    green_led_toggle();
    osalThreadSleep(1);
  }
}











