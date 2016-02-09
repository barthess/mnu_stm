//#pragma GCC optimize "-ffast-math"
//#pragma GCC optimize "-funroll-loops"


#include <cmath>
#include <cstdlib>
#include <cstring>

#include "main.h"
#include "pads.h"

#include "fpga_mul_test.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

static const size_t CTL_OP = 0;
static const size_t CTL_SIZES = 1;

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

static double mtrx_A[32*32];
static double mtrx_B[32*32];
static double mtrx_C[32*32];

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
static double rand_double(void) {
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
static double pyramid_accumulate(const double *d, size_t len) {
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
static bool fpga_mtrx_compare_approx(const T *C, const T *ref, size_t len) {
  T tmp1, tmp2;

  for(size_t i=0; i<len; i++) {
    tmp1 = C[i];
    tmp2 = ref[i];
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
static bool fpga_mtrx_compare_exact(const T *C, const T *ref, size_t len) {
  T tmp1, tmp2;

  for(size_t i=0; i<len; i++) {
    tmp1 = C[i];
    tmp2 = ref[i];
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
  while (! FSMCDataFlushed())
    ;

  while(! FPGAMulRdy())
    green_led_toggle();
}

/**
 *
 */
static fpgaword_t fill_sizes(size_t m, size_t p, size_t n) {

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
static fpgaword_t fill_sizes(size_t m, size_t n) {
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
static fpgaword_t fill_blk_adr(fpgaword_t A, fpgaword_t B, fpgaword_t C, fpgaword_t opcode) {
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
static fpgaword_t fill_blk_adr(fpgaword_t A, fpgaword_t C, fpgaword_t opcode) {
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
static fpgaword_t fill_blk_adr(fpgaword_t C, fpgaword_t opcode) {
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
static void fill_constant(double val, fpgaword_t *ctl) {
  memcpy(&val, &ctl[4], sizeof(val));
}

/**
 * функция принимает размеры, пригодные для прямой записи в регистры,
 * т.е. 0х0 - это вырожденная матрица 1х1
 */
static void fpga_mtrx_dot(size_t m, size_t p, size_t n,
                          size_t A, size_t B, size_t C,
                          fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes(m, p, n);
  ctl[CTL_OP]    = fill_blk_adr(A, B, C, MATH_OP_DOT);

  // wait
  fpga_mtrx_wait_polling();
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

  double *op0 = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF0);
  double *op1 = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF1);
  double *res = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF2);

  double *dbg0 = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF3);
  double *dbg1 = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF4);
  double *dbg2 = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF5);
  double *dbg3 = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF6);
  double *dbg4 = (double *)fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_BUF7);

  fpgaword_t *ctl = fpgaGetCmdSlice(fpgap, FPGA_WB_SLICE_MUL_CTL);

  chTMObjectInit(&tmu_soft);
  chTMObjectInit(&tmu_hard);

  while (turns--) {

    for(size_t i=0; i<32*32; i++) {
      mtrx_A[i] = i*0.01;
      mtrx_B[i] = i*0.01 + 1.1;
    }

    chTMStartMeasurementX(&tmu_soft);
    //distance = boost_test();
    matrix_multiply(32, 32, 32, mtrx_A, mtrx_B, mtrx_C);
    //matrix_multiply_fast(32, 32, 32, mtrx_A, mtrx_B, mtrx_C);
    chTMStopMeasurementX(&tmu_soft);

    for(size_t i=0; i<32*32; i++) {
      op0[i] = i*0.01;
      op1[i] = i*0.01 + 1.1;
      res[i] = i+10.1;
      dbg0[i] = 666;
      dbg1[i] = 666;
      dbg2[i] = 666;
      dbg3[i] = 666;
      dbg4[i] = 666;
    }

    size_t m = 32;
    size_t p = 32;
    size_t n = 32;

    chTMStartMeasurementX(&tmu_hard);
    fpga_mtrx_dot(m, p, n, 0, 1, 2, ctl);
    chTMStopMeasurementX(&tmu_hard);

    osalDbgCheck(true == fpga_mtrx_compare_approx(mtrx_C, res, 32*32));
  }

  green_led_off();
  red_led_off();
  orange_led_off();
}
