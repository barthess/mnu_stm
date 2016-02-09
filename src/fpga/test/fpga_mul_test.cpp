#pragma GCC optimize "-ffast-math"
#pragma GCC optimize "-funroll-loops"


#include <cmath>

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


template <typename T>
static bool fpga_mtrx_compare(const T *C, const T *ref, size_t len) {
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
static void fpga_mtrx_wait_polling(void) {
  while (! FSMCDataFlushed())
    ;

  while(! FPGAMulRdy())
    green_led_toggle();
}

/**
 *
 */
static void fpga_mtrx_dot(size_t m, size_t p, size_t n,
                          size_t A, size_t B, size_t C,
                          fpgaword_t *ctl) {
  fpgaword_t tmp;

  // check matrix slices numbers
  osalDbgCheck((A != B) & (B != C) & (C != A));
  osalDbgCheck((A < FPGA_MTRX_BRAMS_CNT) &
               (B < FPGA_MTRX_BRAMS_CNT) &
               (C < FPGA_MTRX_BRAMS_CNT));

  // check and prepare matrix sizes
  osalDbgCheck((m > 0) & (p > 0) & (n > 0));
  m -= 1;
  p -= 1;
  n -= 1;
  osalDbgCheck((m <= FPGA_MTRX_MAX_INDEX) &
               (p <= FPGA_MTRX_MAX_INDEX) &
               (n <= FPGA_MTRX_MAX_INDEX));

  // write sizes
  tmp   = m << FPGA_MTRX_INDEX_WIDTH * 0;
  tmp  |= p << FPGA_MTRX_INDEX_WIDTH * 1;
  tmp  |= n << FPGA_MTRX_INDEX_WIDTH * 2;
  ctl[CTL_SIZES] = tmp;

  // write control register
  tmp  = A << FPGA_MTRX_BRAMS_CNT_BITS * 0;
  tmp |= B << FPGA_MTRX_BRAMS_CNT_BITS * 1;
  tmp |= C << FPGA_MTRX_BRAMS_CNT_BITS * 2;
  tmp |= MATH_OP_DOT << FPGA_MTRX_BRAMS_CNT_BITS * 3;
  tmp |= 1 << FPGA_MTRX_DV_BIT;
  ctl[CTL_OP] = tmp;

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

    osalDbgCheck(true == fpga_mtrx_compare(mtrx_C, res, 32*32));
  }

  green_led_off();
  red_led_off();
  orange_led_off();
}
