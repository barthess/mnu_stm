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

static double mtrx_A[10*10];
static double mtrx_B[10*10];
static double mtrx_C[10*10];

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

    for(size_t i=0; i<100; i++) {
      mtrx_A[i] = i+0.1;
      mtrx_B[i] = i+1.1;
    }

    chTMStartMeasurementX(&tmu_soft);
    //distance = boost_test();
    matrix_multiply(10, 10, 10, mtrx_A, mtrx_B, mtrx_C);
    chTMStopMeasurementX(&tmu_soft);

    for(size_t i=0; i<32*32; i++) {
      op0[i] = i;
      op1[i] = 1;
      res[i] = i+10.1;
      dbg0[i] = 666;
      dbg1[i] = 666;
      dbg2[i] = 666;
      dbg3[i] = 666;
      dbg4[i] = 666;
    }

    uint8_t m = 2;
    uint8_t p = 2;
    uint8_t n = 2;
    //ctl[CTL_SIZES] = (n << 10) | (p << 5) | (m << 0);
    ctl[CTL_SIZES] = 0;
    ctl[CTL_OP]    = (1 << 15) | (0 << 9) | (3 << 6) | (1 << 3) | (0 << 0);

    chTMStartMeasurementX(&tmu_hard);
    while (! FSMCDataFlushed())
      ;

    while(! FPGAMulRdy())
      green_led_toggle();
    chTMStopMeasurementX(&tmu_hard);

    tmp = res[2];
  }

  green_led_off();
  red_led_off();
  orange_led_off();
}
