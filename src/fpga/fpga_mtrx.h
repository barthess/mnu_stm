#ifndef FPGA_MTRX_H_
#define FPGA_MTRX_H_

#include "fpga.h"

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  MTRXMUL_UNINIT = 0,             /**< Not initialized.                   */
  MTRXMUL_STOP = 1,               /**< Stopped.                           */
  MTRXMUL_READY = 2,              /**< Ready.                             */
  MTRXMUL_ACTIVE = 3,             /**< Active.                            */
} mtrxstate_t;

/**
 * @brief   Forward declaration.
 */
typedef struct Mtrx Mtrx;

/**
 * @brief   Structure handling matrix multiplier.
 */
struct Mtrx {
  /**
   * @brief   Pointer to FPGA driver.
   */
  const FPGADriver  *fpgap;
  /**
   * @brief   Operation word pointer.
   */
  fpgaword_t        *op;
  /**
   * @brief   Sizes word pointer.
   */
  fpgaword_t        *sizes;
  /**
   * @brief   Scale/memset value.
   */
  double            *constant;
  /**
   * @brief   Pool for matrix data.
   */
  double            *pool[FPGA_MTRX_BRAMS_CNT];
  /**
   * @brief   Bitmask for free matrix regions.
   */
  uint32_t          empty;
  /**
   * @brief   Multiplicator state.
   */
  mtrxstate_t       state;
};

/**
 *
 */
extern Mtrx MTRXD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgaMtrxObjectInit(Mtrx *mtrxp);
  void fpgaMtrxStart(Mtrx *mtrxp, const FPGADriver *fpgap);
  void fpgaMtrxStop(Mtrx *mtrxp);
  double* fpgaMtrxMalloc(Mtrx *mtrxp);
  void fpgaMtrxFree(Mtrx *mtrxp, double *slice);
  void fpgaMtrxDot(Mtrx *mtrxp, size_t m, size_t p, size_t n,
                    const double *A, const double *B, double *C);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_MTRX_H_ */
