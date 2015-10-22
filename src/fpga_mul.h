#ifndef FPGA_MUL_H_
#define FPGA_MUL_H_

#include "fpga.h"

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  MTRXMUL_UNINIT = 0,             /**< Not initialized.                   */
  MTRXMUL_STOP = 1,               /**< Stopped.                           */
  MTRXMUL_READY = 2,              /**< Ready.                             */
} mtrxmulstate_t;

/**
 * @brief   Forward declaration.
 */
typedef struct MtrxMul MtrxMul;

/**
 * @brief   Structure handling matrix multiplier.
 */
struct MtrxMul {
  /**
   * @brief   Command region.
   */
  fpgacmd_t       *cmd;
  /**
   * @brief   Pool for matrix data.
   */
  double          *mtrx;
  /**
   * @brief   Bitmask for free matrix regions.
   */
  uint32_t        empty;
  /**
   * @brief   Multiplicator state.
   */
  mtrxmulstate_t  state;
};

/**
 *
 */
extern MtrxMul MTRXMULD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void MulObjectInit(MtrxMul *mulp);
  void MulStart(MtrxMul *mulp, const FPGADriver *fpgap);
  void MulStop(MtrxMul *mulp);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_MUL_H_ */
