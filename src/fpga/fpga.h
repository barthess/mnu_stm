#ifndef FPGA_H_
#define FPGA_H_

#include "fsmc_sram.h"

typedef uint16_t        fpgaword_t;         /* fpga talks with stm32 using 16-bit words */

#define FPGA_WB_SLICE_SIZE   65536    /* address space size single wishbone slice in fpga_words */
#define FPGA_WB_SLICE_CNT    8        /* total number of slices */

/* current FPGA firmware limitations */
#define FPGA_MTRX_MAX_ROW   32
#define FPGA_MTRX_MAX_COL   32

/* IDs of command slices for differ peripherals */
#define FPGA_WB_SLICE_MEMTEST     0
#define FPGA_WB_SLICE_LED         1
#define FPGA_WB_SLICE_MUL_OP1     2
#define FPGA_WB_SLICE_MUL_OP2     3
#define FPGA_WB_SLICE_MUL_RES     4
#define FPGA_WB_SLICE_MUL_CTRL    5
#define FPGA_WB_SLICE_RESERVED1   6
#define FPGA_WB_SLICE_RESERVED2   7

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FPGA_UNINIT = 0,                  /**< Not initialized.                   */
  FPGA_STOP = 1,                    /**< Stopped.                           */
  FPGA_READY = 2,                   /**< Ready.                             */
} fpgastate_t;

/**
 *
 */
typedef struct FPGADriver FPGADriver;

/**
 * @brief   Structure handling matrix multiplier.
 */
struct FPGADriver {
  fpgaword_t  *memspace;
  fpgastate_t state;
};

/**
 *
 */
extern FPGADriver FPGAD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgaObjectInit(FPGADriver *fpgap);
  void fpgaStart(FPGADriver *fpgap);
  void fpgaStop(FPGADriver *fpgap);
  fpgaword_t * fpgaGetCmdSlice(const FPGADriver *fpgap, size_t N);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_H_ */


