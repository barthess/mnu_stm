#include <cstring>
#include <cstdint>

#include "main.h"
#include "fpga_mtrx.h"

#include "matrix_mem_pool.hpp"
#include "matrix_mem_mgr.hpp"

namespace matrix {

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define AGE_BORN_NOW    0

struct cache_record {
  void clear(void) {
    data = nullptr;
    ram_idx = nullptr;
    len = 0;
    age = AGE_BORN_NOW;
  }
  double        **data;
  const size_t  *ram_idx;
  size_t        len; // in elements of type double
  uint32_t      age;
};

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

cache_record cache[FPGA_MTRX_BRAMS_CNT];

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 * @brief   Deep move data from RAM to FPGA
 */
void deep_copy(const double *src, double *dst, size_t len) {

  osalDbgCheck((nullptr != src) && (dst != nullptr));

  for (size_t i=0; i<len; i++) {
    dst[i] = src[i];
  }
}

/**
 *
 */
void update_cache(double **ram, const size_t *ram_idx, size_t len, size_t fpga_idx) {
  cache[fpga_idx].data    = ram;
  cache[fpga_idx].ram_idx = ram_idx;
  cache[fpga_idx].len     = len;
  cache[fpga_idx].age     = AGE_BORN_NOW;
}

/**
 *
 */
size_t eviction_strategy(void) {
  size_t smallest = ~0;
  size_t ret = 0; // default eviction candidate

  for(size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    size_t S = cache[i].len;
    if (S < smallest) {
      smallest = S;
      ret = i;
    }
  }

  return ret;
}


/**
 * @brief   Deep move data from FPGA to RAM
 */
void fpga_evict_auto(void) {
  fpga_evict(eviction_strategy());
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void * fpga_malloc(size_t *fpga_idx, size_t size) {
  (void)size;

  void *ret = fpgaMtrxMalloc(&MTRXD1, fpga_idx);

  if (nullptr == ret) {
    fpga_evict_auto();
    ret = fpgaMtrxMalloc(&MTRXD1, fpga_idx);
    osalDbgCheck(nullptr != ret); // something broken
  }

  return ret;
}

/**
 *
 */
void fpga_free(void *data, size_t pool_idx) {
  fpgaMtrxFree(&MTRXD1, data, pool_idx);
}

/**
 * @brief   Deep move data from FPGA to RAM
 */
void fpga_evict(size_t idx) {

  double *src = fpgaMtrxDataPtr(&MTRXD1, idx);
  double *dst = (double *)pool_malloc(*cache[idx].ram_idx, cache[idx].len*sizeof(double));
  deep_copy(src, dst, cache[idx].len);
  *cache[idx].data = dst;
  fpga_free(src, idx);

  // delete reference from cache registry
  cache[idx].clear();
}

/**
 * @brief   Return number of elements to be evicted from FPGA to RAM
 * @retval  0 when there are some free slices in FPGA
 */
size_t fpga_evict_prediction(void) {

  if (fpgaMtrxHaveFreeSlice(&MTRXD1)) {
    return 0;
  }
  else {
    return cache[eviction_strategy()].len;
  }
}

/**
 * @brief   Deep move data from RAM to FPGA
 */
void fpga_settle(double **ram, size_t m, size_t n, const size_t *ram_idx) {

  size_t fpga_idx;
  size_t len = m * n;
  double *dst = (double *)fpga_malloc(&fpga_idx, len*sizeof(double));

  deep_copy(*ram, dst, m*n);
  update_cache(ram, ram_idx, len, fpga_idx);
  pool_free(*ram, *ram_idx);
}

} // namespace



