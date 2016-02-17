#include <cstring>
#include <cstdint>

#include "main.h"
#include "fpga_mtrx.h"
#include "matrix_ram_pool.hpp"
#include "matrix_memory.hpp"

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
    size = 0;
    age = AGE_BORN_NOW;
  }
  double        **data;
  const size_t  *ram_idx;
  size_t        size; // bytes
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
void update_cache(double **ram, const size_t *ram_idx, size_t size, size_t fpga_idx) {
  cache[fpga_idx].data    = ram;
  cache[fpga_idx].ram_idx = ram_idx;
  cache[fpga_idx].size    = size;
  cache[fpga_idx].age     = AGE_BORN_NOW;
}

/**
 *
 */
size_t eviction_strategy(void) {
  size_t smallest = ~0;
  size_t ret = 0; // default eviction candidate

  for(size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    size_t S = cache[i].size;
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
double * fpga_alloc(size_t *fpga_idx, size_t size) {
  (void)size;

  double *ret = fpgaMtrxMalloc(&MTRXD1, fpga_idx);

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
void fpga_free(const double *data, size_t pool_idx) {
  fpgaMtrxFree(&MTRXD1, data, pool_idx);
}

/**
 * @brief   Deep move data from FPGA to RAM
 */
void fpga_evict(size_t i) {

  const double *src = fpgaMtrxDataPtr(&MTRXD1, i);
  double *dst = (double *)mempool_alloc(*cache[i].ram_idx, cache[i].size);
  deep_copy(src, dst, cache[i].size/sizeof(double));
  *cache[i].data = dst;
  fpga_free(src, i);

  // delete reference from cache registry
  cache[i].clear();
}

/**
 * @brief   Deep move data from RAM to FPGA
 */
void fpga_settle(double **ram, size_t m, size_t n, const size_t *ram_idx) {

  size_t fpga_idx;
  size_t size = m*n*sizeof(double);
  double *dst = fpga_alloc(&fpga_idx, size);

  deep_copy(*ram, dst, m*n);
  update_cache(ram, ram_idx, size, fpga_idx);
  mempool_free(*ram_idx, *ram);
}

} // namespace



