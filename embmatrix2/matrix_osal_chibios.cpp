#include "hal.h"

#include "matrix_mempool.hpp"

#define MATRIX_HARD_SIZE_LIMIT

/**
 *
 */
void * matrix_malloc(size_t *pool_index, size_t size) {

}

/**
 *
 */
void * matrix_malloc_soft(size_t pool_index, size_t size) {
  void *ret;

  osalDbgCheck(pool_index < MATRIX_MEMPOOL_LEN);
  osalDbgCheck(pool_array[pool_index].mp_object_size >= size);
  ret = chPoolAlloc(&pool_array[pool_index]);

  osalDbgCheck(NULL != ret);

  return ret;
}

void matrix_free(size_t pool_index, void *mem) {
  if (NULL != mem){
    chPoolFree(&pool_array[pool_index], mem);
  }
}

void matrixDbgCheck(bool a){
  osalDbgCheck(a);
}

void matrixDbgPanic(const char *msg){
  osalSysHalt(msg);
}

void matrixDbgPrint(const char *msg){
  (void)msg;
}
