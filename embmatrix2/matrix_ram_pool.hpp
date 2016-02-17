#ifndef MATRIX_RAM_POOL_HPP_
#define MATRIX_RAM_POOL_HPP_

void * mempool_malloc(size_t pool_index, size_t size);
void mempool_free(void *mem, size_t pool_index);

#endif /* MATRIX_RAM_POOL_HPP_ */
