#ifndef MATRIX_MEM_POOL_HPP_
#define MATRIX_MEM_POOL_HPP_

namespace matrix {

void * pool_malloc(size_t pool_index, size_t size);
void pool_free(void *mem, size_t pool_index);

} // namespace

#endif /* MATRIX_MEM_POOL_HPP_ */
