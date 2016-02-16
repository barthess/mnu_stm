#ifndef MATRIX_OSAL_HPP_
#define MATRIX_OSAL_HPP_

#include <stdint.h>
#include <stddef.h>

#ifdef _MSC_FULL_VER
#if _MSC_FULL_VER <= 180030324
#ifndef constexpr
#define constexpr inline
#endif /* constexpr */
#endif /* _MSC_FULL_VER <= 180030324 */
#endif /* _MSC_FULL_VER */

void *matrix_malloc(size_t pool_index, size_t size);
void matrix_free(size_t pool_index, void *mem);
void matrixDbgCheck(bool a);
void matrixDbgPanic(const char *msg);
void matrixDbgPrint(const char *msg);

#endif /* MATRIX_OSAL_HPP_ */
