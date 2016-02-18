#ifndef MATRIX_MEM_MGR_HPP
#define MATRIX_MEM_MGR_HPP

namespace matrix {

#ifdef __cplusplus
extern "C" {
#endif
  void * fpga_malloc(size_t *slice_idx, size_t size);
  void fpga_free(void *data, size_t slice_idx);
  void fpga_evict(size_t idx);
  size_t fpga_evict_prediction(void);
  void fpga_settle(double **ram, size_t m, size_t n, const size_t *ram_idx);
#ifdef __cplusplus
}
#endif

} // namespace

#endif // MATRIX_MEM_MGR_HPP
