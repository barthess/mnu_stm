namespace matrix {

double * fpga_alloc(size_t *fpga_idx, size_t size);
void fpga_free(const double *data, size_t pool_idx);
void fpga_evict(size_t N);
void fpga_settle(double **ram, size_t m, size_t n, const size_t *ram_idx);

} // namespace

