#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>
#include <sparse_gp/impl/sparse_gp.hpp>

template class sparse_gp<rbf_kernel, gaussian_noise>;

