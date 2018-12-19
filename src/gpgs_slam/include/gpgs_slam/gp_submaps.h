#ifndef GP_SUBMAPS_H
#define GP_SUBMAPS_H

#include <data_tools/std_data.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

struct gp_submaps : public std_data::pt_submaps
{
    using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
    using SubmapsGPT = std::vector<ProcessT>; // Process does not require aligned allocation as all matrices are dynamic

    SubmapsGPT gps;
    
    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(cereal::base_class<pt_submaps>(this), CEREAL_NVP(gps));
    }
};

#endif // GP_SUBMAPS_H
