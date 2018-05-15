#ifndef GPGS_COST_FUNCTION_H
#define GPGS_COST_FUNCTION_H

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>

using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;

// first parameter block, translation 1, then rotation 1, translation 2 and rotation 2
class GaussianProcessCostFunction : public ceres::SizedCostFunction<1, 3, 3, 3, 3> {
private:
    ProcessT& gp1;
	Eigen::MatrixXd& points2;
public:

    GaussianProcessCostFunction(ProcessT& gp1, Eigen::MatrixXd& points2) : gp1(gp1), points2(points2)
    {
        
    }

    virtual ~GaussianProcessCostFunction() {}

    void get_transform_jacobian(Eigen::MatrixXd& J, const Eigen::Vector3d& x) const;
	// somehow, the parameters must contain the transform both for 1 and 2
	// parameters: translation1, rotation1, translation2, rotation2, dim 2*(3+3)
	// residuals: neg-log-likelihood of points2 with respect to gp1 and transforms, dim 1
	// jacobians: derivatives of neg-log-likelihood w.r.t. parameters, dim 1*2*(3+3)
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
};

#endif // GPGS_COST_FUNCTION_H
