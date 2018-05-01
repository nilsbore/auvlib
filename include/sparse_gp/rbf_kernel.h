#ifndef RBF_KERNEL_H
#define RBF_KERNEL_H

#include <Eigen/Dense>
#include <vector>

class rbf_kernel
{
private:
    double sigmaf_sq;
    double l_sq; // length parameter, how far points influence each other
    Eigen::VectorXd p;
public:
    int param_size();
    Eigen::VectorXd& param();
    Eigen::VectorXd param() const;
    void dKdtheta(std::vector<Eigen::ArrayXXd>& Ks, const Eigen::MatrixXd& BV);
    double kernel_function(const Eigen::Vector2d& xi, const Eigen::Vector2d& xj);
    void kernel_dtheta(Eigen::MatrixXd& k_dtheta, const Eigen::Vector2d& x, const Eigen::MatrixXd& BV);
    void kernel_dx(Eigen::MatrixXd& k_dx, const Eigen::VectorXd& x, const Eigen::MatrixXd& BV);
    void kernels_fast(Eigen::ArrayXXd& K_dx, Eigen::ArrayXXd& K_dy, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV);
    void construct_covariance_fast(Eigen::MatrixXd& K, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV);
    //rbf_kernel(double sigmaf_sq = 1e-0f, double l_sq = 0.05*0.05);
    rbf_kernel(double sigmaf_sq = 100e-0f, double l_sq = 1*1);
};

#endif // RBF_KERNEL_H
