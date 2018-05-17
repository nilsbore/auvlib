#ifndef RBF_KERNEL_H
#define RBF_KERNEL_H

#include <Eigen/Dense>
#include <vector>

class rbf_kernel
{
public:
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
    //void kernels_fast(Eigen::ArrayXXd& K_dx, Eigen::ArrayXXd& K_dy, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV);
    //void construct_covariance_fast(Eigen::MatrixXd& K, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV);

    // fast computation of covariance matrix
    // This should be OK
    template <int N>
    void rbf_kernel::kernel_function_fast(Eigen::MatrixXd& K, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV)
    {
        K.resize(X.rows(), BV.cols());
        Eigen::MatrixXd rep;
        Eigen::ArrayXd temp;
        for (int i = 0; i < BV.cols(); ++i) {
            rep = BV.col(i).transpose().replicate(X.rows(), 1); // typically more cols in X than in Xb
            temp = (X - rep).rowwise().squaredNorm();
            K.col(i) = p(0)*(-0.5f/p(1)*temp).exp();
        }
    }
    
    template <int N>
    void rbf_kernel::construct_covariance_fast(Eigen::VectorXd& K_star, const Eigen::MatrixXd& X)
    {
        K.resize(BV.cols(), X.cols());
        Eigen::MatrixXd rep;
        Eigen::ArrayXd temp;
        for (int i = 0; i < BV.cols(); ++i) {
            rep = BV.col(i).replicate(1, X.cols()); // typically more cols in X than in Xb
            temp = (X - rep).colwise().squaredNorm();
            K.row(i) = p(0)*(-0.5f/p(1)*temp).exp();
        }
    }
    
    // fast computation of kernel derivatives
    template <int N>
    void rbf_kernel::kernel_dx_fast(Eigen::ArrayXXd& K_dx, Eigen::ArrayXXd& K_dy, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV)
    {
        K_dx.resize(BV.cols(), X.cols());
        K_dy.resize(BV.cols(), X.cols());
        Eigen::MatrixXd offset;
        Eigen::ArrayXd exppart;
        Eigen::ArrayXd temp;
        for (int i = 0; i < BV.cols(); ++i) {
            offset = X - BV.col(i).replicate(1, X.cols());
            temp = offset.colwise().squaredNorm();
            exppart = -p(0)/p(1)*(-0.5/p(1)*temp).exp();
            K_dx.row(i) = offset.row(0).array()*exppart.transpose();
            K_dy.row(i) = offset.row(1).array()*exppart.transpose();
        }
    }

    //rbf_kernel(double sigmaf_sq = 100e-0f, double l_sq = 1*1); // NOTE: original values
    rbf_kernel(double sigmaf_sq = 100e-0f, double l_sq = 1000*1000);

    template <class Archive>
    void serialize(Archive & ar)
    {
       ar(sigmaf_sq, l_sq, p);
    }
};

#endif // RBF_KERNEL_H
