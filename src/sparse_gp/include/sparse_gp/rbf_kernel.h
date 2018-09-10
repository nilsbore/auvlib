#ifndef RBF_KERNEL_H
#define RBF_KERNEL_H

#include <eigen3/Eigen/Dense>
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
    void construct_covariance_fast(Eigen::MatrixXd& K, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV)
    {
        K.resize(X.rows(), BV.cols());
        Eigen::MatrixXd rep;
        Eigen::ArrayXd temp;
        for (int i = 0; i < BV.cols(); ++i) {
            rep = BV.col(i).transpose().replicate(X.rows(), 1); // typically more cols in X than in Xb
            temp = (X - rep).rowwise().squaredNorm();
            K.col(i) = p(0)*(-0.5f/p(1)*temp).exp(); // OK
        }
    }
    
    // fast computation of kernel derivatives
    template <int N>
    void kernel_dx_fast(Eigen::MatrixXd& K_dx1, Eigen::MatrixXd& K_dx2, const Eigen::MatrixXd& X, const Eigen::MatrixXd& BV)
    {
        K_dx1.resize(X.rows(), BV.cols()); // OK
        K_dx2.resize(X.rows(), BV.cols()); // OK

        Eigen::ArrayXXd offset;
        Eigen::ArrayXd exppart;
        for (int i = 0; i < BV.cols(); ++i) {
            offset = X - BV.col(i).transpose().replicate(X.rows(), 1); // OK
            exppart = -0.5f/p(1)*offset.matrix().rowwise().squaredNorm();
            exppart = exppart.exp(); // OK
            K_dx1.col(i) = p(0)/p(1)*offset.col(0)*exppart; // OK
            K_dx2.col(i) = p(0)/p(1)*offset.col(1)*exppart; // OK
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
