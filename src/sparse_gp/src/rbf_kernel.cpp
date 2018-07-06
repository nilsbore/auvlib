#include <sparse_gp/rbf_kernel.h>

#include <iostream> // DEBUG

using namespace Eigen;

rbf_kernel::rbf_kernel(double sigmaf_sq, double l_sq) :
    p(2), sigmaf_sq(sigmaf_sq), l_sq(l_sq)
{
    p(0) = sigmaf_sq;
    p(1) = l_sq;
}

// squared exponential coviariance, should use matern instead
double rbf_kernel::kernel_function(const Vector2d& xi, const Vector2d& xj)
{
    return p(0)*exp(-0.5f / p(1) * (xi - xj).squaredNorm());
}


// the differential kernel vector with respect to x
void rbf_kernel::kernel_dx(MatrixXd& k_dx, const VectorXd& x, const MatrixXd& BV)
{
    k_dx.resize(BV.cols(), x.rows());
    RowVectorXd offset;
    for (int i = 0; i < BV.cols(); ++i) {
        offset =  (x - BV.col(i)).transpose();
        k_dx.row(i) = p(0)/p(1)*offset*exp(-0.5f/p(1)*offset.squaredNorm()); // NOTE: this was negative before!
    }
}

// the differential kernel vector with respect to x
void rbf_kernel::kernel_dtheta(MatrixXd& k_dtheta, const Vector2d& x, const MatrixXd& BV)
{
    k_dtheta.resize(BV.cols(), 2);
    double offset;
    for (int i = 0; i < BV.cols(); ++i) {
        offset =  (x - BV.col(i)).squaredNorm();
        k_dtheta(i, 0) = exp(-0.5f/p(1)*offset);
        k_dtheta(i, 1) = p(0)*0.5f/(p(1)*p(1))*offset*k_dtheta(i, 0);
    }
}

int rbf_kernel::param_size()
{
    return 2;
}

VectorXd& rbf_kernel::param()
{
    return p;
}

VectorXd rbf_kernel::param() const
{
    return p;
}

void rbf_kernel::dKdtheta(std::vector<ArrayXXd>& Ks, const MatrixXd& BV)
{
    std::cout << "BV size: " << BV.cols() << std::endl;
    Ks[0].resize(BV.cols(), BV.cols());
    Ks[1].resize(BV.cols(), BV.cols());
    double temp;
    for (int i = 0; i < BV.cols(); ++i) {
        for (int j = 0; j <= i; ++j) {
            temp = -0.5f * (BV.col(i) - BV.col(j)).squaredNorm();
            Ks[0](i, j) = exp(temp / p(1));
            Ks[0](j, i) = Ks[0](i, j);
            Ks[1](i, j) = -temp/(p(1)*p(1))*p(0)*Ks[0](i, j);
            Ks[1](j, i) = Ks[1](i, j);
            //std::cout << Ks[1](i, j);
        }
    }
}

//void rbf_kernel::kernel_dtheta(MatrixXd& k_dtheta, )
