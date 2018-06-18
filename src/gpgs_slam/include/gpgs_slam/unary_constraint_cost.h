#ifndef UNARY_CONSTRAINT_COST
#define UNARY_CONSTRAINT_COST

#include <ceres/ceres.h>
//#include <ceres/rotation.h>

class UnaryConstraintCostFunctor {
public:

    UnaryConstraintCostFunctor(const Eigen::Vector3d& r0, double sigma) : r0(r0), sigma(sigma)
    {
        const int swath_width = 512;
        C = 1./(sigma*sigma)*Eigen::Matrix3d::Identity();
        denom = -3./2.*log(2.*M_PI*sigma*sigma);
    }

    template <typename T>
    bool operator()(const T* const rot1, T* e) const
    {
        e[0] = 1./sigma*(rot1[0] - r0[0]);
        e[1] = 1./sigma*(rot1[1] - r0[1]);
        e[2] = 1./sigma*(rot1[2] - r0[2]);
        
        
        //T diff = rot1[2] - r0[2];
        //T prob = -.5/(sigma*sigma)*diff*diff;
        
        //std::cout << "Computing derivatives, value: " << prob << std::endl;
        //e[0] = -prob;

        return true;
    }
    
    static ceres::CostFunction* Create(const Eigen::Vector3d& r0, double sigma)
    {
        //return new ceres::AutoDiffCostFunction<UnaryConstraintCostFunctor, 1, 3>(
        return new ceres::AutoDiffCostFunction<UnaryConstraintCostFunctor, 3, 3>(
            new UnaryConstraintCostFunctor(r0, sigma));
    }

private:
    Eigen::Vector3d r0;
    Eigen::Matrix3d C;
    double sigma;
    double denom;
};

#endif // UNARY_CONSTRAINT_COST
