#ifndef BINARY_CONSTRAINT_COST
#define BINARY_CONSTRAINT_COST

#include <ceres/ceres.h>
#include <ceres/rotation.h>

class BinaryConstraintCostFunctor {
public:
    //BinaryConstraintCostFunctor(const Eigen::MatrixXd& points1, const Eigen::MatrixXd& points2, double sigma)
    BinaryConstraintCostFunctor(const Eigen::Vector3d& last_point1, const Eigen::Vector3d& first_point2, double sigma) :
        last_point1(last_point1), first_point2(first_point2), sigma(sigma)
    {
        // assuming 512 beam wide swaths
        const int swath_width = 512;
        //last_point1 = points1.row(points1.rows()-swath_width/2).transpose();
        //first_point2 = points2.row(swath_width/2).transpose();
        C = 1./(sigma*sigma)*Eigen::Matrix3d::Identity();
        denom = -3./2.*log(2.*M_PI*sigma*sigma);
    }

    template <typename T>
    bool operator()(const T* const t1p, const T* const rot1p,
                    const T* const t2p, const T* const rot2p,
                    T* e) const
    {
        

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > t1(t1p);
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > t2(t2p);
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > rot1(rot1p);
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > rot2(rot2p);
        //Eigen::Matrix<T, 3, 1> rot1 = T(180./M_PI)*rot1deg;
        //Eigen::Matrix<T, 3, 1> rot2 = T(180./M_PI)*rot2deg;
        Eigen::Matrix<T, 3, 1> angle_axis1 = rot1p[2]*Eigen::Matrix<T, 3, 1>::UnitZ();
        Eigen::Matrix<T, 3, 1> angle_axis2 = rot2p[2]*Eigen::Matrix<T, 3, 1>::UnitZ();
    
        Eigen::Matrix<T, 3, 3> R1;
        Eigen::Matrix<T, 3, 3> R2;
        ceres::AngleAxisToRotationMatrix(angle_axis1.data(), R1.data());
        ceres::AngleAxisToRotationMatrix(angle_axis2.data(), R2.data());
        //Eigen::Matrix3d R1 = Eigen::AngleAxis<T>(rot1[2], Eigen::Matrix<T, 3, 1>::UnitZ()).matrix();
        //Eigen::Matrix3d R2 = Eigen::AngleAxis<T>(rot2[2], Eigen::Matrix<T, 3, 1>::UnitZ()).matrix();

        // this gives us Rz*Ry*Rx, which is fine for now but we need to change the other code
        //ceres::EulerAnglesToRotationMatrix(rot1.data(), 3, R1.data());
        //ceres::EulerAnglesToRotationMatrix(rot2.data(), 3, R2.data());

        // NOTE: casting to Jet incurs performance penalty for differentiating.
        // In newer versions of Ceres, this is not necessary.
        // If we want to work around this, we need to use Ceres mult functions
        Eigen::Matrix<T, 3, 1> diff = t1 + R1*last_point1.cast<T>() - (t2 + R2*first_point2.cast<T>());
        //Eigen::Matrix<T, 3, 1> diff = t1 + last_point1.cast<T>() - (t2 + first_point2.cast<T>());

        //T nom = -diff.transpose()*C.cast<T>()*diff;
        //T cost = -denom - 0.5*nom;
        //T cost = -0.5*nom;
        //std::cout << "Computing derivatives, value: " << cost << std::endl;
        
        //e[0] = cost;
        e[0] = 1./sigma*diff[0];
        e[1] = 1./sigma*diff[1];

        return true;
    }
    
    //static ceres::CostFunction* Create(const Eigen::MatrixXd& points1,
    //                                   const Eigen::MatrixXd& points2,
    static ceres::CostFunction* Create(const Eigen::Vector3d& last_point1,
                                       const Eigen::Vector3d& first_point2,
                                       double sigma)
    {
        return new ceres::AutoDiffCostFunction<BinaryConstraintCostFunctor, 2, 3, 3, 3, 3>(
        //return new ceres::AutoDiffCostFunction<BinaryConstraintCostFunctor, 1, 3, 3, 3, 3>(
            new BinaryConstraintCostFunctor(last_point1, first_point2, sigma));
    }

private:
    Eigen::Vector3d last_point1;
    Eigen::Vector3d first_point2;
    Eigen::Matrix3d C;
    double sigma;
    double denom;
};

#endif // BINARY_CONSTRAINT_COST
