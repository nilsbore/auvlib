#include <gpgs_slam/transforms.h>

Eigen::Matrix3d euler_to_matrix(double x, double y, double z)
{
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).matrix();
    return Rx*Ry*Rz;
}
