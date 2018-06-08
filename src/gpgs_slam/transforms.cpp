#include <gpgs_slam/transforms.h>

Eigen::Matrix3d euler_to_matrix(double x, double y, double z)
{
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).matrix();
    return Rx*Ry*Rz;
}

std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > euler_to_matrices(double x, double y, double z) 
{
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).matrix();
    return {Rx, Ry, Rz};
}

std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > euler_to_diff_matrices(double x, double y, double z)
{
    //Eigen::Matrix3d Rx; Rx << 1., 0., 0., 0., cos(x), -sin(x), 0., sin(x), cos(x);
    Eigen::Matrix3d Rx; Rx << 0., 0., 0., 0., -sin(x), -cos(x), 0., cos(x), -sin(x);
    //Eigen::Matrix3d Ry; Ry << cos(x), 0., sin(x), 0., 1., 0., -sin(x), 0., cos(x);
    Eigen::Matrix3d Ry; Ry << -sin(x), 0., cos(x), 0., 0., 0., -cos(x), 0., -sin(x);
    //Eigen::Matrix3d Rz; Rz << cos(x), -sin(x), 0., sin(x), cos(x), 0., 0., 0., 1.;
    Eigen::Matrix3d Rz; Rz << -sin(x), -cos(x), 0., cos(x), -sin(x), 0., 0., 0., 0.;
    return {Rx, Ry, Rz};
}
