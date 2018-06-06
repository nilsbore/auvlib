#ifndef GPGS_TRANSFORMS_H
#define GPGS_TRANSFORMS_H

#include <Eigen/Dense>

Eigen::Matrix3d euler_to_matrix(double x, double y, double z);

std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > euler_to_matrices(double x, double y, double z);

std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > euler_to_diff_matrices(double x, double y, double z);

#endif // GPGS_TRANSFORMS_H
