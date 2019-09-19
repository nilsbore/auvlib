#ifndef ALIGN_MAP_H
#define ALIGN_MAP_H

#include <Eigen/Dense>
#include <igl/AABB.h>
#include <data_tools/xyz_data.h>

namespace align_map {

double points_to_mesh_rmse(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                           const igl::AABB<Eigen::MatrixXd, 3>& tree);
double points_to_mesh_rmse(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >
    align_maps_icp(const std::vector<xyz_data::Points>& maps, const std::vector<int>& maps_to_align, bool align_jointly);

std::tuple<Eigen::Matrix4d, double, bool> icp_iteration(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                                        const igl::AABB<Eigen::MatrixXd, 3>& tree);

std::pair<Eigen::Matrix4d, bool> align_points_to_mesh_icp_vis(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V,
                                                              const Eigen::MatrixXi& F,
                                                              const igl::AABB<Eigen::MatrixXd, 3>& tree);

std::pair<Eigen::Matrix4d, bool> align_points_to_mesh_icp(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

std::pair<Eigen::Matrix4d, bool> align_points_to_mesh_icp(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                                          const igl::AABB<Eigen::MatrixXd, 3>& tree);

std::pair<Eigen::Matrix4d, bool> align_points_to_mesh_icp(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                                          const igl::AABB<Eigen::MatrixXd, 3>& tree,
                                                          std::function<void(const Eigen::Affine3d&)> vis_callback);

Eigen::MatrixXd filter_points_mesh_offset(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                          double offset, const igl::AABB<Eigen::MatrixXd, 3>& tree);

double compute_overlap_ratio(const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2);
void show_multiple_clouds(const std::vector<Eigen::MatrixXd>& clouds);

} // namespace align_map

#endif // ALIGN_MAP_H
