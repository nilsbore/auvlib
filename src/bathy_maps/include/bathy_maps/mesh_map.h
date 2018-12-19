#ifndef MESH_MAP_H
#define MESH_MAP_H

#include <eigen3/Eigen/Dense>
#include <data_tools/std_data.h>
#include <data_tools/xtf_data.h>

namespace mesh_map {

    using BoundsT = Eigen::Matrix2d;

    std::pair<Eigen::MatrixXd, Eigen::MatrixXi> mesh_from_height_map(const Eigen::MatrixXd& height_map, const BoundsT& bounds);
    std::pair<Eigen::MatrixXd, BoundsT> height_map_from_pings(const std_data::mbes_ping::PingsT& pings, double res);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_pings(const std_data::mbes_ping::PingsT& pings, double res=0.5);
    void show_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    void show_height_map(const Eigen::MatrixXd& height_map);

}

#endif // MESH_MAP_H
