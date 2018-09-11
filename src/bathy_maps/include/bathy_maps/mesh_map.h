#ifndef MESH_MAP_H
#define MESH_MAP_H

#include <eigen3/Eigen/Dense>
#include <data_tools/data_structures.h>
#include <data_tools/xtf_data.h>

class bathy_map_mesh {
public:
    using BoundsT = Eigen::Matrix2d;

    std::pair<Eigen::MatrixXd, Eigen::MatrixXi> mesh_from_height_map(const Eigen::MatrixXd& height_map, const BoundsT& bounds);
    std::pair<Eigen::MatrixXd, BoundsT> height_map_from_pings(const mbes_ping::PingsT& pings, double res);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_pings(const mbes_ping::PingsT& pings, double res=0.5);
    void display_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    void display_height_map(const Eigen::MatrixXd& height_map);
    Eigen::MatrixXd overlay_sss(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const BoundsT& bounds, const xtf_sss_ping::PingsT& pings);
};

#endif // MESH_MAP_H