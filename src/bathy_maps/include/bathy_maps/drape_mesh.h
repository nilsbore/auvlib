#ifndef DRAPE_MESH_H
#define DRAPE_MESH_H

#include <eigen3/Eigen/Dense>
#include <data_tools/xtf_data.h>
#include <data_tools/csv_data.h>

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_sss_dirs(const Eigen::Matrix3d& R, double tilt_angle, double beam_width);


std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi> compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width, const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXd, Eigen::VectorXd> embree_compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width, const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1);

Eigen::MatrixXd correlate_hits(const Eigen::MatrixXd& hits_port,
                               const Eigen::VectorXi& hits_port_inds,
                               const Eigen::VectorXd& mod_port,
                               const xtf_sss_ping_side& ping,
                               const Eigen::Vector3d& origin,
                               double sound_vel,
                               const Eigen::MatrixXi& F1,
                               const csv_asvp_sound_speed::EntriesT& sound_speeds,
                               bool sound_speed_layers,
                               Eigen::MatrixXd& C,
                               Eigen::VectorXd& hit_sums,
                               Eigen::VectorXi& hit_counts,
                               bool is_left = false);

bool point_in_view(const xtf_sss_ping& ping, const Eigen::Vector3d& point, double sensor_yaw);



#endif // DRAPE_MESH_H
