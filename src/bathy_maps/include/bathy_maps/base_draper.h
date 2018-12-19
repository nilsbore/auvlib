#ifndef BASE_DRAPER_H
#define BASE_DRAPER_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>

#include <eigen3/Eigen/Dense>

#include <data_tools/xtf_data.h>
#include <data_tools/csv_data.h>

struct BaseDraper {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    igl::opengl::glfw::Viewer viewer;
    xtf_sss_ping::PingsT pings;
    int i;
    Eigen::MatrixXd V1;
    Eigen::MatrixXi F1;
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::Vector3d offset;
    Eigen::VectorXd hit_sums;
    Eigen::VectorXi hit_counts;
    Eigen::MatrixXd N_faces; // the normals of F1, V1
    csv_asvp_sound_speed::EntriesT sound_speeds;
    double sensor_yaw;
    bool ray_tracing_enabled; // is snell ray tracing enabled?

public:

    void set_sidescan_yaw(double new_sensor_yaw) { sensor_yaw = new_sensor_yaw; }
    void set_ray_tracing_enabled(bool enabled);
    void set_vehicle_mesh(const Eigen::MatrixXd& new_V2, const Eigen::MatrixXi& new_F2, const Eigen::MatrixXd& new_C2);

    BaseDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
               const xtf_sss_ping::PingsT& pings,
               const BoundsT& bounds,
               const csv_asvp_sound_speed::EntriesT& sound_speeds = csv_asvp_sound_speed::EntriesT());

    void show();
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXi, Eigen::VectorXi, Eigen::Vector3d> project_sss();
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
};

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd> get_vehicle_mesh();
Eigen::MatrixXd color_jet_from_mesh(const Eigen::MatrixXd& V);
void drape_viewer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                  const BaseDraper::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                  const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw);

#endif // BASE_DRAPER_H
