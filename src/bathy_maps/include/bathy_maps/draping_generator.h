#ifndef DRAPING_GENERATOR_H
#define DRAPING_GENERATOR_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>

#include <eigen3/Eigen/Dense>

#include <data_tools/xtf_data.h>
#include <data_tools/csv_data.h>

struct draping_generator {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    igl::opengl::glfw::Viewer viewer;
    const xtf_sss_ping::PingsT& pings;
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

    draping_generator(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
        const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
        const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset,
        const csv_asvp_sound_speed::EntriesT& sound_speeds = csv_asvp_sound_speed::EntriesT(),
        double sensor_yaw = 0.);

    void launch();
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Vector3d> project_sss();
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    void set_ray_tracing_enabled(bool enabled);
};

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd> get_vehicle_mesh();
Eigen::MatrixXd color_jet_from_mesh(const Eigen::MatrixXd& V);
void generate_draping(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                      const draping_generator::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                      const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw);

#endif // DRAPING_VIEWER_H
