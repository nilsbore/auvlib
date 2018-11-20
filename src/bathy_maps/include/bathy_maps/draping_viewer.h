#ifndef DRAPING_VIEWER_H
#define DRAPING_VIEWER_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>
#include <igl/unproject_onto_mesh.h>

#include <eigen3/Eigen/Dense>
#include <data_tools/xtf_data.h>

struct survey_viewer {
public:

    using BoundsT = Eigen::Matrix2d;

private:

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

public:

    survey_viewer(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
        const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
        const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset);

    void launch();
    void project_sss();
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int);
    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods);
};

void overlay_sss(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                 const survey_viewer::BoundsT& bounds, const xtf_sss_ping::PingsT& pings);

#endif // DRAPING_VIEWER_H
