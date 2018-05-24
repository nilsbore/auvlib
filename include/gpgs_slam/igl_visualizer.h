#ifndef GPGS_IGL_VISUALIZER_H
#define GPGS_IGL_VISUALIZER_H

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <igl/opengl/glfw/Viewer.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

#include <opencv2/core.hpp>

using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using TransT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using SubmapsGPT = std::vector<ProcessT>; // Process does not require aligned allocation as all matrices are dynamic
using ObsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;

class IglVisCallback : public ceres::IterationCallback {
private:
	int sz;
    int nbr_faces;
	int nbr_vertices;
	bool updated;
    igl::opengl::glfw::Viewer viewer;
    ObsT& points;
    SubmapsGPT& gps;
    TransT& trans;
    AngsT& rots;
    Eigen::MatrixXd V; // the vertices used in the viewer
    Eigen::MatrixXd C; // the colormap of the different maps
    Eigen::MatrixXd C_jet; // the jet colormap of one map
    Eigen::MatrixXd V_orig; // the vertices used in the viewer
    Eigen::MatrixXd V_new; // the transformed vertices computed in each iteration
    Eigen::MatrixXi F; // the faces used in the viewer
	//std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > Vs; // the transformed vertices computed in each iteration
    cv::Mat vis;
    /*cv::Point old_point;
    double step_offset;
    double factor;
    Vector3d t0;*/
public:
    //cv::Mat visualize_likelihoods(Eigen::Vector3d& t2, Eigen::Matrix3d& R2);
    explicit IglVisCallback(ObsT& points, SubmapsGPT& gps, TransT& trans, AngsT& rots);
    ~IglVisCallback() {}

    void visualizer_step(std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >& RMs);
	std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> vertices_faces_from_gp(Eigen::MatrixXd& points, ProcessT& gp);
	void display();
	bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods);
    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
};

#endif // GPGS_IGL_VISUALIZER_H
