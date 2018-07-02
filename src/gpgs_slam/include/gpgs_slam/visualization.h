#ifndef GPGS_VISUALIZATION_H
#define GPGS_VISUALIZATION_H

#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

#include <opencv2/core.hpp>

using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

class VisCallback : public ceres::IterationCallback {
private:
    pcl::visualization::CloudViewer viewer; 
    Eigen::MatrixXd& points1;
    Eigen::MatrixXd& points2;
    ProcessT& gp1;
    Eigen::Vector3d& t1;
    Eigen::Vector3d& R1;
    cv::Mat vis;
    cv::Point old_point;
    double step_offset;
    double factor;
    Eigen::Vector3d t0;
public:
    cv::Mat visualize_likelihoods(Eigen::Vector3d& t2, Eigen::Matrix3d& R2);
    explicit VisCallback(Eigen::MatrixXd& points1, Eigen::MatrixXd& points2,
                         ProcessT& gp1, ProcessT& gp2,
                         Eigen::Vector3d& t1, Eigen::Vector3d& R1,
                         Eigen::Vector3d& t2, Eigen::Vector3d& R2);
    ~VisCallback() {}

    void visualizer_step(Eigen::Matrix3d& RM1);
    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
};

CloudT::Ptr construct_submap_and_gp_cloud(Eigen::MatrixXd points, ProcessT& gp,
				                          Eigen::Vector3d& t, Eigen::Matrix3d& R,
										  int offset);

#endif // GPGS_VISUALIZATION_H
