#ifndef GPGS_MULTI_VISUALIZER_H
#define GPGS_MULTI_VISUALIZER_H

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
using TransT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using SubmapsGPT = std::vector<ProcessT>; // Process does not require aligned allocation as all matrices are dynamic
using ObsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<MatrixXd> >;

class MultiVisCallback : public ceres::IterationCallback {
private:
    pcl::visualization::CloudViewer viewer; 
    ObsT points;
    SubmapsGPT gps;
    TransT trans;
    AngsT rots;
    /*cv::Mat vis;
    cv::Point old_point;
    double step_offset;
    double factor;
    Vector3d t0;*/
public:
    //cv::Mat visualize_likelihoods(Eigen::Vector3d& t2, Eigen::Matrix3d& R2);
    explicit MultiVisCallback(ObsT& points, SubmapsGPT& gps, TransT& trans, AngsT& rots);
    ~MultiVisCallback() {}

    void visualizer_step(std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >& RMs);
    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
};

#endif // GPGS_MULTI_VISUALIZER_H
