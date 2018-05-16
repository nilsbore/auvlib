#include <gpgs_slam/multi_visualizer.h>
#include <gpgs_slam/transforms.h>
#include <data_tools/submaps.h>
#include <data_tools/colormap.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <gpgs_slam/visualization.h>

using namespace std;

MultiVisCallback::MultiVisCallback(ObsT& points, SubmapsGPT& gps, TransT& trans, AngsT& rots)
    : viewer("Simple Cloud Viewer"), points(points), gps(gps), trans(trans), rots(rots)
{

    for (int i = 0; i < points.size(); ++i) {
        Eigen::Matrix3d RM = euler_to_matrix(rots[i](0), rots[i](1), rots[i](2));
        CloudT::Ptr cloud = construct_submap_and_gp_cloud(points[i], gps[i], trans[i], RM, 2*i);
        viewer.showCloud(cloud, string("cloud")+to_string(i));
    }
    vis = cv::imread("temp.png");

    /*
    vis = visualize_likelihoods(t2, RM2);
    t0 = t1;
    old_point = cv::Point(vis.cols/2+0, vis.rows/2+0);

    step_offset = 15.;
    factor = 20.;
    */
    cv::imshow("registration", vis);
    cv::waitKey(0);
}

void MultiVisCallback::visualizer_step(vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >& RMs)
{
    /*
    Vector3d rt = t1 - t0;
    cv::Point new_point(vis.cols/2+int(factor*(rt(0)/step_offset+0.5)), vis.rows/2+int(factor*(rt(1)/step_offset+0.5)));
    cv::line(vis, old_point, new_point, cv::Scalar(0, 0, 255)); //, int thickness=1, int lineType=8, int shift=0)
    old_point = new_point;
    //cv::waitKey(10);
    */

    cout << "Visualizing step" << endl;
    //Eigen::MatrixXd points3 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
    for (int i = 0; i < points.size(); ++i) {
        CloudT::Ptr cloud = construct_submap_and_gp_cloud(points[i], gps[i], trans[i], RMs[i], 2*i);
        viewer.removeVisualizationCallable(string("cloud")+to_string(i));
        viewer.showCloud(cloud, string("cloud")+to_string(i));
    }
    
    //cv::imshow("registration", vis);
    //cv::waitKey(0);
}

ceres::CallbackReturnType MultiVisCallback::operator()(const ceres::IterationSummary& summary)
{
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > RMs;
    for (const Eigen::Vector3d& rot : rots) {
        RMs.push_back(euler_to_matrix(rot(0), rot(1), rot(2)));
    }
    visualizer_step(RMs);
    return ceres::SOLVER_CONTINUE;
}
