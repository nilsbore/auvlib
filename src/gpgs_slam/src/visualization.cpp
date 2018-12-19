#include <gpgs_slam/visualization.h>
#include <data_tools/transforms.h>
#include <data_tools/submaps.h>
#include <data_tools/colormap.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace submaps;
using namespace data_transforms;

CloudT::Ptr construct_submap_and_gp_cloud(Eigen::MatrixXd points, ProcessT& gp,
				                          Eigen::Vector3d& t, Eigen::Matrix3d& R,
										  int offset)
{
	double maxx = points.col(0).maxCoeff();
	double minx = points.col(0).minCoeff();
	double maxy = points.col(1).maxCoeff();
	double miny = points.col(1).minCoeff();
	double maxz = points.col(2).maxCoeff();
	double minz = points.col(2).minCoeff();

	cout << "Max z: " << maxz << ", Min z: " << minz << endl;

	int sz = 50;
	double xstep = (maxx - minx)/float(sz-1);
	double ystep = (maxy - miny)/float(sz-1);
    
	cout << "Predicting gaussian process..." << endl;

	Eigen::MatrixXd X_star(sz*sz, 2);
    Eigen::VectorXd f_star(sz*sz); // mean?
    f_star.setZero();
	Eigen::VectorXd V_star; // variance?
    int i = 0;
    for (int y = 0; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < sz; ++x) {
		    //if (i % 100 != 0) {
            //    continue;
            //}
		    //int ind = x*sz + y;
		    /*if (!W(ind, i)) {
			    continue;
		    }*/
		    X_star(i, 0) = minx + x*xstep;
		    X_star(i, 1) = miny + y*ystep;
		    ++i;
	    }
    }
    //X_star.conservativeResize(i, 2);
    gp.predict_measurements(f_star, X_star, V_star);
	//cout << "Predicted heights: " << f_star << endl;
	
	cout << "Done predicting gaussian process..." << endl;
	cout << "X size: " << maxx - minx << endl;
	cout << "Y size: " << maxy - miny << endl;
	cout << "Z size: " << maxz - minz << endl;

	Eigen::MatrixXd predicted_points(X_star.rows(), 3);
	predicted_points.leftCols(2) = X_star;
	predicted_points.col(2) = 1.*f_star;

	CloudT::Ptr cloud(new CloudT);

	predicted_points *= R.transpose();

    for (int i = 0; i < predicted_points.rows(); ++i) {
        PointT p;
        p.getVector3fMap() = predicted_points.row(i).cast<float>().transpose() + t.cast<float>();
        p.r = colormap[offset][0];
	    p.g = colormap[offset][1];
		p.b = colormap[offset][2];
		cloud->push_back(p);
    }

    points *= R.transpose();

    for (int i = 0; i < points.rows(); ++i) {
        PointT p;
        p.getVector3fMap() = points.row(i).cast<float>().transpose() + t.cast<float>();
        p.r = colormap[offset+1][0];
	    p.g = colormap[offset+1][1];
		p.b = colormap[offset+1][2];
		cloud->push_back(p);
    }

	return cloud;
}

cv::Mat VisCallback::visualize_likelihoods(Eigen::Vector3d& t2, Eigen::Matrix3d& RM2)
{
	
    if (boost::filesystem::exists("temp.png")) {
        cv::Mat vis = cv::imread("temp.png");
        return vis;
    }
    
    Eigen::Matrix3d RM1 = euler_to_matrix(R1(0), R1(1), R1(2));

    int sz = 10;
    cv::Mat float_image = cv::Mat::zeros(2*sz, 2*sz, CV_32FC1);
    double step_offset = 1.;
    for (int y = -sz; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = -sz; x < sz; ++x) {
            Eigen::Vector3d tt = t1 + Eigen::Vector3d(x*step_offset, y*step_offset, 0.);
            Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, RM2, tt, RM1, 465);
            Eigen::VectorXd ll;
            Eigen::MatrixXd dX;
            gp1.compute_neg_log_derivatives_fast(ll, dX, points2in1.leftCols(2), points2in1.col(2), false);
            double mean_ll = ll.mean();
            float_image.at<float>(sz+y, sz+x) = mean_ll;
	    }
    }
    
    double min;
    double max;
    cv::minMaxIdx(float_image, &min, &max);

    cv::Mat adjMap;
    cv::convertScaleAbs(float_image-min, adjMap, 255. / max);

    cv::Mat large, color;
    double factor = 20.;
    cv::resize(adjMap, large, cv::Size(factor*2*sz, factor*2*sz));//resize image
    cv::cvtColor(large, color, cv::COLOR_GRAY2BGR);

    double arrow_len = 10.;
    for (int y = -sz; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = -sz; x < sz; ++x) {
            Eigen::Vector3d tt = t1 + Eigen::Vector3d(x*step_offset, y*step_offset, 0.);
            Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, RM2, tt, RM1, 465);
            Eigen::VectorXd ll;
            Eigen::MatrixXd dX;
            cout << "Computing derivatives..." << endl;
            gp1.compute_neg_log_derivatives_fast(ll, dX, points2in1.leftCols(2), points2in1.col(2), true);
            //gp1.compute_derivatives(dX, points2in1.leftCols(2), points2in1.col(2));
            Eigen::Vector2d mean_dx = dX.colwise().mean().head<2>();
            double norm_dx = mean_dx.norm();
            mean_dx.normalize();
            Eigen::Vector2d origin(factor*(.5+sz+x), factor*(.5+sz+y));
            Eigen::Vector2d vector = origin + arrow_len*mean_dx;

            cv::Point point1(int(origin(0)), int(origin(1)));
            cv::Point point2(int(vector(0)), int(vector(1)));

            arrowedLine(color, point1, point2, cv::Scalar(0,255,0)); //, int thickness=1, int line_type=8, int shift=0, double tipLength=0.1);
	    }
    }

    cout << "Likelihoods: \n" << float_image << endl;

    cv::imshow("Out", color);
    cv::waitKey(0);
    cv::imwrite("temp.png", color);

    return color;
}

VisCallback::VisCallback(Eigen::MatrixXd& points1, Eigen::MatrixXd& points2,
                         ProcessT& gp1, ProcessT& gp2,
                         Eigen::Vector3d& t1, Eigen::Vector3d& R1,
                         Eigen::Vector3d& t2, Eigen::Vector3d& R2)
    : viewer("Simple Cloud Viewer"), points1(points1), points2(points2), gp1(gp1), t1(t1), R1(R1)
{

    Eigen::Matrix3d RM1 = euler_to_matrix(R1(0), R1(1), R1(2));
    Eigen::Matrix3d RM2 = euler_to_matrix(R2(0), R2(1), R2(2));
    vis = visualize_likelihoods(t2, RM2);
    t0 = t1;
    CloudT::Ptr cloud1 = construct_submap_and_gp_cloud(points1, gp1, t1, RM1, 0);
    CloudT::Ptr cloud2 = construct_submap_and_gp_cloud(points2, gp2, t2, RM2, 2);
    viewer.showCloud(cloud1, "cloud1");
    viewer.showCloud(cloud2, "cloud2");

    old_point = cv::Point(vis.cols/2+0, vis.rows/2+0);

    step_offset = 1.;
    factor = 20.;
}

void VisCallback::visualizer_step(Eigen::Matrix3d& RM1)
{
    Eigen::Vector3d rt = t1 - t0;
    cv::Point new_point(vis.cols/2+int(factor*(rt(0)/step_offset+0.5)), vis.rows/2+int(factor*(rt(1)/step_offset+0.5)));
    cv::line(vis, old_point, new_point, cv::Scalar(0, 0, 255)); //, int thickness=1, int lineType=8, int shift=0)
    old_point = new_point;
    //cv::waitKey(10);

    cout << "Visualizing step" << endl;
    //Eigen::MatrixXd points3 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
    CloudT::Ptr cloud1 = construct_submap_and_gp_cloud(points1, gp1, t1, RM1, 0);
    //CloudT::Ptr cloud3 = construct_cloud(points2in1, t1, R1, 4);
    
    viewer.removeVisualizationCallable("cloud1");
    viewer.showCloud(cloud1, "cloud1");
    cv::imshow("registration", vis);
    cv::waitKey(0);
}

ceres::CallbackReturnType VisCallback::operator()(const ceres::IterationSummary& summary)
{
    Eigen::Matrix3d RM1 = euler_to_matrix(R1(0), R1(1), R1(2));
    visualizer_step(RM1);
    return ceres::SOLVER_CONTINUE;
}
