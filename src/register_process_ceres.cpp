#include <Eigen/Dense>
#include <cxxopts.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/colormap.h>
#include <data_tools/submaps.h>

#include <ceres/ceres.h>

using namespace std;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

void get_transform_jacobian(Eigen::MatrixXd& J, const Eigen::Vector3d& x)
{
    // Ok, let's write this out for the future
	// 3 is the X axis rotation, 4 Y axis, and 5 Z axis rotation
	// so rotating around Z(5), would give us higher Y, proportional to x(1)
	// and lower X, proportional to x(0)
	
    J.setZero();
    J.block<3, 3>(0, 0).setIdentity();
	
	// X axis rotations
    J(1, 3) = 0.*-x(2);
    J(2, 3) = 0.*x(1);
	// Y axis rotations
    J(2, 4) = 0.*-x(0);
    J(0, 4) = 0.*x(2);
	// Z axis rotations
    J(0, 5) = -x(1);
    J(1, 5) = x(0);

    J(2, 2) = 0.;
	//J.rightCols<3>() = 0.00001*J.rightCols<3>();
	J.rightCols<3>() = 0.0*J.rightCols<3>();
	
}

Eigen::Matrix3d euler_to_matrix(double x, double y, double z)
{
    Matrix3d Rx = Eigen::AngleAxisd(x, Vector3d::UnitX()).matrix();
    Matrix3d Ry = Eigen::AngleAxisd(y, Vector3d::UnitY()).matrix();
    Matrix3d Rz = Eigen::AngleAxisd(z, Vector3d::UnitZ()).matrix();
    return Rx*Ry*Rz;
}

// first parameter block, translation 1, then rotation 1, translation 2 and rotation 2
class GaussianProcessCostFunction : public ceres::SizedCostFunction<1, 3, 3, 3, 3> {
private:
    ProcessT& gp1;
	Eigen::MatrixXd& points2;
public:

    GaussianProcessCostFunction(ProcessT& gp1, Eigen::MatrixXd& points2) : gp1(gp1), points2(points2)
    {
        
    }

    virtual ~GaussianProcessCostFunction() {}

	// somehow, the parameters must contain the transform both for 1 and 2
	// parameters: translation1, rotation1, translation2, rotation2, dim 2*(3+3)
	// residuals: neg-log-likelihood of points2 with respect to gp1 and transforms, dim 1
	// jacobians: derivatives of neg-log-likelihood w.r.t. parameters, dim 1*2*(3+3)
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
    {
		Eigen::Vector3d t1(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Matrix3d R1 = euler_to_matrix(parameters[1][0], parameters[1][1], parameters[1][2]);
		Eigen::Vector3d t2(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Matrix3d R2 = euler_to_matrix(parameters[3][0], parameters[3][1], parameters[3][2]);

        cout << "Translation: " << t1.transpose() << ", Rotation: \n" << R1 << endl;

		Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
        Eigen::VectorXd ll;
        gp1.compute_neg_log_likelihoods(ll, points2in1.leftCols(2), points2in1.col(2));
		residuals[0] = -ll.mean();
		
		// Compute the Jacobian if asked for.
		if (jacobians != NULL && jacobians[0] != NULL) {
		    Eigen::MatrixXd dX;
	        cout << "Computing derivatives..." << endl;
            //gp.compute_derivatives(dX, points.leftCols(2), points.col(2));
            gp1.compute_neg_log_derivatives(dX, points2in1.leftCols(2), points2in1.col(2));
	        cout << "Done computing derivatives..." << endl;
			dX *= R1.transpose();

			Eigen::MatrixXd J(3, 6);
			Eigen::MatrixXd deltas(points2in1.rows(), 6);
			for (int m = 0; m < points2in1.rows(); ++m) {
				get_transform_jacobian(J, R1*points2in1.row(m).transpose()+t1);
				deltas.row(m) = dX.row(m)*J;
			}
			Eigen::RowVectorXd delta = deltas.colwise().mean();
			for (int j = 0; j < 6; ++j) {
                jacobians[0][j] = -delta[j];
		    }

            cout << "Derivative: " << delta << endl;
		}
		return true;
    }
};

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

	MatrixXd X_star(sz*sz, 2);
    VectorXd f_star(sz*sz); // mean?
    f_star.setZero();
	VectorXd V_star; // variance?
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

CloudT::Ptr construct_cloud(Eigen::MatrixXd points, Eigen::Vector3d& t,
				            Eigen::Matrix3d& R, int offset)
{
	CloudT::Ptr cloud(new CloudT);
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

void visualize_cloud(CloudT::Ptr& cloud)
{
	cout << "Done constructing point cloud, starting viewer..." << endl;

	//... populate cloud
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}

/*
void register_processes(Eigen::MatrixXd& points1, ProcessT& gp1, Eigen::Vector3d& t1, Eigen::Matrix3d& R1,
				        Eigen::MatrixXd& points2, ProcessT& gp2, Eigen::Vector3d& t2, Eigen::Matrix3d& R2,
                        cv::Mat& vis)
{
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    CloudT::Ptr cloud1 = construct_submap_and_gp_cloud(points1, gp1, t1, R1, 0);
    CloudT::Ptr cloud2 = construct_submap_and_gp_cloud(points2, gp2, t2, R2, 2);
	viewer.showCloud(cloud1, "cloud1");
	viewer.showCloud(cloud2, "cloud2");
	//while (!viewer.wasStopped ())
	//{
	//}

    Eigen::Vector3d rt;
    rt.setZero();
    Eigen::Matrix3d rR;
    rR.setIdentity();

    cv::Point old_point(vis.cols/2+0, vis.rows/2+0);

    Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
    VectorXd delta(6);
	delta.setZero();
    bool delta_diff_small = false;
	double step_offset = 15.;
	double factor = 20.;
    while (true) { //!delta_diff_small) {
		Eigen::VectorXd delta_old = delta;
		cout << "Computing registration delta" << endl;
		delta = compute_step(points2in1, gp1, t1, R1);
        delta_diff_small = (delta - delta_old).norm() < 1e-5f;
		Eigen::Vector3d dt;
		Eigen::Matrix3d dR;
		cout << "Computing registration update" << endl;
		tie(dt, dR) = update_step(delta);
		cout << "Registration update: " << delta << endl;
        rt += dt;

        cv::Point new_point(vis.cols/2+int(factor*(rt(0)/step_offset+0.5)), vis.rows/2+int(factor*(rt(1)/step_offset+0.5)));
        cv::line(vis, old_point, new_point, cv::Scalar(0, 0, 255)); //, int thickness=1, int lineType=8, int shift=0)
        old_point = new_point;
        //cv::waitKey(10);

        rR = dR*rR;
		R1 = dR*R1; // add to total rotation
		t1 += dt; // add to total translation
        cout << "Relative transformations: " << endl;
        cout << "T: " << rt.transpose() << endl;
        cout << "R: \n" << rR << endl;
		cout << "Getting points from submap 2 in submap 1" << endl;
        points2in1 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
	    
		cout << "Visualizing step" << endl;
		//Eigen::MatrixXd points3 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
	    CloudT::Ptr cloud1 = construct_submap_and_gp_cloud(points1, gp1, t1, R1, 0);
	    //CloudT::Ptr cloud3 = construct_cloud(points2in1, t1, R1, 4);
        
		viewer.removeVisualizationCallable("cloud1");
		viewer.showCloud(cloud1, "cloud1");
        cv::imshow("registration", vis);
		cv::waitKey(0);

    }
}
*/

cv::Mat visualize_likelihoods(ProcessT& gp1, Eigen::Vector3d& t1, Eigen::Matrix3d& R1,
                              Eigen::MatrixXd& points2, Eigen::Vector3d& t2, Eigen::Matrix3d& R2)
{
    int subsample = 37;
    int counter = 0;
    for (int i = 0; i < points2.rows(); ++i) {
        if (i % subsample == 0) {
            points2.row(counter) = points2.row(i);
            ++counter;
        }
    }
    points2.conservativeResize(counter, 3);
	
    if (boost::filesystem::exists("temp.png")) {
        cv::Mat vis = cv::imread("temp.png");
        return vis;
    }

    int sz = 10;
    cv::Mat float_image = cv::Mat::zeros(2*sz, 2*sz, CV_32FC1);
    double step_offset = 15.0;
    for (int y = -sz; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = -sz; x < sz; ++x) {
            Eigen::Vector3d tt = t1 + Eigen::Vector3d(x*step_offset, y*step_offset, 0.);
            Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, R2, tt, R1, 465);
            Eigen::VectorXd ll;
            gp1.compute_neg_log_likelihoods(ll, points2in1.leftCols(2), points2in1.col(2));
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
            Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, R2, tt, R1, 465);
            Eigen::MatrixXd dX;
            cout << "Computing derivatives..." << endl;
            gp1.compute_neg_log_derivatives(dX, points2in1.leftCols(2), points2in1.col(2));
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

void register_processes_ceres(Eigen::MatrixXd& points1, ProcessT& gp1, Eigen::Vector3d& t1, Eigen::Vector3d& R1,
                              Eigen::MatrixXd& points2, ProcessT& gp2, Eigen::Vector3d& t2, Eigen::Vector3d& R2,
                              cv::Mat& vis)
{


    ceres::Problem problem;
    //ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
    ceres::CostFunction* cost_function = new GaussianProcessCostFunction(gp1, points2);

    ceres::LossFunction* loss_function = NULL;
    problem.AddResidualBlock(cost_function, loss_function, t1.data(), R1.data(), t2.data(), R2.data());

    /*problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
        quaternion_local_parameterization);
    problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
        quaternion_local_parameterization);*/

    problem.SetParameterBlockConstant(t2.data());
    problem.SetParameterBlockConstant(R2.data());
    problem.SetParameterBlockConstant(R1.data());
    problem.SetParameterLowerBound(t1.data(), 0, t1(0) - 100.);
    problem.SetParameterLowerBound(t1.data(), 1, t1(1) - 100.);
    problem.SetParameterUpperBound(t1.data(), 0, t1(0) + 100.);
    problem.SetParameterUpperBound(t1.data(), 1, t1(1) + 100.);

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << '\n';

    std::cout << "Is usable?: " << summary.IsSolutionUsable() << std::endl;
    std::cout << "T1 final value: " << t1.transpose() << std::endl;

}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    string folder_str;
	double lsq = 100.;
	double sigma = 10.;
	double s0 = 1.;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	//options.positional_help("[optional args]").show_positional_help();
	options.add_options()
      ("help", "Print help")
      ("folder", "Folder", cxxopts::value(folder_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide folder arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
	cout << "Folder : " << folder << endl;

    //SubmapsT submaps = read_submaps(folder);
	//visualize_submaps(submaps);
	
	Eigen::MatrixXd points1 = read_submap(folder / "patch_00_00.xyz");
	Eigen::MatrixXd points2 = read_submap(folder / "patch_00_01.xyz");
	//points = 0.1/930.*points;
	//visualize_submap(points);

	Eigen::Vector3d t1, t2; // translations
    Eigen::Vector3d R1, R2; // Euler angles
    R1 << 0., 0., 0.;
    R2 << 0., 0., 0.;
	Eigen::Matrix3d RM1, RM2; // rotation matrices
	
	ProcessT gp1(100, s0);
	gp1.kernel.sigmaf_sq = sigma;
	gp1.kernel.l_sq = lsq*lsq;
    gp1.kernel.p(0) = gp1.kernel.sigmaf_sq;
    gp1.kernel.p(1) = gp1.kernel.l_sq;
	tie(t1, RM1) = train_gp(points1, gp1);
    //R1 = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).matrix();
    RM1 = euler_to_matrix(R1(0), R1(1), R1(2));
    Eigen::Vector3d t1_gt = t1;
	t1.array() += -70.0;
    t1(2) -= -70.0; 
	
	ProcessT gp2(100, s0);
	gp2.kernel.sigmaf_sq = sigma;
	gp2.kernel.l_sq = lsq*lsq;
    gp2.kernel.p(0) = gp2.kernel.sigmaf_sq;
    gp2.kernel.p(1) = gp2.kernel.l_sq;
	tie(t2, RM2) = train_gp(points2, gp2);
    RM2 = euler_to_matrix(R2(0), R2(1), R2(2));

    cv::Mat vis = visualize_likelihoods(gp1, t1, RM1, points2, t2, RM2);

	Eigen::MatrixXd points3 = get_points_in_bound_transform(points2, t2, RM2, t1, RM1, 465);

	CloudT::Ptr cloud1 = construct_submap_and_gp_cloud(points1, gp1, t1, RM1, 0);
	CloudT::Ptr cloud2 = construct_submap_and_gp_cloud(points2, gp2, t2, RM2, 2);
	CloudT::Ptr cloud3 = construct_cloud(points3, t1, RM1, 4);

	*cloud1 += *cloud2;
	*cloud1 += *cloud3;
	visualize_cloud(cloud1);
    
	register_processes_ceres(points1, gp1, t1, R1, points2, gp2, t2, R2, vis);

    cout << "T1 gt value: " << t1_gt.transpose() << endl;

    return 0;
}
