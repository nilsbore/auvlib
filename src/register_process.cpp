#include <Eigen/Dense>
#include <cxxopts.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/colormap.h>
#include <data_tools/submaps.h>

using namespace std;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

void train_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
    cout << "Training gaussian process..." << endl;
	double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();

    Eigen::MatrixXd X = points.leftCols(2);
	X.col(0).array() -= meanx;
	X.col(1).array() -= meany;
	Eigen::VectorXd y = points.col(2).array() - meanz;
	//gp.train_parameters(X, y);
	gp.add_measurements(X, y);

    cout << "Done training gaussian process..." << endl;
}

CloudT::Ptr construct_submap_and_gp_cloud(Eigen::MatrixXd& points, ProcessT& gp, int offset)
{
	double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();
	Eigen::Vector3f mean_vector(meanx, meany, meanz);
	
	points.col(0).array() -= meanx;
	points.col(1).array() -= meany;
	points.col(2).array() -= meanz;
    
	/*for (int i = 0; i < points.rows(); ++i) {
		if (points(i, 2) < -10.) {
            points(i, 2) = -10.;
		}
    }*/

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
	cout << "Predicted heights: " << f_star << endl;
	
	cout << "Done predicting gaussian process..." << endl;
	cout << "X size: " << maxx - minx << endl;
	cout << "Y size: " << maxy - miny << endl;
	cout << "Z size: " << maxz - minz << endl;

	Eigen::MatrixXd predicted_points(X_star.rows(), 3);
	predicted_points.leftCols(2) = X_star;
	predicted_points.col(2) = 1.*f_star;

	CloudT::Ptr cloud(new CloudT);

    for (int i = 0; i < predicted_points.rows(); ++i) {
        PointT p;
        p.getVector3fMap() = predicted_points.row(i).cast<float>().transpose() + mean_vector;
        p.r = colormap[offset][0];
	    p.g = colormap[offset][1];
		p.b = colormap[offset][2];
		cloud->push_back(p);
    }
    for (int i = 0; i < points.rows(); ++i) {
        PointT p;
        p.getVector3fMap() = points.row(i).cast<float>().transpose() + mean_vector;
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

void get_transform_jacobian(Eigen::MatrixXd& J, const Eigen::Vector3d& x)
{
    J.block<3, 3>(0, 0).setIdentity();
    J(0, 3) = -x(1);
    J(0, 5) = x(2);
    J(1, 3) = x(0);
    J(1, 4) = -x(2);
    J(2, 4) = x(1);
    J(2, 5) = -x(0);
}

void update(Eigen::MatrixXd& points, ProcessT& gp, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    MatrixXd dX;
    gp.compute_derivatives(dX, points.leftCols(2), points.col(2));
	R = rotations[i].toRotationMatrix();
    t = means[i];
	dX *= R.transpose();
	double added_derivatives = 0;
	Eigen::RowVectorXd delta(6);
	delta.setZero();
	Eigen::MatrixXd J(3, 6);
	// This would be really easy to do as one operation
	for (int m = 0; m < points.cols(); ++m) {
        //points.col(m) = R*points.col(m) + t;
        get_transform_jacobian(J, points.col(m));
        delta = (added_derivatives/(added_derivatives+1.))*delta + 1./(added_derivatives+1.)*dX.row(m)*J;
        ++added_derivatives;
	}
	return delta.transpose();
}

void gradient_step(Eigen::MatrixXd& delta, Matrix3d& R, Vector3d& t)
{
    double step = 1e-1;
    Matrix3d Rx = Eigen::AngleAxisd(step*delta(3), Vector3d::UnitX()).matrix();
    Matrix3d Ry = Eigen::AngleAxisd(step*delta(4), Vector3d::UnitY()).matrix();
    Matrix3d Rz = Eigen::AngleAxisd(step*delta(5), Vector3d::UnitZ()).matrix();
    R = Rx*Ry*Rz;
    t = step*delta.head<3>().transpose();
}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string folder_str;
	double lsq = 100.;
	double sigma = 1.;
	double s0 = 1.;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	//options.positional_help("[optional args]").show_positional_help();
	options.add_options()
      ("help", "Print help")
      ("folder", "Folder", cxxopts::value(folder_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("s0", "Probit noise", cxxopts::value(s0));

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
	
	ProcessT gp1(100, s0);
	gp1.kernel.sigmaf_sq = sigma;
	gp1.kernel.l_sq = lsq*lsq;
    gp1.kernel.p(0) = gp1.kernel.sigmaf_sq;
    gp1.kernel.p(1) = gp1.kernel.l_sq;
	train_gp(points1, gp1);
	
	ProcessT gp2(100, s0);
	gp2.kernel.sigmaf_sq = sigma;
	gp2.kernel.l_sq = lsq*lsq;
    gp2.kernel.p(0) = gp2.kernel.sigmaf_sq;
    gp2.kernel.p(1) = gp2.kernel.l_sq;
	train_gp(points2, gp2);

	CloudT::Ptr cloud1 = construct_submap_and_gp_cloud(points1, gp1, 0);
	CloudT::Ptr cloud2 = construct_submap_and_gp_cloud(points2, gp2, 2);

	*cloud1 += *cloud2;
	visualize_cloud(cloud1);

    return 0;
}
