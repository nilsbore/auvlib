#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <Eigen/Dense>
#include <cxxopts.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>
#include <sparse_gp/colormap.h>

using namespace std;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using SubmapsT = vector<vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;

Eigen::MatrixXd read_submap(const boost::filesystem::path& filename)
{
	Eigen::MatrixXd points;
	points.resize(90000, 3);
	std::ifstream infile(filename.string());
	std::string line;
	int i = 0;
	int counter = 0;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		double x, y, z;
		if (!(iss >> x >> y >> z)) {
			break;
		} // error
		if (counter % 13 != 0) {
		    ++counter;
		    continue;
		}
		if (i >= points.rows()) {
			points.conservativeResize(points.rows() + 90000, 3);
		}
		points.row(i) << x, y, z;
		++counter;
		++i;
	}
	points.conservativeResize(i, 3);
	return points;
}

SubmapsT read_submaps(const boost::filesystem::path& folder)
{
    SubmapsT submaps;

	bool should_break = false;
	for (int ii = 0; !should_break; ++ii) {
		for (int jj = 0; ; ++jj) {
			stringstream ss;
			ss << "patch_";
			ss << setfill('0') << setw(2) << ii << "_";
			ss << setfill('0') << setw(2) << jj;
			ss << ".xyz";
			boost::filesystem::path filename = folder / ss.str();
			cout << "Processing " << filename.string() << endl;
			if (!boost::filesystem::exists(filename)) {
				if (jj == 0) {
					should_break = true;
				}
				break;
			}
			Eigen::MatrixXd points = read_submap(filename);
			if (jj == 0) {
                submaps.push_back(vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >());
            }
			submaps.back().push_back(points);
		}
	}

	return submaps;
}

void visualize_submaps(SubmapsT& submaps)
{
	CloudT::Ptr cloud(new CloudT);

	int i = 0;
    for (int ii = 0; ii < submaps.size(); ++ii) {
        for (int jj = 0; jj < submaps[ii].size(); ++jj) {
		    PointT p;
			p.getVector3fMap() = submaps[ii][jj].row(i).cast<float>();
		    p.r = colormap[i % 44][0];
		    p.g = colormap[i % 44][1];
		    p.b = colormap[i % 44][2];
		    cloud->push_back(p);
            ++i;
		}
    }

	cout << "Done constructing point cloud, starting viewer..." << endl;

	//... populate cloud
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}

void visualize_submap(Eigen::MatrixXd& points)
{
	CloudT::Ptr cloud(new CloudT);

    for (int i = 0; i < points.rows(); ++i) {
		PointT p;
		p.getVector3fMap() = points.row(i).cast<float>();
		p.r = colormap[0][0];
		p.g = colormap[0][1];
		p.b = colormap[0][2];
		cloud->push_back(p);
    }

	cout << "Done constructing point cloud, starting viewer..." << endl;

	//... populate cloud
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}

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

void visualize_submap_and_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
	double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();
	
	points.col(0).array() -= meanx;
	points.col(1).array() -= meany;
	points.col(2).array() -= meanz;
    
	for (int i = 0; i < points.rows(); ++i) {
		if (points(i, 2) < -10.) {
            points(i, 2) = -10.;
		}
    }

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
        p.getVector3fMap() = predicted_points.row(i).cast<float>();
        p.r = colormap[0][0];
	    p.g = colormap[0][1];
		p.b = colormap[0][2];
		cloud->push_back(p);
    }
    for (int i = 0; i < points.rows(); ++i) {
        PointT p;
        p.getVector3fMap() = points.row(i).cast<float>();
        p.r = colormap[1][0];
	    p.g = colormap[1][1];
		p.b = colormap[1][2];
		cloud->push_back(p);
    }

	cout << "Done constructing point cloud, starting viewer..." << endl;

	//... populate cloud
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string folder_str;
	double lsq = 800.;
	double sigma = 10.;
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
	
	Eigen::MatrixXd points = read_submap(folder / "patch_00_00.xyz");
	//points = 0.1/930.*points;
	//visualize_submap(points);
	
	ProcessT gp(100, s0);
	gp.kernel.sigmaf_sq = sigma;
	gp.kernel.l_sq = lsq*lsq;
    gp.kernel.p(0) = gp.kernel.sigmaf_sq;
    gp.kernel.p(1) = gp.kernel.l_sq;
	train_gp(points, gp);

	visualize_submap_and_gp(points, gp);

    return 0;
}
