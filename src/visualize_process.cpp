#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <Eigen/Dense>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/colormap.h>

using namespace std;
using PointT = pcl::PointXYZRGB;
using SubmapsT = vector<vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >;
using ProcessT = sparse_gp<rbf_kernel, probit_noise>;

Eigen::MatrixXd read_submap(const boost::filesystem::path& filename)
{
	Eigen::MatrixXd points;
	points.resize(90000, 3);
	std::ifstream infile(filename.string());
	std::string line;
	int i = 0;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		double x, y, z;
		if (!(iss >> x >> y >> z)) {
			break;
		} // error
		if (i >= points.rows()) {
			points.conservativeResize(points.rows() + 90000, 3);
		}
		points.row(i) << x, y, z;
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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

void train_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
    cout << "Training gaussian process..." << endl;

    Eigen::MatrixXd X = points.leftCols(2);
	Eigen::VectorXd y = points.col(2);
	gp.add_measurements(X, y);

    cout << "Done training gaussian process..." << endl;
}

void visualize_submap_and_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
	double maxx = points.col(0).maxCoeff();
	double minx = points.col(0).minCoeff();
	double maxy = points.col(1).maxCoeff();
	double miny = points.col(1).minCoeff();
	double meanz = points.col(2).mean();

	int sz = 50;
	double xstep = (maxx - minx)/float(sz-1);
	double ystep = (maxy - miny)/float(sz-1);
    
	cout << "Predicting gaussian process..." << endl;

	MatrixXd X_star;
    VectorXd f_star; // mean?
	VectorXd V_star; // variance?
	X_star.resize(sz*sz, 2);
    int i = 0;
    for (int y = 0; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < sz; ++x) {
		    //int ind = x*sz + y;
		    /*if (!W(ind, i)) {
			    continue;
		    }*/
		    X_star(i, 0) = minx + x*xstep;
		    X_star(i, 1) = miny + y*ystep;
		    ++i;
	    }
    }
    X_star.conservativeResize(i, 2);
    gp.predict_measurements(f_star, X_star, V_star);
	
	cout << "Done predicting gaussian process..." << endl;

	Eigen::MatrixXd predicted_points(X_star.rows(), 3);
	predicted_points.leftCols(3) = X_star;
	predicted_points.col(2) = f_star;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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

int main(int argc, char** argv)
{
	boost::filesystem::path folder(argv[1]);
	cout << "Folder : " << folder << endl;

    //SubmapsT submaps = read_submaps(folder);
	//visualize_submaps(submaps);
	
	Eigen::MatrixXd points = read_submap(folder / "patch_00_00.xyz");
	
	ProcessT gp(100, 100.);
	train_gp(points, gp);
	visualize_submap_and_gp(points, gp);

    return 0;
}
