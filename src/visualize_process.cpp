#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <Eigen/Dense>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>

using namespace std;
using PointT = pcl::PointXYZRGB;
using submap_matrix = vector<vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >;

const int colormap[][3] = {
    { 166, 206, 227 },
    { 31, 120, 180 },
    { 178, 223, 138 },
    { 51, 160, 44 },
    { 251, 154, 153 },
    { 227, 26, 28 },
    { 253, 191, 111 },
    { 255, 127, 0 },
    { 202, 178, 214 },
    { 106, 61, 154 },
    { 255, 255, 153 },
    { 177, 89, 40 },
    { 141, 211, 199 },
    { 255, 255, 179 },
    { 190, 186, 218 },
    { 251, 128, 114 },
    { 128, 177, 211 },
    { 253, 180, 98 },
    { 179, 222, 105 },
    { 252, 205, 229 },
    { 217, 217, 217 },
    { 188, 128, 189 },
    { 204, 235, 197 },
    { 255, 237, 111 },
    { 255, 179, 0 },
    { 128, 62, 117 },
    { 255, 104, 0 },
    { 166, 189, 215 },
    { 193, 0, 32 },
    { 206, 162, 98 },
    { 0, 125, 52 },
    { 246, 118, 142 },
    { 0, 83, 138 },
    { 255, 122, 92 },
    { 83, 55, 122 },
    { 255, 142, 0 },
    { 179, 40, 81 },
    { 244, 200, 0 },
    { 127, 24, 13 },
    { 147, 170, 0 },
    { 89, 51, 21 },
    { 241, 58, 19 },
    { 35, 44, 22 }
};

submap_matrix read_submaps(const boost::filesystem::path& folder)
{
    submap_matrix submaps;

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
			if (jj == 0) {
                submaps.push_back(vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >);
            }
			submaps.back().push_back(points);
		}
	}

	return submaps;
}

void visualize_submaps(submap_matrix& submaps)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	int i = 0;
    for (int ii = 0; ii < submaps.size(); ++ii) {
        for (int jj = 0; jj < submaps[ii].size(); ++jj) {
		    PointT p;
			p.getVector3fMap() = submaps[ii][jj].row(i).cast<float>();
		    p.r = colormap[counter % 44][0];
		    p.g = colormap[counter % 44][1];
		    p.b = colormap[counter % 44][2];
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

void train_gp(Eigen::MatrixXd& points)
{
    using gp = sparse_gp<rbf_kernel, probit_noise>;
	gp mygp(100, 0.1);
    Eigen::MatrixXd X = points.leftCols(2);
	Eigen::VectorXd y = points.col(2);
	gp.add_measurements(X, y);

	MatrixXd X_star;
    VectorXd f_star;
	int sz = 50;
	X_star.resize(sz*sz, 2);
    int points = 0;
    for (int y = 0; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < sz; ++x) {
		    ind = x*sz + y;
		    /*if (!W(ind, i)) {
			    continue;
		    }*/
		    X_star(points, 0) = res*((double(x) + 0.5f)/double(sz) - 0.5f);
		    X_star(points, 1) = res*((double(y) + 0.5f)/double(sz) - 0.5f);
		    ++points;
	    }
    }
    X_star.conservativeResize(points, 2);
    mygp.predict_measurements(f_star, X_star, V_star);
}

int main(int argc, char** argv)
{
	boost::filesystem::path folder(argv[1]);
	cout << "Folder : " << folder << endl;

	submap_matrix submaps = read_submaps(folder);
	visualize_submaps(submaps);

    return 0;
}
