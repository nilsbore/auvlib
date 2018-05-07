#include <data_tools/submaps.h>
#include <data_tools/colormap.h>

#include <pcl/visualization/cloud_viewer.h>
#include <sstream>

using namespace std;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

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
		if (counter % 37 != 0) {
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
