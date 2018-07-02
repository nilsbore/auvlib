#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <sstream>

using namespace std;
using PointT = pcl::PointXYZRGB;

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

int main(int argc, char** argv)
{
	boost::filesystem::path folder(argv[1]);
	cout << "Folder : " << folder << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	int counter = 0;
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
			std::ifstream infile(filename.string());
			std::string line;
			while (std::getline(infile, line)) {
				std::istringstream iss(line);
				PointT p;
				if (!(iss >> p.x >> p.y >> p.z)) {
					break;
				} // error
				p.r = colormap[counter % 44][0];
				p.g = colormap[counter % 44][1];
				p.b = colormap[counter % 44][2];
				cloud->push_back(p);
			}

			++counter;
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
