#include <Eigen/Dense>
#include <cxxopts.hpp>

#include <boost/filesystem.hpp>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <data_tools/data_structures.h>
#include <gpgs_slam/gp_submaps.h>
#include <gpgs_slam/igl_visualizer.h>

using namespace std;
using namespace data_structures;

int main(int argc, char** argv)
{
    string file_str;
    int subsample = 1;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("file", "The file with submaps and gps", cxxopts::value(file_str))
      ("subsample", "Subsampling rate", cxxopts::value(subsample));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("file") == 0) {
		cout << "Please provide file arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path file_path(file_str);
	cout << "File : " << file_path << endl;
    
    if (!boost::filesystem::exists(file_path)) {
        cout << "File: " << file_path << " does not exist..." << endl;
        exit(0);
    }

    gp_submaps ss = read_data<gp_submaps>(file_path);		
	
    IglVisCallback vis(ss.points, ss.gps, ss.trans, ss.angles, ss.bounds);
    vis.set_matches(ss.matches);
	vis.display();

    return 0;
}
