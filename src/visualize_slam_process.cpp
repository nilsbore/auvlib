#include <Eigen/Dense>
#include <cxxopts.hpp>

#include <boost/filesystem.hpp>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <gpgs_slam/igl_visualizer.h>

using namespace std;
using TransT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using SubmapsGPT = vector<ProcessT>;
using ObsT = vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;

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
		
    TransT trans;
	RotsT rots;
    AngsT angles;
    SubmapsGPT gps;
    ObsT obs;
    MatchesT matches;
    BBsT bounds;
    std::ifstream is(file_path.string(), std::ifstream::binary);
    {
        cereal::BinaryInputArchive archive(is);
        archive(obs, gps, trans, rots, angles, matches, bounds);
    }
    is.close();
	
    IglVisCallback* vis = new IglVisCallback(obs, gps, trans, angles, bounds);
    vis->set_matches(matches);
	vis->display();

    return 0;
}
