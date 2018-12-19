#include <Eigen/Dense>
#include <cxxopts.hpp>

// we should really do a types library, would avoid recompiling this all the time
#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/submaps.h>
#include <data_tools/data_structures.h>
#include <data_tools/transforms.h>
#include <gpgs_slam/gp_submaps.h>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <random>
#include <thread>
#include <future>

using namespace std;
using namespace data_structures;

void subsample_cloud(Eigen::MatrixXd& points, int subsample)
{
    int counter = 0;
    for (int i = 0; i < points.rows(); ++i) {
        if (i % subsample == 0) {
            points.row(counter) = points.row(i);
            ++counter;
        }
    }
    points.conservativeResize(counter, 3);
}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string input_file_str;
    string result_file_str;
    int subsample = 1;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("file", "Input file", cxxopts::value(input_file_str))
      ("result", "Result file", cxxopts::value(result_file_str))
      ("subsample", "Subsampling rate", cxxopts::value(subsample));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("file") == 0) {
		cout << "Please provide input arg..." << endl;
		exit(0);
    }
	
    boost::filesystem::path input_path(input_file_str);
    boost::filesystem::path result_path(result_file_str);
	cout << "Input file : " << input_path << endl;
	cout << "Result file : " << result_path << endl;

    gp_submaps ss = read_data<gp_submaps>(input_path);
    subsample_cloud(ss.points[0], subsample);
    ss.gps[0].reset(300);
    ss.gps[0].train_log_parameters(ss.points[0].leftCols<2>(), ss.points[0].col(2));

    return 0;
}
