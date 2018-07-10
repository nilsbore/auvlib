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

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <random>
#include <thread>
#include <future>

using namespace std;

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string input_file_str;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("file", "Input file", cxxopts::value(input_file_str));

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
	cout << "Input file : " << input_path << endl;

    gp_submaps ss = read_data<gp_submaps>(input_path);

    boost::filesystem::path points_path("uncompressed_points.cereal"); 
    boost::filesystem::path gps_path("sparse_gps.cereal"); 
    write_data(ss.points, points_path);
    write_data(ss.gps, gps_path);

    uintmax_t points_bytes = boost::filesystem::file_size(points_path);
    uintmax_t gps_bytes = boost::filesystem::file_size(gps_path);

    cout << "Points bytes: " << points_bytes << endl;
    cout << "GPS bytes: " << gps_bytes << endl;
    cout << "Compression ratio: " << double(gps_bytes)/double(points_bytes) << endl;

    return 0;
}
