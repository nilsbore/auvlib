#include <Eigen/Dense>
#include <cxxopts.hpp>

// we should really do a types library, would avoid recompiling this all the time
#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/submaps.h>
#include <gpgs_slam/gp_submaps.h>
#include <data_tools/data_structures.h>
#include <data_tools/benchmark.h>
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
using namespace data_structures;

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string input_file_str;
    string result_file_str;
    double thresh = -1.2;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("input", "Input file", cxxopts::value(input_file_str))
      ("result", "Result file", cxxopts::value(result_file_str))
      ("thresh", "Likelihood threshold", cxxopts::value(thresh));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("input") == 0 || result.count("result") == 0) {
		cout << "Please provide input and result args..." << endl;
		exit(0);
    }
	
    boost::filesystem::path input_path(input_file_str);
    boost::filesystem::path result_path(result_file_str);
	cout << "Input file : " << input_path << endl;
	cout << "Result file : " << result_path << endl;

    gp_submaps input_ss = read_data<gp_submaps>(input_path);
    gp_submaps result_ss = read_data<gp_submaps>(result_path);
	
    pt_submaps::TransT trans_0 = input_ss.trans;
    pt_submaps::RotsT rots_0 = input_ss.rots;

    boost::filesystem::path benchmark_path(input_ss.dataset_name + "_benchmark.cereal");
    benchmark::track_error_benchmark benchmark = read_data<benchmark::track_error_benchmark>(benchmark_path);
    
    pt_submaps::TransT trans_corr;
    pt_submaps::RotsT rots_corr;
    for (int i = 0; i < result_ss.points.size(); ++i) {
        Eigen::Matrix3d R = data_transforms::euler_to_matrix(result_ss.angles[i][0], result_ss.angles[i][1], result_ss.angles[i][2]);
        Eigen::Matrix3d Rc = R*rots_0[i].transpose();
        Eigen::Vector3d tc = benchmark.submap_origin + result_ss.trans[i] - Rc*(benchmark.submap_origin + trans_0[i]);
        trans_corr.push_back(tc);
        rots_corr.push_back(Rc);
    }

    mbes_ping::PingsT pings = benchmark.input_pings;
    Eigen::VectorXd ll;
    Eigen::MatrixXd dX;
    int i = -1;
    int counter = 0;
    for (mbes_ping& ping : pings) {
        if (ping.first_in_file_) {
            ++i;
            counter = 0;
            input_ss.gps[i].compute_neg_log_derivatives_fast(ll, dX, input_ss.points[i].leftCols<2>(), input_ss.points[i].col(2), false);
            cout << "min likelihood: " << ll.minCoeff() << endl;
            cout << "max likelihood: " << ll.maxCoeff() << endl;
        }
        
        Eigen::Matrix3d R = data_transforms::euler_to_matrix(result_ss.angles[i][0], result_ss.angles[i][1], result_ss.angles[i][2]);
        ping.pos_ = rots_corr[i]*ping.pos_ + trans_corr[i];

        pt_submaps::TransT new_beams;
        new_beams.reserve(ping.beams.size());
        for (int j = 0; j < ping.beams.size(); ++j) {
            if (ll(counter) > thresh) {
                new_beams.push_back(rots_corr[i]*ping.beams[j] + trans_corr[i]);
            }
            ++counter;
        }
        ping.beams = new_beams;

        // NOTE: this is incredibly slow
        /*
        for (Eigen::Vector3d b : ping.beams) {
            Eigen::Vector3d gpb = rots_0[i].transpose()*(b - benchmark.submap_origin - trans_0[i]);
            double ll = input_ss.gps[i].neg_log_likelihood(gpb.head<2>(), gpb[2]);
            if (ll > thresh) {
                new_beams.push_back(rots_corr[i]*b + trans_corr[i]);
            }
        }*/

    }

    benchmark.add_benchmark(pings, "slam_filtered");
    benchmark.print_summary();
    write_data(benchmark, benchmark_path);

    return 0;
}
