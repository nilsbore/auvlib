#include <Eigen/Dense>
#include <cxxopts.hpp>

// we should really do a types library, would avoid recompiling this all the time
#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/submaps.h>
#include <data_tools/data_structures.h>
#include <data_tools/benchmark.h>
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

mbes_ping::PingsT construct_gp_pings(gp_submaps::ProcessT& gp, Eigen::Matrix2d& bb,
                                     int nbr_pings, int swath_width)
{
	double maxx = bb(1, 0);
	double minx = bb(0, 0);
    double maxy = bb(1, 1);
	double miny = bb(0, 1);

	double xstep = (maxx - minx)/float(nbr_pings-1);
	double ystep = (maxy - miny)/float(swath_width-1);
    
	cout << "Predicting gaussian process..." << endl;

    cout << __FILE__ << ", " << __LINE__ << endl;

    Eigen::MatrixXd X_star(nbr_pings*swath_width, 2);
    Eigen::VectorXd f_star(nbr_pings*swath_width); // mean?
    f_star.setZero();
	Eigen::VectorXd V_star; // variance?
    for (int y = 0; y < swath_width; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < nbr_pings; ++x) {
		    X_star(y*nbr_pings+x, 0) = minx + x*xstep;
		    X_star(y*nbr_pings+x, 1) = miny + y*ystep;
	    }
    }
    cout << __FILE__ << ", " << __LINE__ << endl;

    gp.predict_measurements(f_star, X_star, V_star);
    cout << __FILE__ << ", " << __LINE__ << endl;

    mbes_ping::PingsT pings(nbr_pings);
	for (int x = 0; x < nbr_pings; ++x) {
        Eigen::Vector3d pos(0., 0., 0.);
        for (int y = 0; y < swath_width; ++y) { // ROOM FOR SPEEDUP
            Eigen::Vector3d point;
            point.head<2>() = X_star.row(y*nbr_pings+x).transpose();
            point(2) = f_star(y*nbr_pings+x);
            pings[x].beams.push_back(point);
            pos.array() += point.array();
        }
        pings[x].pos_ = 1./double(pings[x].beams.size())*pos;
        pings[x].first_in_file_ = false;
    }
    pings[0].first_in_file_ = true;
    cout << __FILE__ << ", " << __LINE__ << endl;

    return pings;
}

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

    boost::filesystem::path track_benchmark_path(ss.dataset_name + "_benchmark.cereal");
    benchmark::track_error_benchmark track_benchmark = read_data<benchmark::track_error_benchmark>(track_benchmark_path);

    benchmark::registration_summary_benchmark benchmark(ss.dataset_name);
    for (int i = 0; i < ss.points.size(); ++i) {
        Eigen::Matrix3d R = data_transforms::euler_to_matrix(ss.angles[i][0], ss.angles[i][1], ss.angles[i][2]);
        Eigen::Vector3d t = track_benchmark.submap_origin + ss.trans[i];

        mbes_ping::PingsT pings_points = benchmark::registration_summary_benchmark::get_submap_pings_index(track_benchmark.input_pings, i);
        mbes_ping::PingsT pings_gps = construct_gp_pings(ss.gps[i], ss.bounds[i], pings_points.size(), 400);
        for (mbes_ping& ping : pings_gps) {
            ping.pos_ = R*ping.pos_ + t;
            for (Eigen::Vector3d& beam : ping.beams) {
                beam = R*beam + t;
            }
        }
        mbes_ping::PingsT original_pings = pings_points;
        pings_points.insert(pings_points.end(), original_pings.begin(), original_pings.end());
        pings_gps.insert(pings_gps.end(), original_pings.begin(), original_pings.end());
        benchmark.add_registration_benchmark(pings_points, pings_gps, i, i);
    }
    benchmark.print_summary();
    //boost::filesystem::path benchmark_path(ss.dataset_name + "_registration_benchmark.cereal");
    //write_data(benchmark, benchmark_path);

    cout << "Points bytes: " << points_bytes << endl;
    cout << "GPS bytes: " << gps_bytes << endl;
    cout << "Compression ratio: " << double(gps_bytes)/double(points_bytes) << endl;

    return 0;
}
