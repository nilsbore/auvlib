#include <cxxopts.hpp>
#include <data_tools/navi_data.h>
#include <data_tools/data_structures.h>
#include <data_tools/transforms.h>
#include <data_tools/colormap.h>
#include <boost/filesystem.hpp>
#include <random>

#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <opencv2/highgui/highgui.hpp>

#include <gpgs_slam/igl_visualizer.h>

using namespace std;

void clip_submap(Eigen::MatrixXd& points, Eigen::Matrix2d& bounds, double minx, double maxx)
{
    bounds(0, 0) = max(minx, bounds(0, 0));
    bounds(1, 0) = min(maxx, bounds(1, 0));

    int counter = 0;
    for (int i = 0; i < points.rows(); ++i) {
        if (points(i, 0) > minx && points(i, 0) < maxx) {
            points.row(counter) = points.row(i);
            ++counter;
        }
    }
    points.conservativeResize(counter, 3);
}

// ok, let's have two things here, an angle bias,
// and a cumulative dvl error (probably with some bias).
void distort_tracks(mbes_ping::PingsT& pings)
{
    double dvl_std = 0.0; //01;
    double dvl_res_x = 0.0005;
    double dvl_res_y = 0.0002; //001;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0., 1.);
    Eigen::Vector3d cumulative_error; cumulative_error.setZero();
    int counter = 0;
    for (mbes_ping& ping : pings) {
        //if (ping.first_in_file_) {
        //    cumulative_error.setZero();
        //}
        if (counter % 2 == 0) {
            Eigen::Vector3d first_beam = ping.beams.front();
            Eigen::Vector3d last_beam = ping.beams.back();

            Eigen::Vector3d motion_dir(first_beam[1]-last_beam[1], -first_beam[0]+last_beam[0], 0);
            motion_dir.normalize();

            cumulative_error[0] += dvl_std*distribution(generator) + dvl_res_x*motion_dir[0];
            cumulative_error[1] += dvl_std*distribution(generator) + dvl_res_y*motion_dir[1];
        }

        ping.pos_ += cumulative_error;
        for (Eigen::Vector3d& beam : ping.beams) {
            beam += cumulative_error;
        }
    }
    
}

tuple<ObsT, TransT, AngsT, MatchesT, BBsT> load_or_create_submaps(const boost::filesystem::path& folder)
{
	// Parse ROV track files
	boost::filesystem::path nav_dir = folder / "NavUTM";

    // Parse MBES pings files
    boost::filesystem::path pings_dir = folder / "Pings";

    // Parse Intensity files
    boost::filesystem::path intensities_dir = folder / "Intensities";

    mbes_ping::PingsT pings;
    nav_entry::EntriesT entries;
    if (boost::filesystem::exists("navi_pings.cereal")) {
        std::ifstream is("navi_pings.cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(pings, entries);
        }
        is.close();
    }
    else {
        pings = parse_folder<mbes_ping>(pings_dir);
        entries = parse_folder<nav_entry>(nav_dir);
        std::ofstream os("navi_pings.cereal", std::ofstream::binary);
        {
            cereal::BinaryOutputArchive archive(os);
			archive(pings, entries);
        }
        os.close();
    }

    // distort submaps

    match_timestamps(pings, entries);

    int rows = 1000;
    int cols = 1000;
    track_error_benchmark benchmark;
    benchmark.track_img_params(pings, rows, cols);
    benchmark.draw_track_img(pings);

    for (mbes_ping& ping : pings) {
        benchmark.gt_track.push_back(ping.pos_);
    }

    //distort_tracks(pings);
    benchmark.draw_track_img(pings);
    
    cv::imshow("Track", benchmark.track_img);
    cv::waitKey();

    //divide_tracks(pings);
    //divide_tracks_equal(pings);
	//view_cloud(pings);

    for (mbes_ping& ping : pings) {
        if (ping.first_in_file_) {
            benchmark.submap_tracks.push_back(pt_submaps::TransT());
        }
        benchmark.submap_tracks.back().push_back(ping.pos_);
    }

    ObsT submaps;
    TransT trans;
    AngsT angs;
    MatchesT matches;
    BBsT bounds;
    tie(submaps, trans, angs, matches, bounds) = create_submaps(pings);
    
    Eigen::Vector3d origin = trans[0];
    for (int i = 0; i < trans.size(); ++i) {
        cout << "Trans " << i << ": " << trans[i].transpose() << endl;
        trans[i].array() -= origin.array();
    }
	
    benchmark.submap_origin = origin;
    write_data(benchmark, boost::filesystem::path("my_benchmark.cereal"));

    return make_tuple(submaps, trans, angs, matches, bounds);
}

int main(int argc, char** argv)
{
    string folder_str;
    string file_str;
	double lsq = 2.;
	double sigma = 1.;
	double s0 = .5;
    double minx = -50.;
    double maxx = 0.;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	//options.positional_help("[optional args]").show_positional_help();
	options.add_options()
      ("help", "Print help")
      ("folder", "Input folder", cxxopts::value(folder_str))
      ("file", "Output file", cxxopts::value(file_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("minx", "X clip min", cxxopts::value(minx))
      ("maxx", "X clip max", cxxopts::value(maxx))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide input folder arg..." << endl;
		exit(0);
    }
    if (result.count("file") == 0) {
		cout << "Please provide output file arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
    boost::filesystem::path path(file_str);

	cout << "Input folder : " << folder << endl;
	cout << "Output file : " << path << endl;
    
    gp_submaps ss;
    tie(ss.points, ss.trans, ss.angles, ss.matches, ss.bounds) = load_or_create_submaps(folder);
    
    for (int i = 0; i < ss.points.size(); ++i) {

        clip_submap(ss.points[i], ss.bounds[i], minx, maxx);

        gp_submaps::ProcessT gp(100, s0);
        gp.kernel.sigmaf_sq = sigma;
        gp.kernel.l_sq = lsq*lsq;
        gp.kernel.p(0) = gp.kernel.sigmaf_sq;
        gp.kernel.p(1) = gp.kernel.l_sq;
        // this will also centralize the points
        Eigen::MatrixXd X = ss.points[i].leftCols(2);
        Eigen::VectorXd y = ss.points[i].col(2);
        //gp.train_parameters(X, y);
        gp.add_measurements(X, y);

        std::cout << "Done training gaussian process..." << std::endl;
        ss.gps.push_back(gp);
        cout << "Pushed back..." << endl;

        ss.rots.push_back(euler_to_matrix(ss.angles[i](0), ss.angles[i](1), ss.angles[i](2)));
    }
	
    IglVisCallback vis(ss.points, ss.gps, ss.trans, ss.angles, ss.bounds);
    vis.display();

    write_data(ss, path);

    return 0;
}