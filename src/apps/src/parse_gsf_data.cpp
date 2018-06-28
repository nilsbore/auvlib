#include <cereal/archives/json.hpp>

#include <cxxopts.hpp>
#include <data_tools/gsf_data.h>
#include <data_tools/transforms.h>

#include <gpgs_slam/igl_visualizer.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

using namespace std;

void divide_gsf_map(mbes_ping::PingsT& pings)
{
    pings[3926].first_in_file_ = false;
    pings[5200].first_in_file_ = true;
    pings[10151].first_in_file_ = false;
    pings[10400].first_in_file_ = true;
    pings[15500].first_in_file_ = true;
    pings[16376].first_in_file_ = false;
    pings[20700].first_in_file_ = true;
    pings[22601].first_in_file_ = false;
    pings[25800].first_in_file_ = true;
    pings[28827].first_in_file_ = false;
    pings[30750].first_in_file_ = true;
    pings[33300].first_in_file_ = true;
    pings[34500].first_in_file_ = true;
    pings[35052].first_in_file_ = false;
    pings[36800].first_in_file_ = true;
    pings[37800].first_in_file_ = true;
    pings[40300].first_in_file_ = true;
    pings[43700].first_in_file_ = true;
    pings[44600].first_in_file_ = true;
    pings[47000].first_in_file_ = true;
    pings[47502].first_in_file_ = false;
    pings[48000].first_in_file_ = true;
}

pt_submaps::MatchesT get_gsf_matches()
{
    pt_submaps::MatchesT matches;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            matches.push_back(make_pair(i, 6+2*j));
        }
    }

    return matches;
}

int main(int argc, char** argv)
{
    string folder_str;
    string sounds_file_str;
    string poses_file_str;
    string file_str;
	double lsq = 2.;
	double sigma = 1.;
	double s0 = .5;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("swaths", "Input gsf mb swaths folder", cxxopts::value(folder_str))
      ("sounds", "Input sound speed file", cxxopts::value(sounds_file_str))
      ("poses", "Input nav file", cxxopts::value(poses_file_str))
      ("file", "Output file", cxxopts::value(file_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("swaths") == 0 || result.count("sounds") == 0 || result.count("poses") == 0) {
		cout << "Please provide input swaths, sounds and poses args..." << endl;
		exit(0);
    }
    if (result.count("file") == 0) {
		cout << "Please provide output file arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
    boost::filesystem::path sounds_path(sounds_file_str);
    boost::filesystem::path poses_path(poses_file_str);
    boost::filesystem::path path(file_str);

	cout << "Input folder : " << folder << endl;
	cout << "Output file : " << path << endl;
    gsf_mbes_ping::PingsT pings_unfiltered = parse_folder<gsf_mbes_ping>(folder);
    std::stable_sort(pings_unfiltered.begin(), pings_unfiltered.end(), [](const gsf_mbes_ping& ping1, const gsf_mbes_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });
    gsf_mbes_ping::PingsT pings(pings_unfiltered.begin() + 2300, pings_unfiltered.begin() + 52600);
    pings[0].first_in_file_ = true;

    gsf_nav_entry::EntriesT entries = parse_file<gsf_nav_entry>(poses_path);
    
    gsf_sound_speed::SpeedsT speeds = parse_file<gsf_sound_speed>(sounds_path);

    match_sound_speeds(pings, speeds);
    mbes_ping::PingsT new_pings = convert_matched_entries(pings, entries);
    divide_gsf_map(new_pings);

    for (mbes_ping& ping : new_pings) {
        ping.beams.erase(std::remove_if(ping.beams.begin(), ping.beams.end(), [](const Eigen::Vector3d& p) {
            return p[2] > -28. || p[2] < -34.;
            //return false; //p[2] > -16. || p[2] < -13.;
        }), ping.beams.end());
    }

    /*
    gp_submaps ss;
    tie(ss.points, ss.trans, ss.angles, ss.matches, ss.bounds) = create_submaps(new_pings);
    ss.matches = get_gsf_matches();

    for (int i = 0; i < ss.points.size(); ++i) {

        //clip_submap(ss.points[i], ss.bounds[i], minx, maxx);

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
    */

    cout << "Wrote output submaps file " << path << endl;
    
    track_error_benchmark benchmark;
    
    benchmark.add_ground_truth(new_pings);

    benchmark.add_benchmark(new_pings, "initial");
    benchmark.add_initial(new_pings);

    benchmark.submap_origin = Eigen::Vector3d::Zero(); // this should be a method
    write_data(benchmark, boost::filesystem::path("gsf_benchmark.cereal"));

    return 0;
}

