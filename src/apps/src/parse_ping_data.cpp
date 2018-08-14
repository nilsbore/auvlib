#include <cereal/archives/json.hpp>

#include <cxxopts.hpp>
#include <data_tools/gsf_data.h>
#include <data_tools/csv_data.h>
#include <data_tools/transforms.h>
#include <data_tools/submaps.h>

#include <gpgs_slam/igl_visualizer.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

#include <chrono>

using namespace std;

pair<gsf_mbes_ping::PingsT, csv_nav_entry::EntriesT> load_or_parse_data(const boost::filesystem::path& swaths_folder,
                                                                        const boost::filesystem::path& poses_path)
{
    gsf_mbes_ping::PingsT pings;
    if (boost::filesystem::exists("ping_swaths.cereal")) {
        cout << "Reading saved pings..." << endl;
        std::ifstream is("ping_swaths.cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(pings);
        }
        is.close();
    }
    else {
        cout << "Parsing pings..." << endl;
        pings = parse_folder<gsf_mbes_ping>(swaths_folder);
        std::stable_sort(pings.begin(), pings.end(), [](const gsf_mbes_ping& ping1, const gsf_mbes_ping& ping2) {
            return ping1.time_stamp_ < ping2.time_stamp_;
        });
        std::ofstream os("ping_swaths.cereal", std::ofstream::binary);
        {
            cereal::BinaryOutputArchive archive(os);
			archive(pings);
        }
        os.close();
    }

    csv_nav_entry::EntriesT entries;
    if (boost::filesystem::exists("ping_poses.cereal")) {
        cout << "Reading saved poses..." << endl;
        std::ifstream is("ping_poses.cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(entries);
        }
        is.close();
    }
    else {
        cout << "Parsing poses..." << endl;
        entries = parse_file<csv_nav_entry>(poses_path);
        std::ofstream os("ping_poses.cereal", std::ofstream::binary);
        {
            cereal::BinaryOutputArchive archive(os);
			archive(entries);
        }
        os.close();
    }

    return make_pair(pings, entries);
}

int main(int argc, char** argv)
{
    string folder_str;
    string sounds_file_str;
    string poses_file_str;
    string file_str;
	double lsq = 7; //10.;
	double sigma = 1.; //5.;
	double s0 = 0.03; //.2;
    double pose_sigma = 0.2; //0.4;
    string dataset_name = "ping";

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("swaths", "Input gsf mb swaths folder", cxxopts::value(folder_str))
      ("poses", "Input nav file", cxxopts::value(poses_file_str))
      ("file", "Output file", cxxopts::value(file_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("pose_sigma", "The standard deviation pose update per meter", cxxopts::value(pose_sigma))
      ("name", "The name of the dataset, without spaces", cxxopts::value(dataset_name))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("swaths") == 0 || result.count("poses") == 0) {
		cout << "Please provide input swaths and poses args..." << endl;
		exit(0);
    }
    if (result.count("file") == 0) {
		cout << "Please provide output file arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
    boost::filesystem::path poses_path(poses_file_str);
    boost::filesystem::path path(file_str);

	cout << "Input folder : " << folder << endl;
	cout << "Output file : " << path << endl;

    gsf_mbes_ping::PingsT pings_unfiltered;
    csv_nav_entry::EntriesT entries;
    tie(pings_unfiltered, entries) = load_or_parse_data(folder, poses_path);

    mbes_ping::PingsT pings = convert_matched_entries(pings_unfiltered, entries);

    int counter = 0;
    for (gsf_mbes_ping ping : pings_unfiltered) {
        if (counter % 1000 == 0) {
            //cereal::JSONOutputArchive ar(std::cout);
            //ar(ping);
            cout << "Time string: " << ping.time_string_ << ", time stamp: " << ping.time_stamp_ << endl;
        }
        ++counter;
    }
    counter = 0;
    for (csv_nav_entry entry : entries) {
        if (counter % 100000 == 0) {
            //cereal::JSONOutputArchive ar(std::cout);
            //ar(entry);
            cout << "Time string: " << entry.time_string_ << ", time stamp: " << entry.time_stamp_ << endl;
        }
        ++counter;
    }
    //view_cloud(pings);

    track_error_benchmark benchmark(dataset_name);
    
    benchmark.add_ground_truth(pings);

    benchmark.add_benchmark(pings, "initial");
    benchmark.add_initial(pings);

    benchmark.submap_origin = Eigen::Vector3d::Zero(); // this should be a method

    boost::filesystem::path benchmark_path(dataset_name + "_benchmark.cereal");
    write_data(benchmark, benchmark_path);

    //write_data(entries, path);

    return 0;
}

