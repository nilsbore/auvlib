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

gsf_mbes_ping::PingsT load_or_parse_pings(const boost::filesystem::path& swaths_folder, const string& dataset_name)
{
    gsf_mbes_ping::PingsT pings;
    if (boost::filesystem::exists("ping_swaths_" + dataset_name + ".cereal")) {
        cout << "Reading saved pings..." << endl;
        std::ifstream is("ping_swaths_" + dataset_name + ".cereal", std::ifstream::binary);
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
        std::ofstream os("ping_swaths_" + dataset_name + ".cereal", std::ofstream::binary);
        {
            cereal::BinaryOutputArchive archive(os);
			archive(pings);
        }
        os.close();
    }
    return pings;
}

csv_nav_entry::EntriesT load_or_parse_entries(const boost::filesystem::path& poses_path)
{
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
    return entries;
}

int main(int argc, char** argv)
{
    string pre_folder_str;
    string post_folder_str;
    //string poses_file_str;
    string file_str;
	double lsq = 7; //10.;
	double sigma = 1.; //5.;
	double s0 = 0.03; //.2;
    double pose_sigma = 0.2; //0.4;
    string dataset_name = "ping_compare";

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("pre_swaths", "Input gsf mb swaths folder pre deployment", cxxopts::value(pre_folder_str))
      ("post_swaths", "Input gsf mb swaths folder post deployment", cxxopts::value(post_folder_str))
      //("poses", "Input nav file", cxxopts::value(poses_file_str))
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
    if (result.count("pre_swaths") == 0 || result.count("post_swaths") == 0) {
		cout << "Please provide input swaths and poses args..." << endl;
		exit(0);
    }
    if (result.count("file") == 0) {
		cout << "Please provide output file arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path pre_folder(pre_folder_str);
	boost::filesystem::path post_folder(post_folder_str);
    //boost::filesystem::path poses_path(poses_file_str);
    boost::filesystem::path path(file_str);

	cout << "Input pre folder : " << pre_folder << endl;
	cout << "Input post folder : " << post_folder << endl;
	cout << "Output file : " << path << endl;

    gsf_mbes_ping::PingsT pings_unfiltered_pre = load_or_parse_pings(pre_folder, "pre");
    gsf_mbes_ping::PingsT pings_unfiltered_post = load_or_parse_pings(post_folder, "post");
    //csv_nav_entry::EntriesT entries = load_or_parse_entries(poses_path);

    for (gsf_mbes_ping& ping : pings_unfiltered_pre) {
        ping.first_in_file_ = false;
    }
    for (gsf_mbes_ping& ping : pings_unfiltered_post) {
        ping.first_in_file_ = false;
    }
    pings_unfiltered_pre[0].first_in_file_ = true;
    pings_unfiltered_post[0].first_in_file_ = true;
    pings_unfiltered_pre.insert(pings_unfiltered_pre.end(), pings_unfiltered_post.begin(), pings_unfiltered_post.end());

    //mbes_ping::PingsT pings = convert_matched_entries(pings_unfiltered, entries);
    mbes_ping::PingsT pings = convert_pings(pings_unfiltered_pre);

    track_error_benchmark benchmark(dataset_name);
    
    benchmark.add_ground_truth(pings);

    //benchmark.add_benchmark(pings, "initial");
    //benchmark.add_initial(pings);

    benchmark.submap_origin = Eigen::Vector3d::Zero(); // this should be a method

    boost::filesystem::path benchmark_path(dataset_name + "_benchmark.cereal");
    write_data(benchmark, benchmark_path);

    //write_data(entries, path);

    return 0;
}

