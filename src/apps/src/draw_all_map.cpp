#include <cereal/archives/json.hpp>

#include <cxxopts.hpp>
#include <data_tools/all_data.h>
#include <data_tools/xtf_data.h>

#include <bathy_maps/draw_map.h>
#include <bathy_maps/mesh_map.h>

#include <chrono>

using namespace std;
using namespace std_data;
using namespace all_data;

int main(int argc, char** argv)
{
    string folder_str;
    string file_str;
	double lsq = 7; //10.;
	double sigma = 1.; //5.;
	double s0 = 0.03; //.2;
    double pose_sigma = 0.2; //0.4;
    string dataset_name = "borno_group1";

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("folder", "Input all mb swaths folder pre deployment", cxxopts::value(folder_str))
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

	cout << "Input mbes folder : " << folder << endl;
	cout << "Output file : " << path << endl;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    mesh_map::BoundsT bounds;

    // we need to separate the reading of mbes and side scan pings since they consume a lot of memory
    {
	    all_nav_entry::EntriesT entries = parse_folder<all_nav_entry>(folder);
	    all_mbes_ping::PingsT pings = parse_folder<all_mbes_ping>(folder);
	    //all_nav_depth::EntriesT depths = parse_folder<all_nav_depth>(folder);
        mbes_ping::PingsT new_pings = convert_matched_entries(pings, entries);
        tie(V, F, bounds) = mesh_map::mesh_from_pings(new_pings, 2.0);
        mesh_map::show_mesh(V, F);
    }
    //mesh.show_mesh(V, F);

    /*
    {
        xtf_sss_ping::PingsT pings_sss = load_or_parse_pings<xtf_sss_ping>(sss_folder, dataset_name + "_sss");
        mesh.overlay_sss(V, F, bounds, pings_sss);
    }
    */

    return 0;
}

