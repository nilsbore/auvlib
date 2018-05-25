#include <cxxopts.hpp>
#include <data_tools/navi_data.h>
#include <boost/filesystem.hpp>

#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <gpgs_slam/igl_visualizer.h>

using namespace std;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using SubmapsGPT = vector<ProcessT>; // Process does not require aligned allocation as all matrices are dynamic

int main(int argc, char** argv)
{

    string folder_str;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	//options.positional_help("[optional args]").show_positional_help();
	options.add_options()
      ("help", "Print help")
      ("folder", "Folder", cxxopts::value(folder_str));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide folder arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
	cout << "Folder : " << folder << endl;
    
	// Parse ROV track files
	boost::filesystem::path nav_dir = folder / "NavUTM";

    // Parse MBES pings files
    boost::filesystem::path pings_dir = folder / "Pings";

    // Parse Intensity files
    boost::filesystem::path intensities_dir = folder / "Intensities";

	vector<mbes_ping> pings = read_folder<mbes_ping>(pings_dir);
	vector<nav_entry> entries = read_folder<nav_entry>(nav_dir);

    match_timestamps(pings, entries);
    //divide_tracks(pings);
    divide_tracks_equal(pings);
	//view_cloud(pings);
    ObsT submaps;
    TransT trans;
    AngsT angs;
    MatchesT matches;
    BBsT bounds;
    tie(submaps, trans, angs, matches, bounds) = create_submaps(pings);
    RotsT rots;
    SubmapsGPT gps;
    //visualize_submaps(submaps, trans, angs);
	
    IglVisCallback* vis = new IglVisCallback(submaps, gps, trans, angs, bounds);
    vis->display();
		
    std::ifstream is("gp_submaps.cereal", std::ifstream::binary);
    {
        cereal::BinaryInputArchive archive(is);
        archive(submaps, gps, trans, rots, matches, bounds);
    }
    is.close();

    return 0;
}
