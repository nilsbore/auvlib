#include <cxxopts.hpp>
#include <data_tools/navi_data.h>
#include <data_tools/data_structures.h>
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
#include <gpgs_slam/transforms.h>

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

tuple<ObsT, TransT, AngsT, MatchesT, BBsT> load_or_create_submaps(const boost::filesystem::path& folder)
{
    if (boost::filesystem::exists("navi_submaps.cereal")) {
        ObsT submaps;
        TransT trans;
        AngsT angs;
        MatchesT matches;
        BBsT bounds;
        std::ifstream is("navi_submaps.cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(submaps, trans, angs, matches, bounds);
        }
        is.close();
		return make_tuple(submaps, trans, angs, matches, bounds);
    }
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
    //divide_tracks_equal(pings);
	//view_cloud(pings);
    ObsT submaps;
    TransT trans;
    AngsT angs;
    MatchesT matches;
    BBsT bounds;
    tie(submaps, trans, angs, matches, bounds) = create_submaps(pings);

    std::ofstream os("navi_submaps.cereal", std::ofstream::binary);
    {
        cereal::BinaryOutputArchive archive(os);
        archive(submaps, trans, angs, matches, bounds);
    }
    os.close();
	
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
    
    Eigen::Vector3d origin = ss.trans[0];
    for (int i = 0; i < ss.trans.size(); ++i) {
        cout << "Trans " << i << ": " << ss.trans[i].transpose() << endl;
        ss.trans[i].array() -= origin.array();
    }
    
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
