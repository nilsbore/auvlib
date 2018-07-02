#include <boost/filesystem.hpp>
#include <cxxopts.hpp>
#include <data_tools/data_structures.h>
#include <data_tools/navi_data.h>
#include <data_tools/transforms.h>

using namespace std;

int main(int argc, char** argv)
{
    string folder_str;

    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()("help", "Print help")("folder", "Input folder", cxxopts::value(folder_str));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }
    if (result.count("folder") == 0) {
        cout << "Please provide input folder arg..." << endl;
        exit(0);
    }

    boost::filesystem::path folder(folder_str);
    boost::filesystem::path pings_path = folder / "mbes_pings.cereal";
    boost::filesystem::path entries_path = folder / "nav_entries.cereal";
    boost::filesystem::path submaps_path = folder / "submaps.cereal";

    cout << "Input folder : " << folder << endl;

    // Parse ROV track files
    boost::filesystem::path nav_dir = folder / "NavUTM";

    // Parse MBES pings files
    boost::filesystem::path pings_dir = folder / "Pings";

    // Parse Intensity files
    boost::filesystem::path intensities_dir = folder / "Intensities";

    // parse the files from the folder
    mbes_ping::PingsT pings = parse_folder<mbes_ping>(pings_dir);
    nav_entry::EntriesT entries = parse_folder<nav_entry>(nav_dir);

    // match timestamps of pings and nav entries
    match_timestamps(pings, entries);

    // Divides the tracks into roughly square pieces
    divide_tracks_equal(pings);
    //divide_tracks(pings);

    // write to disk
    write_data(pings, pings_path);
    write_data(entries, entries_path);

    // read from disk
    pings = read_data<mbes_ping::PingsT>(pings_path);
    entries = read_data<nav_entry::EntriesT>(entries_path);

    // convert to submaps
    pt_submaps ss;
    tie(ss.points, ss.trans, ss.angles, ss.matches, ss.bounds, ss.tracks) = create_submaps(pings);
    for (const Eigen::Vector3d& ang : ss.angles) {
        ss.rots.push_back(euler_to_matrix(ang(0), ang(1), ang(2)));
    }

    // write to disk
    write_data(ss, submaps_path);
    // read from disk
    ss = read_data<pt_submaps>(submaps_path);

    // create a new becnhmark object
    track_error_benchmark benchmark;
    
    // add ground truth pings
    benchmark.add_ground_truth(pings);

    // may be something more interesting but, add gt as initial value of optimization
    benchmark.add_benchmark(pings, "initial");
    benchmark.add_initial(pings);

    // save the benchmark to disk, for use later
    write_data(benchmark, boost::filesystem::path("my_benchmark.cereal"));

    // e.g., if we have optimized the position of the pings:
    mbes_ping::PingsT optimized_pings = pings; // hopefully, we have a better slam algorithm than this
    benchmark.add_benchmark(optimized_pings, "slam");

    // save the benchmark to disk, for use later
    write_data(benchmark, boost::filesystem::path("my_benchmark.cereal"));

    return 0;
}
