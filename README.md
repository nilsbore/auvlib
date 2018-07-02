# gpgs_slam
Gaussian Process Graph Slam Multibeam SLAM System

## Dependencies

On Ubuntu 16.04, just use the following command to install all dependencies:
```
sudo apt-get install libcereal-dev libglfw3-dev libceres-dev
```

## Building

Once cloned, get the libigl submodule via `git submodule init`, `git submodule update`.
Then, create a build folder, and run `cmake ../src`, followed by `make` within that folder.

## Running

You can run a toy example with data provided in this repo.
In the `scripts` folder, execute `./generate_submaps.py`. This creates
a bunch of data files within the `scripts` folder. Next, go to your
build folder and run
```
./parse_sim_data --folder ../scripts --file example_problem.cereal
```
This creates the optimization problem `example_problem.cereal`. You can view it with
```
./visualize_slam_process --file example_problem.cereal
```
Now, we can optimize it by running
```
./slam_process_ceres --file example_problem.cereal --output example_results.cereal
```
Again, the results `example_results.cereal` can be viewed using the visualizer.

## data_tools

### navi_data

The `navi_data` library contains tools for parsing files exported from NaviEdit.
See this example program, also available in [the test folder](https://github.com/nilsbore/gpgs_slam/blob/master/src/apps/test/test_parse_navi_data.cpp).

```cpp
#include <boost/filesystem.hpp>
#include <cxxopts.hpp>
#include <data_tools/data_structures.h>
#include <data_tools/navi_data.h>
#include <gpgs_slam/transforms.h>

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

    boost::filesystem::path folder(folder_str);
    boost::filesystem::path pings_path = folder / "mbes_pings.cereal";
    boost::filesystem::path entries_path = folder / "nav_entries.cereal";
    boost::filesystem::path submaps_path = folder / "submaps.cereal";

    // parse the files from the folder
    mbes_ping::PingsT pings = parse_folder<mbes_ping>(folder / "Pings");
    nav_entry::EntriesT entries = parse_folder<nav_entry>(folder / "NavUTM");

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
    tie(ss.points, ss.trans, ss.angles, ss.matches, ss.bounds) = create_submaps(pings);
    for (const Eigen::Vector3d& ang : ss.angles) {
        ss.rots.push_back(euler_to_matrix(ang(0), ang(1), ang(2)));
    }

    // write to disk
    write_data(ss, submaps_path);
    // read from disk
    ss = read_data<pt_submaps>(submaps_path);

    return 0;
}
```

### gsf_data

You read the gsf-based data in much the same way, using the `data_tools` library.
The main differences are highlighted in the example below, with a full example in
the [apps directory](https://github.com/nilsbore/gpgs_slam/blob/master/src/apps/src/parse_gsf_data.cpp):

```cpp
#include <cxxopts.hpp>
#include <data_tools/gsf_data.h>
#include <data_tools/transforms.h>

using namespace std;

// we divide the map by hand into submaps right now
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

int main(int argc, char** argv)
{
    string folder_str;
    string sounds_file_str;
    string poses_file_str;

    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
      ("help", "Print help")
      ("swaths", "Input gsf mb swaths folder", cxxopts::value(folder_str))
      ("sounds", "Input sound speed file", cxxopts::value(sounds_file_str))
      ("poses", "Input nav file", cxxopts::value(poses_file_str));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
    }
    if (result.count("swaths") == 0 || result.count("sounds") == 0 || result.count("poses") == 0) {
        cout << "Please provide input swaths, sounds and poses args..." << endl;
        exit(0);
    }
	
    boost::filesystem::path folder(folder_str);
    boost::filesystem::path sounds_path(sounds_file_str);
    boost::filesystem::path poses_path(poses_file_str);
    
    // this should read from the folder containing the .gsf swath files
    gsf_mbes_ping::PingsT pings_unfiltered = parse_folder<gsf_mbes_ping>(folder);
    
    // the first and last entries are not valid, discard them
    std::stable_sort(pings_unfiltered.begin(), pings_unfiltered.end(), [](const gsf_mbes_ping& ping1, const gsf_mbes_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });
    gsf_mbes_ping::PingsT pings(pings_unfiltered.begin() + 2300, pings_unfiltered.begin() + 52600);
    pings[0].first_in_file_ = true;

    // read the dr poses, called something like dr_pose_est.data
    gsf_nav_entry::EntriesT entries = parse_file<gsf_nav_entry>(poses_path);
    // read the sounds speed data, called something like sound_speed.data
    gsf_sound_speed::SpeedsT speeds = parse_file<gsf_sound_speed>(sounds_path);
    // match ping times with sound speeds
    match_sound_speeds(pings, speeds);
    // convert to mbes_ping format
    mbes_ping::PingsT new_pings = convert_matched_entries(pings, entries);
    
    // divide the map manually, per the function above
    divide_gsf_map(new_pings);
    
    return 0;
}
```
To know which files to look at, it may also help to look at the [tests in `data_tools`](https://github.com/nilsbore/gpgs_slam/blob/master/src/data_tools/src/test_gsf.cpp).

### Benchmarking

The tests also [contains an example](https://github.com/nilsbore/gpgs_slam/blob/master/src/apps/test/test_parse_navi_data.cpp)
of how to use the benchmark system. After having read your files like above, simply do something like the following:
```cpp
// create a new becnhmark object
track_error_benchmark benchmark;

// add ground truth pings
benchmark.add_ground_truth(pings);

// may be something more interesting but, add gt as initial value of optimization
benchmark.add_benchmark(pings, "initial");
benchmark.add_initial(pings);

// save the benchmark to disk, for use later
write_data(benchmark, boost::filesystem::path("my_benchmark.cereal"));
```
Now, in the file where we do our optimization, we can load our benchmark
again, add our optimized pings to the benchmark again, and save the results:
```cpp
// load the benchmark saved above
track_error_benchmark benchmark = load_data<track_error_benchmark>(boost::filesystem::path("my_benchmark.cereal"));

mbes_ping::PingsT optimized_pings;
// fill in the pings using some slam algorithm ...
// add the benchmark with a random name e.g. "slam"
benchmark.add_benchmark(optimized_pings, "slam");

// print the results of the benchmark
benchmark.print_summary();

// save the benchmark to disk, for use later
write_data(benchmark, boost::filesystem::path("my_benchmark.cereal"));
```
