# CPP Usage

This document is mainly concerned with how to parse data with the C++ interface, and how to
benchmark SLAM and registration algorithms.

## Library data interfaces

The main interfaces are defined in the [std_data](https://github.com/nilsbore/auvlib/blob/master/src/data_tools/include/data_tools/std_data.h) header.

### mbes_ping

For example, it contains the data structure `mbes_ping` that is the common
data structure for representing multibeam swath data. Similar to other data
structures, it defines a vector of itself, `PingsT`. In this way, `mbes_ping::PingsT`
can be used to represent all multibeam swaths from the deployment.
The data structure also contains information about the vehicle positions if available.
It also contains a flag, `first_in_file_`, that is set to `true` if the swath
was the first one of a parsed file, and otherwise to `false`.

All multibeam data structures defined below can be converted into this
common `mbes_ping` type through conversion functions (see [the example](https://github.com/nilsbore/auvlib/blob/master/example_projects/data_project/src/example_reader.cpp)).

### Other data structures

The `data_tools` project contains the following libraries: `navi_data`, `all_data`, `xtf_data`, `gsf_data`, `csv_data`
for reading data of different types.

* `navi_data` - for reading ASCII data exported from NaviEdit, contains parsers for:
  * `mbes_ping` - standard auvlib multibeam swath data structure
  * `nav_entry` - NaviEdit navigation exports
* `all_data` - for reading `.all` data files from Kongsberg, contains data structures and parsers for:
  * `all_mbes_ping` - Kongsberg multibeam swath data structure
  * `all_nav_entry` - Kongsberg navigation info
  * `all_nav_depth` - Kongsberg depth info
  * `all_echosounder_depth` - Kongsberg single echosounder depth data
  * `all_sound_speed_profile` - Sound speed profile data
  * `all_raw_range_and_beam_angle` - Raw range and beam angle data
  * `all_installation_param` - Installation parameters
* `xtf_data` - for reading `.xtf` sidescan files
  * `xtf_sss_ping` - xtf side scan swath data structure
* `gsf_data` - for reading `.gsf` files containing various sensor data
  * `gsf_mbes_ping` - Kongsberg multibeam swath data structure
  * `gsf_nav_entry` - Kongsberg navigation info
  * `gsf_sound_speed` - Kongsberg depth info
* `csv_data` - for reading `.csv` navigation files collected for the change detection experiment
  * `csv_nav_entry` - csv navigation exports

Again, note that all multibeam data can be converted into `mbes_ping`.
This is the recommended path for software using the library, since the
same software can then be used for different data types.
See the [corresponding headers](https://github.com/nilsbore/auvlib/tree/master/src/data_tools/include/data_tools) to identify the conversion functions.

### Common patterns

All data structures contain equivalents to `PingsT` of `mbes_ping` for
representing arrays of the data. Similarly, all data types also have the
`first_in_file_` flag. Moreover, they also contain a readable time stamp
called `time_string_` and a counter of milliseconds since `1970-01-01 00:00`, called `time_stamp_`.

### Parsing different formats

The library supports parsing many different data types, as specified above.
Parsing is always done through the templated interfaces `parse_file` and `parse_folder`.
For example, if you want parse multibeam data from a folder of `.all` files, write:
```cpp
all_mbes_ping::PingsT pings = parse_folder<all_mbes_ping>(boost::filesystem::path("/path/to/folder"));
```
Or, if you want to read just one sidescan `.xtf` file:
```cpp
xtf_sss_ping::PingsT pings = parse_file<xtf_sss_ping>(boost::filesystem::path("/path/to/file.xtf"));
```

### Writing and reading binary files

Parsing, especially of ASCII data, can take a long time.
We might therefore want to save our data structures in a binary for
after we have parsed them so that we can read them faster next time.
This can be achieved by the templated `write_data` and `read_data` functions.
They can be used with all data structures defined in this library.
To save a bunch of `gsf` multibeam data that we have just parsed, write
```cpp
gsf_mbes_ping::PingsT pings = parse_folder<gsf_mbes_ping>(boost::filesystem::path("/path/to/folder"));
write_data(pings, boost::filesystem::path("/path/to/file.cereal"));
```
Then you can use the following code to read them faster next time:
```cpp
gsf_mbes_ping::PingsT pings = read_data<gsf_mbes_ping>(boost::filesystem::path("/path/to/file.cereal"));
```

## Running the SLAM toy example

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

## Processing to create real SLAM submaps

### navi_data

The `navi_data` library contains tools for parsing files exported from NaviEdit.
See this example program, also available in [the test folder](https://github.com/nilsbore/auvlib/blob/master/src/apps/test/test_parse_navi_data.cpp).

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
the [apps directory](https://github.com/nilsbore/auvlib/blob/master/src/apps/src/parse_gsf_data.cpp):

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
To know which files to look at, it may also help to look at the [tests in `data_tools`](https://github.com/nilsbore/auvlib/blob/master/src/data_tools/src/test_gsf.cpp).

## Benchmarking SLAM and registration

The tests also [contains an example](https://github.com/nilsbore/auvlib/blob/master/src/apps/test/test_parse_navi_data.cpp)
of how to use the benchmark system. Also check out the [benchmark headers](https://github.com/nilsbore/auvlib/blob/master/src/data_tools/include/data_tools/benchmark.h) for details.
After having read your files like above, simply do something like the following:
```cpp
// create a new benchmark object with some arbitrary name
track_error_benchmark benchmark("medgaz");

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
track_error_benchmark benchmark = read_data<track_error_benchmark>(boost::filesystem::path("my_benchmark.cereal"));

mbes_ping::PingsT optimized_pings;
// fill in the pings using some slam algorithm ...
// add the benchmark with a random name e.g. "slam"
benchmark.add_benchmark(optimized_pings, "slam");

// print the results of the benchmark
benchmark.print_summary();

// save the benchmark to disk, for use later
write_data(benchmark, boost::filesystem::path("my_benchmark.cereal"));
```

### Registration benchmarking

In the file where we do our registration, we can create another benchmark
that summarizes all of our registration results.
Again, add our optimized pings to the benchmark, and save the results.
```cpp
// read data from disk
mbes_ping::PingsT initial_pings = read_data<mbes_ping::PingsT>(pings_path);

// create a new benchmark with some arbitrary name
registration_summary_benchmark benchmark("medgaz");

for (const pair<int, int>& match : matches) {
    // get just the pings of the two submaps
    mbes_ping::PingsT pair_pings = registration_summary_benchmark::get_submap_pings_pair(initial_pings, match.first, match.second);
    mbes_ping::PingsT optimized_pings;
    // fill in the pings using some registration algorithm ...
    benchmark.add_registration_benchmark(pings_pair, optimized_pings, match.first, match.second);
}

// print the results of the benchmark
benchmark.print_summary();

// save the benchmark to disk, for use later
write_data(benchmark, boost::filesystem::path("my_registration_benchmark.cereal"));
```
