#include <cxxopts.hpp>
#include <data_tools/gsf_data.h>
#include <data_tools/csv_data.h>
#include <data_tools/xtf_data.h>
#include <data_tools/navi_data.h>
#include <data_tools/all_data.h>

using namespace std;
using namespace xtf_data;
using namespace std_data;
using namespace gsf_data;
using namespace all_data;
using namespace navi_data;

int main(int argc, char** argv)
{
    string folder_str;
    string file_str;
    string type;

	cxxopts::Options options("example_reader", "Reads different mbes and sss file formats and saves them to a common format");
	options.add_options()
      ("help", "Print help")
      ("folder", "Input folder containing mbes files", cxxopts::value(folder_str))
      ("file", "Output file", cxxopts::value(file_str))
      ("type", "Type of data to read, options: all, xtf, navi, gsf", cxxopts::value(type));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide folder containing mbes or sss files..." << endl;
		exit(0);
    }
    if (result.count("type") == 0) {
		cout << "Please provide input type, options: all, xtf, navi, gsf" << endl;
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

    // if side scan, read and save, then return
    if (type == "xtf") {
        xtf_sss_ping::PingsT pings = parse_folder<xtf_sss_ping>(folder);
        write_data(pings, path);
        cout << "Read folder " << folder << " with type " << type << " and saved to file " << path << endl;
        return 0;
    }

    // otherwise we have multibeam data, read and save
    mbes_ping::PingsT std_pings;
    if (type == "gsf") {
        gsf_mbes_ping::PingsT pings = parse_folder<gsf_mbes_ping>(folder);
        std::stable_sort(pings.begin(), pings.end(), [](const gsf_mbes_ping& ping1, const gsf_mbes_ping& ping2) {
            return ping1.time_stamp_ < ping2.time_stamp_;
        });
        std_pings = convert_pings(pings);
    }
    else if (type == "all") {
	    all_nav_entry::EntriesT entries = parse_folder<all_nav_entry>(folder);
        all_mbes_ping::PingsT pings = parse_folder<all_mbes_ping>(folder);
        std_pings = convert_matched_entries(pings, entries);
    }
    else if (type == "navi") {
        boost::filesystem::path nav_folder = folder.parent_path() / "NavUTM";
        std_pings = parse_folder<mbes_ping>(folder);
        nav_entry::EntriesT entries = parse_folder<nav_entry>(nav_folder);
        match_timestamps(std_pings, entries);
    }
    else {
        cout << "Type " << type << " is not supported!" << endl;
        return 0;
    }

    write_data(std_pings, path);
    cout << "Read folder " << folder << " with type conversion " << type << "-> mbes_ping and saved to file " << path << endl;

    return 0;
}

