#include <cereal/archives/json.hpp>

#include <cxxopts.hpp>
#include <data_tools/gsf_data.h>
#include <data_tools/csv_data.h>
#include <data_tools/xtf_data.h>
#include <data_tools/navi_data.h>
#include <data_tools/transforms.h>

#include <chrono>

using namespace std;

int main(int argc, char** argv)
{
    string folder_str;
    string file_str;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("folder", "Input folder containing mbes files", cxxopts::value(folder_str))
      ("file", "Output file", cxxopts::value(file_str));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide input swaths and sss args..." << endl;
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

    mbes_ping::PingsT std_pings;
    {
        gsf_mbes_ping::PingsT pings = parse_folder<gsf_mbes_ping>(folder);
        std::stable_sort(pings.begin(), pings.end(), [](const gsf_mbes_ping& ping1, const gsf_mbes_ping& ping2) {
            return ping1.time_stamp_ < ping2.time_stamp_;
        });
        std_pings = convert_pings(pings);
    }

    return 0;
}

