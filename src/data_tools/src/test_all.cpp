#include <data_tools/all_data.h>

using namespace std;
using namespace data_structures;
using namespace all_data;

int main(int argc, char** argv)
{
    boost::filesystem::path path(argv[1]);

	cout << "Parsing file " << path << endl;

	all_nav_depth::EntriesT depths = parse_folder<all_nav_depth>(path);
	all_echosounder_depth::EntriesT echo_depths = parse_folder<all_echosounder_depth>(path);
	all_mbes_ping::PingsT pings = parse_file<all_mbes_ping>(path);
	all_nav_entry::EntriesT entries = parse_file<all_nav_entry>(path);
	//all_nav_entry::EntriesT entries = parse_folder<all_nav_entry>(path);
	//all_mbes_ping::PingsT pings = parse_folder<all_mbes_ping>(path);

	cout << "Got " << pings.size() << " mbes pings, and " << entries.size() << " nav entries, and " << depths.size() << " nav depths.." << endl;
	//cout << "Got " << pings.size() << " mbes pings, and " << entries.size() << " nav entries" << endl;
	cout << "Got " << echo_depths.size() << " echo depth entries" << endl;

    /*
    for (const all_mbes_ping& ping : pings) {
        cout << "Got back ping " << ping.id_ << " with time: " << ping.time_string_ << endl;
    }
    */

    return 0;
}
