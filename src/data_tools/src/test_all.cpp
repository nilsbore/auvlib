#include <data_tools/all_data.h>

using namespace std;

int main(int argc, char** argv)
{
    boost::filesystem::path path(argv[1]);

	cout << "Parsing file " << path << endl;

	//all_nav_entry::EntriesT entries = parse_file<all_nav_entry>(path);
	//all_mbes_ping::PingsT pings = parse_file<all_mbes_ping>(path);
	all_nav_entry::EntriesT entries = parse_folder<all_nav_entry>(path);
	all_mbes_ping::PingsT pings = parse_folder<all_mbes_ping>(path);

	cout << "Got " << pings.size() << " mbes pings, and " << entries.size() << " nav entries" << endl;

    return 0;
}
