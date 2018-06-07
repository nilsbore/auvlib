#include <data_tools/data_structures.h>

using namespace std;

// instantiate all versions needed to read the structs
template nav_entry::EntriesT read_data<nav_entry::EntriesT>(const boost::filesystem::path& path);
template mbes_ping::PingsT read_data<mbes_ping::PingsT>(const boost::filesystem::path& path);
template pt_submaps read_data<pt_submaps>(const boost::filesystem::path& path);
template gp_submaps read_data<gp_submaps>(const boost::filesystem::path& path);

// instantiate all versions needed to write the structs
template void write_data<nav_entry::EntriesT>(nav_entry::EntriesT& data, const boost::filesystem::path& path);
template void write_data<mbes_ping::PingsT>(mbes_ping::PingsT& data, const boost::filesystem::path& path);
template void write_data<pt_submaps>(pt_submaps& data, const boost::filesystem::path& path);
template void write_data<gp_submaps>(gp_submaps& data, const boost::filesystem::path& path);
