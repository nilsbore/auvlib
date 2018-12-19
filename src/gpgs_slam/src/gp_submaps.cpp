#include <gpgs_slam/gp_submaps.h>

namespace std_data {

template gp_submaps read_data<gp_submaps>(const boost::filesystem::path& path);
template void write_data<gp_submaps>(gp_submaps& data, const boost::filesystem::path& path);

} // namespace std_data
