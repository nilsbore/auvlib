#ifndef NAVI_DATA_H
#define NAVI_DATA_H

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/range.hpp>

#include <data_tools/data_structures.h>

using TransT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using ObsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
using MatchesT = std::vector<std::pair<int, int> >; // tells us which maps overlap
using BBsT = std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >;

void match_timestamps(std::vector<mbes_ping>& pings, std::vector<nav_entry>& entries);
void view_cloud(const std::vector<mbes_ping>& pings);

template <typename T>
std::vector<T> read_file(const boost::filesystem::path& file)
{
    std::vector<T> rtn;
    return rtn;
}

template <>
std::vector<mbes_ping> read_file(const boost::filesystem::path& file);

template <>
std::vector<nav_entry> read_file(const boost::filesystem::path& file);

template <typename T>
std::vector<T> read_folder(const boost::filesystem::path& folder)
{
	
    std::vector<T> pings;

    if(!boost::filesystem::is_directory(folder)) {
        std::cout << folder << " is not a directory containing" << std::endl;
        return pings;
    }

    for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(folder), {})) {
        std::cout << entry << "\n";
	    if (boost::filesystem::is_directory(entry.path())) {
		    continue;
		}
        std::vector<T> file_pings = read_file<T>(entry.path());
		pings.insert(pings.end(), file_pings.begin(), file_pings.end());
    }

    return pings;
}

void divide_tracks(std::vector<mbes_ping>& pings);
void divide_tracks_equal(std::vector<mbes_ping>& pings);
std::tuple<ObsT, TransT, AngsT, MatchesT, BBsT> create_submaps(const std::vector<mbes_ping>& pings);
void visualize_submaps(ObsT& submaps, TransT& trans, AngsT& angs);

#endif // NAVI_DATA_H
