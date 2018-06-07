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

void match_timestamps(mbes_ping::PingsT& pings, nav_entry::EntriesT& entries);
void view_cloud(const mbes_ping::PingsT& pings);

template <typename T>
std::vector<T, Eigen::aligned_allocator<T> > parse_file(const boost::filesystem::path& file)
{
    std::vector<T, Eigen::aligned_allocator<T> > rtn;
    return rtn;
}

template <>
mbes_ping::PingsT parse_file(const boost::filesystem::path& file);

template <>
nav_entry::EntriesT parse_file(const boost::filesystem::path& file);

template <typename T>
std::vector<T, Eigen::aligned_allocator<T> > parse_folder(const boost::filesystem::path& folder)
{
	
    std::vector<T, Eigen::aligned_allocator<T> > pings;

    if(!boost::filesystem::is_directory(folder)) {
        std::cout << folder << " is not a directory containing" << std::endl;
        return pings;
    }

    for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(folder), {})) {
        std::cout << entry << "\n";
	    if (boost::filesystem::is_directory(entry.path())) {
		    continue;
		}
        std::vector<T, Eigen::aligned_allocator<T> > file_pings = parse_file<T>(entry.path());
		pings.insert(pings.end(), file_pings.begin(), file_pings.end());
    }

    return pings;
}

void divide_tracks(mbes_ping::PingsT& pings);
void divide_tracks_equal(mbes_ping::PingsT& pings);
std::tuple<ObsT, TransT, AngsT, MatchesT, BBsT> create_submaps(const mbes_ping::PingsT& pings);
void visualize_submaps(ObsT& submaps, TransT& trans, AngsT& angs);

#endif // NAVI_DATA_H
