#ifndef NAVI_DATA_H
#define NAVI_DATA_H

#include <eigen3/Eigen/Dense>
#include <data_tools/data_structures.h>

using TransT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using ObsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
using MatchesT = std::vector<std::pair<int, int> >; // tells us which maps overlap
using BBsT = std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >;

void match_timestamps(mbes_ping::PingsT& pings, nav_entry::EntriesT& entries);
void view_cloud(const mbes_ping::PingsT& pings);

template <>
mbes_ping::PingsT parse_file(const boost::filesystem::path& file);

template <>
nav_entry::EntriesT parse_file(const boost::filesystem::path& file);

void divide_tracks(mbes_ping::PingsT& pings);
void divide_tracks_equal(mbes_ping::PingsT& pings);
std::tuple<ObsT, TransT, AngsT, MatchesT, BBsT, ObsT> create_submaps(const mbes_ping::PingsT& pings);
void visualize_submaps(ObsT& submaps, TransT& trans, AngsT& angs);

#endif // NAVI_DATA_H
