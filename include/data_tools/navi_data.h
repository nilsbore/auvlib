#ifndef NAVI_DATA_H
#define NAVI_DATA_H

#include <eigen_cereal/eigen_cereal.h>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/range.hpp>

using TransT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using ObsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
using MatchesT = std::vector<std::pair<int, int> >; // tells us which maps overlap
using BBsT = std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >;

// NOTE: this seems reasonable
struct mbes_ping
{

    unsigned int id_;
    std::string time_string_;
    long long time_stamp_;
    double heading_;
    double heave_;
    double pitch_;
    double roll_;
    bool first_in_file_;
    Eigen::Vector3d pos_; // NOTE: this comes from associating ping with nav data

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > beams;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(heading_),
		   CEREAL_NVP(heave_), CEREAL_NVP(pitch_), CEREAL_NVP(roll_), CEREAL_NVP(first_in_file_));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct nav_entry
{

    std::string time_string_;
    long long time_stamp_;
    bool first_in_file_;
    Eigen::Vector3d pos_;
	
    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(first_in_file_));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

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
