/* Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <map>
#include <Eigen/Dense>
#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/tuple.hpp>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <boost/range.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI // For windows
  #define M_PI 3.14159265358979323846
#endif

const bool DEBUG_OUTPUT = false;

namespace std_data {

struct mbes_ping
{
    // data structure used to store a collection of mbes_pings
    using PingsT = std::vector<mbes_ping, Eigen::aligned_allocator<mbes_ping> >;

    unsigned int id_; // unique ID of swath
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    double heading_; // heading of vehicle
    double heave_; // heave of vehicle
    double pitch_; // pitch of vehicle
    double roll_; // roll of vehicle
    bool first_in_file_; // is first entry in file?
    Eigen::Vector3d pos_; // NOTE: this comes from associating ping with nav data

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > beams; // all 3d points in swath
    std::vector<double> back_scatter;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(heading_),
		   CEREAL_NVP(heave_), CEREAL_NVP(pitch_), CEREAL_NVP(roll_), CEREAL_NVP(first_in_file_),
           CEREAL_NVP(pos_), CEREAL_NVP(beams), CEREAL_NVP(back_scatter));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct sss_ping_side
{
    std::vector<int> pings;
    double slant_range;
    double time_duration;
    double tilt_angle;
    double beam_width;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(pings), CEREAL_NVP(slant_range), CEREAL_NVP(time_duration), CEREAL_NVP(tilt_angle), CEREAL_NVP(beam_width));
    }
};

struct sss_ping
{
    using PingsT = std::vector<sss_ping, Eigen::aligned_allocator<sss_ping> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp

    sss_ping_side port;
    sss_ping_side stbd;
    bool first_in_file_;
    double heading_;
    double pitch_;
    double roll_;
    double lat_;
    double long_;
    double sound_vel_;
    Eigen::Vector3d pos_; // NOTE: this comes from associating ping with nav data

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(port), CEREAL_NVP(stbd), CEREAL_NVP(first_in_file_), CEREAL_NVP(heading_),
           CEREAL_NVP(pitch_), CEREAL_NVP(roll_), CEREAL_NVP(lat_), CEREAL_NVP(long_), CEREAL_NVP(sound_vel_), CEREAL_NVP(pos_));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct nav_entry
{
    // data structure used to store a collection of nav_entries
    using EntriesT = std::vector<nav_entry, Eigen::aligned_allocator<nav_entry> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    bool first_in_file_; // is first entry in a file?
    Eigen::Vector3d pos_; // 3d position of vehicle
	
    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(first_in_file_), CEREAL_NVP(pos_));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct attitude_entry
{
    using EntriesT = std::vector<attitude_entry, Eigen::aligned_allocator<attitude_entry> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    bool first_in_file_; // is first entry in a file?
    double roll;
    double pitch;
    double yaw;
    double heave;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(first_in_file_),
           CEREAL_NVP(roll), CEREAL_NVP(pitch), CEREAL_NVP(yaw), CEREAL_NVP(heave));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct pt_submaps
{
    using PointsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
    using TransT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
    using RotsT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
    using AngsT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
    using MatchesT = std::vector<std::pair<int, int> >; // tells us which maps overlap
    using ConstraintsT = std::vector<std::tuple<int, int, Eigen::Vector3d, Eigen::Vector3d> >;
    using BoundsT = std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >;
    using CovsT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;

    std::string dataset_name; // arbitrary name of the dataset, without spaces
    PointsT points; // Nx3 matrices with all points in the submaps
    TransT trans; // translation of submaps
    RotsT rots; // rotation matrices of submaps, same as angles
    AngsT angles; // euler angles of submaps, same as rots
    MatchesT matches; // overlapping submap matches, containing vector indices of matches
    ConstraintsT binary_constraints; // consecutive submaps, containing vector indices of matches
    BoundsT bounds; // bounds of the submap, bb(0, 0) - min x, bb(0, 1) - min y, bb(1, 0) - max x, bb(1, 1) - max y
    PointsT tracks; // the vehicle track within the submap
    CovsT track_end_covs; // the uncertainty of the last point in the track given the first point
    
    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(dataset_name), CEREAL_NVP(points), CEREAL_NVP(trans), CEREAL_NVP(rots),
           CEREAL_NVP(angles), CEREAL_NVP(matches), CEREAL_NVP(binary_constraints),
           CEREAL_NVP(bounds), CEREAL_NVP(tracks), CEREAL_NVP(track_end_covs));
    }
};

template <typename T>
std::vector<T, Eigen::aligned_allocator<T> > parse_file(const boost::filesystem::path& file)
{
    std::vector<T, Eigen::aligned_allocator<T> > rtn;
    return rtn;
}

template <typename T>
std::vector<T, Eigen::aligned_allocator<T> > parse_file_from_str(const std::string& file)
{
    return parse_file<T>(boost::filesystem::path(file));
}

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

template <typename T>
std::vector<T, Eigen::aligned_allocator<T> > parse_folder_from_str(const std::string& folder)
{
    return parse_folder<T>(boost::filesystem::path(folder));
}

template <typename T>
T read_data(const boost::filesystem::path& path)
{
    if (!boost::filesystem::exists(path)) {
        std::cout << "File " << path << " does not exist..." << std::endl;
        exit(0);
    }

    T rtn;
    std::ifstream is(path.string(), std::ifstream::binary);
    {
        cereal::BinaryInputArchive archive(is);
        archive(rtn);
    }
    is.close();

    return rtn;
}

template <typename T>
T read_data_from_str(const std::string& path)
{
    return read_data<T>(boost::filesystem::path(path));
}


template <typename T>
void write_data(T& data, const boost::filesystem::path& path)
{
    std::ofstream os(path.string(), std::ofstream::binary);
	{
		cereal::BinaryOutputArchive archive(os);
        archive(data);
	}
    os.close();
}

template <typename T>
void write_data_from_str(T& data, const std::string& path)
{
    write_data<T>(data, boost::filesystem::path(path));
}

std::string time_string_from_time_stamp(long long time_stamp_);

} // namespace std_data

#endif // DATA_STRUCTURES_H
