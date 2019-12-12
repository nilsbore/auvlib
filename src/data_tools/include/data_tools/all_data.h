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

#ifndef ALL_DATA_H
#define ALL_DATA_H

#include <data_tools/navi_data.h>
#include <data_tools/csv_data.h>
#include <Eigen/Dense>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
//#include <cereal/types/vector.hpp>

namespace all_data {

struct all_mbes_ping {

    using PingsT = std::vector<all_mbes_ping, Eigen::aligned_allocator<all_mbes_ping> >;

    unsigned int id_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
	double heading_;
	double sound_vel_;
	double transducer_depth_;
    std::vector<float> reflectivities; // amplitudes
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > beams; // all 3d points in swath
    bool first_in_file_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(heading_),
		   CEREAL_NVP(sound_vel_), CEREAL_NVP(transducer_depth_), CEREAL_NVP(reflectivities),
           CEREAL_NVP(beams), CEREAL_NVP(first_in_file_));
    }

};

struct all_nav_entry {

    using EntriesT = std::vector<all_nav_entry, Eigen::aligned_allocator<all_nav_entry> >;

    unsigned int id_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    double lat_;
    double long_;
    double depth_;
	double heading_;
	double course_over_ground_;
    bool first_in_file_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_),
		   CEREAL_NVP(lat_), CEREAL_NVP(long_), CEREAL_NVP(depth_), CEREAL_NVP(heading_),
		   CEREAL_NVP(course_over_ground_), CEREAL_NVP(first_in_file_));
    }

};

struct all_nav_depth {

    using EntriesT = std::vector<all_nav_depth, Eigen::aligned_allocator<all_nav_depth> >;

    unsigned int id_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    double height;
    int height_type;
    bool first_in_file_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_),
		   CEREAL_NVP(height), CEREAL_NVP(height_type), CEREAL_NVP(first_in_file_));
    }

};

struct all_nav_attitude_sample {

    unsigned int ms_since_start;

    double roll;
    double pitch;
    double heading;
    double heave;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(ms_since_start), CEREAL_NVP(roll), CEREAL_NVP(pitch),
           CEREAL_NVP(heading), CEREAL_NVP(heave));
    }

};

struct all_nav_attitude {

    using EntriesT = std::vector<all_nav_attitude, Eigen::aligned_allocator<all_nav_attitude> >;

    unsigned int id_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    bool first_in_file_;

    std::vector<all_nav_attitude_sample> samples;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_),
		   CEREAL_NVP(first_in_file_), CEREAL_NVP(samples));
    }

};

struct all_echosounder_depth {

    using EntriesT = std::vector<all_echosounder_depth, Eigen::aligned_allocator<all_echosounder_depth> >;

    unsigned int id_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    double depth_;
    bool first_in_file_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_),
		   CEREAL_NVP(depth_), CEREAL_NVP(first_in_file_));
    }

};

std_data::mbes_ping::PingsT convert_matched_entries(all_mbes_ping::PingsT& pings, all_nav_entry::EntriesT& entries);
std_data::mbes_ping::PingsT match_attitude(std_data::mbes_ping::PingsT& pings, all_nav_attitude::EntriesT& entries);
csv_data::csv_asvp_sound_speed::EntriesT convert_sound_speeds(const all_mbes_ping::PingsT& pings);
std_data::attitude_entry::EntriesT convert_attitudes(const all_nav_attitude::EntriesT& attitudes);

class StreamParser {
private:

    std::function<void(all_mbes_ping)> mbes_callback;
    std::function<void(all_nav_entry)> nav_entry_callback;

public:

    StreamParser() {}

    bool parse_packet(const std::string& packet_load);

    void set_mbes_callback(const std::function<void(all_mbes_ping)>& callback)
    {
        mbes_callback = callback;
    }

    void set_nav_entry_callback(const std::function<void(all_nav_entry)>& callback)
    {
        nav_entry_callback = callback;
    }

};

} // namespace all_data

namespace std_data {

//template <typename ReturnType>
//std::vector<ReturnType, Eigen::aligned_allocator<ReturnType> > parse_file(const boost::filesystem::path& path);

template <>
all_data::all_mbes_ping::PingsT parse_file(const boost::filesystem::path& file);

template <>
all_data::all_nav_entry::EntriesT parse_file(const boost::filesystem::path& file);

template <>
all_data::all_nav_depth::EntriesT parse_file(const boost::filesystem::path& file);

template <>
all_data::all_nav_attitude::EntriesT parse_file(const boost::filesystem::path& file);

template <>
all_data::all_echosounder_depth::EntriesT parse_file(const boost::filesystem::path& file);

}

#endif // ALL_DATA_H
