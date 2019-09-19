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

#ifndef GSF_DATA_H
#define GSF_DATA_H

#include <data_tools/std_data.h>
#include <data_tools/navi_data.h>
#include <data_tools/csv_data.h>

namespace gsf_data {

struct gsf_mbes_ping {

    using PingsT = std::vector<gsf_mbes_ping, Eigen::aligned_allocator<gsf_mbes_ping> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    std::vector<double> travel_times; // time_stamp_
    std::vector<double> beam_angles; // time_stamp_
    std::vector<double> distances; // distances, to be filled in
    std::vector<int> amplitudes; // amplitudes
    bool first_in_file_;

    double heading_;
    double pitch_;
    double roll_;
    double lat_;
    double long_;
    double depth_;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > beams; // all 3d points in swath

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(travel_times), CEREAL_NVP(beam_angles),
           CEREAL_NVP(distances), CEREAL_NVP(amplitudes), CEREAL_NVP(first_in_file_), CEREAL_NVP(heading_),
           CEREAL_NVP(pitch_), CEREAL_NVP(roll_), CEREAL_NVP(lat_), CEREAL_NVP(long_), CEREAL_NVP(depth_), CEREAL_NVP(beams));
    }

};

struct gsf_sound_speed
{
    using SpeedsT = std::vector<gsf_sound_speed, Eigen::aligned_allocator<gsf_sound_speed> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp

    double near_speed;
    double below_speed;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(near_speed), CEREAL_NVP(below_speed));
    }

};

struct gsf_nav_entry
{
    using EntriesT = std::vector<gsf_nav_entry, Eigen::aligned_allocator<gsf_nav_entry> >;

    unsigned int id_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    
    double roll_; // roll of vehicle
    double pitch_; // pitch of vehicle
    double yaw_;

    double lat_;
    double long_;

    double altitude;

    Eigen::Vector3d pos_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(id_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(roll_), CEREAL_NVP(pitch_), CEREAL_NVP(yaw_), CEREAL_NVP(lat_), CEREAL_NVP(long_), CEREAL_NVP(altitude), CEREAL_NVP(pos_));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void match_sound_speeds(gsf_mbes_ping::PingsT& pings, gsf_sound_speed::SpeedsT& speeds);

std_data::mbes_ping::PingsT convert_matched_entries(gsf_mbes_ping::PingsT& pings, gsf_nav_entry::EntriesT& entries);

std_data::mbes_ping::PingsT convert_matched_entries(gsf_data::gsf_mbes_ping::PingsT& pings, csv_data::csv_nav_entry::EntriesT& entries);

std_data::mbes_ping::PingsT convert_pings(gsf_mbes_ping::PingsT& pings);

} // namespace gsf_data

namespace std_data {

template <>
gsf_data::gsf_nav_entry::EntriesT parse_file(const boost::filesystem::path& file);

template <>
gsf_data::gsf_sound_speed::SpeedsT parse_file(const boost::filesystem::path& file);

template <>
gsf_data::gsf_mbes_ping::PingsT parse_file(const boost::filesystem::path& file);

}

#endif // GSF_DATA_H
