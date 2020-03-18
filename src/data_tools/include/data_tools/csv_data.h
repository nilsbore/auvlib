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

#ifndef CSV_DATA_H
#define CSV_DATA_H

#include <data_tools/std_data.h>
#include <data_tools/navi_data.h>
//#include <data_tools/xtf_data.h>

namespace csv_data {

struct csv_nav_entry
{
    using EntriesT = std::vector<csv_nav_entry, Eigen::aligned_allocator<csv_nav_entry> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    
    double roll_; // roll of vehicle
    double pitch_; // pitch of vehicle
    double heading_;

    double roll_std_; // roll of vehicle
    double pitch_std_; // pitch of vehicle
    double heading_std_;

    double lat_;
    double long_;

    double altitude;

    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(roll_),
           CEREAL_NVP(pitch_), CEREAL_NVP(heading_), CEREAL_NVP(roll_std_), CEREAL_NVP(pitch_std_),
           CEREAL_NVP(heading_std_), CEREAL_NVP(lat_), CEREAL_NVP(long_), CEREAL_NVP(altitude),
           CEREAL_NVP(pos_), CEREAL_NVP(vel_));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct csv_asvp_sound_speed
{
    using EntriesT = std::vector<csv_asvp_sound_speed, Eigen::aligned_allocator<csv_asvp_sound_speed> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp

    double lat_;
    double long_;

    Eigen::VectorXd dbars;
    Eigen::VectorXd vels;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(lat_),
           CEREAL_NVP(long_), CEREAL_NVP(dbars), CEREAL_NVP(vels));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std_data::sss_ping::PingsT convert_matched_entries(std_data::sss_ping::PingsT& pings, csv_data::csv_nav_entry::EntriesT& entries);

} // namespace csv_data

namespace std_data {

template <>
csv_data::csv_nav_entry::EntriesT parse_file(const boost::filesystem::path& file);

template <>
csv_data::csv_asvp_sound_speed::EntriesT parse_file(const boost::filesystem::path& file);

}

#endif // CSV_DATA_H
