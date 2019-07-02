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

#ifndef XTF_DATA_H
#define XTF_DATA_H

#include <data_tools/navi_data.h>
#include <eigen3/Eigen/Dense>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <opencv2/core/core.hpp>

namespace xtf_data {

struct xtf_sss_ping_side
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

struct xtf_sss_ping
{
    using PingsT = std::vector<xtf_sss_ping, Eigen::aligned_allocator<xtf_sss_ping> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp

    xtf_sss_ping_side port;
    xtf_sss_ping_side stbd;
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

cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings);
Eigen::MatrixXd make_eigen_waterfall_image(const xtf_sss_ping::PingsT& pings);
void show_waterfall_image(const xtf_sss_ping::PingsT& pings);

xtf_sss_ping::PingsT correct_sensor_offset(const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& sensor_offset);

} // namespace xtf_data

namespace std_data {

template <>
xtf_data::xtf_sss_ping::PingsT parse_file(const boost::filesystem::path& file);

}

#endif // XTF_DATA_H
