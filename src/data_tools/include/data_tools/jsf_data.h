/* Copyright 2018 Nils Bore (nbore@kth.se), Yiping Xie (yipingx@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JSF_DATA_H
#define JSF_DATA_H

#include <data_tools/std_data.h>
//#include <data_tools/xtf_data.h>
#include <libjsf/jsf.h>
#include <Eigen/Dense>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <opencv2/core/core.hpp>
#include <unordered_map> 
#ifndef M_PI // For windows
  #define M_PI 3.14159265358979323846
#endif

namespace jsf_data {

struct jsf_sss_ping_side
{
    std::vector<float> pings;
    std::vector<float> pings_phase;
    double time_duration;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(pings), CEREAL_NVP(pings_phase), CEREAL_NVP(time_duration));
    }
};

struct jsf_sss_ping
{
    using PingsT = std::vector<jsf_sss_ping, Eigen::aligned_allocator<jsf_sss_ping> >;

    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp

    jsf_sss_ping_side port;
    jsf_sss_ping_side stbd;
    bool first_in_file_;
    Eigen::Vector3d rpy;
    double lat_;
    double long_;
    std::string utm_zone;
    double sound_vel;
    uint16_t frequency;
    Eigen::Vector3d pos_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(port), CEREAL_NVP(stbd), CEREAL_NVP(first_in_file_), CEREAL_NVP(rpy), CEREAL_NVP(lat_), CEREAL_NVP(long_), CEREAL_NVP(utm_zone), CEREAL_NVP(sound_vel), CEREAL_NVP(frequency), CEREAL_NVP(pos_));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct jsf_dvl_ping
{
    using PingsT = std::vector<jsf_dvl_ping, Eigen::aligned_allocator<jsf_dvl_ping> >;

    bool first_in_file_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp

    Eigen::Vector4d dist_to_bottom_; // Disctance to bottom in meter for up to 4 beams
    bool ship_coord_; // true -> velocity in ship coordinates; false -> velocity in earth coordinates
    Eigen::Vector3d vel_wrt_bottom_; // (x,y,z) Velocity with respect to the bottom in m/second
    Eigen::Vector3d vel_wrt_water_; // (x,y,z) Velocity with respect to a water layer in m/second
    
    double depth_; // Depth from depth sensor in meters
    double pitch_; // Pitch in radian + Bow up
    double roll_; // Roll in radian + Port up
    double heading_; // Heading in radian 

    double salinity_; // Salinity in 1 part per thousand
    double temp_; // Temperature in degree Celsius
    double sound_vel_; // Sound velocity in meters per second

    std::unordered_map<std::string,bool> flag_; // flag indicates which values present
    bool error_; // true -> error detected

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(first_in_file_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(ship_coord_), CEREAL_NVP(dist_to_bottom_), CEREAL_NVP(vel_wrt_bottom_), CEREAL_NVP(vel_wrt_water_), 
        CEREAL_NVP(depth_), CEREAL_NVP(pitch_), CEREAL_NVP(roll_), CEREAL_NVP(heading_), CEREAL_NVP(salinity_), CEREAL_NVP(temp_), CEREAL_NVP(sound_vel_),  CEREAL_NVP(flag_), CEREAL_NVP(error_));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

cv::Mat make_waterfall_image(const jsf_sss_ping::PingsT& pings);
void show_waterfall_image(const jsf_sss_ping::PingsT& pings);
jsf_sss_ping::PingsT filter_frequency(const jsf_sss_ping::PingsT& pings, int desired_freq);
std_data::sss_ping::PingsT convert_to_xtf_pings(const jsf_sss_ping::PingsT& pings);


} // namespace jsf_data

namespace std_data {

template <>
jsf_data::jsf_sss_ping::PingsT parse_file(const boost::filesystem::path& file);

template <>
jsf_data::jsf_dvl_ping::PingsT parse_file(const boost::filesystem::path& file);

}

#endif // JSF_DATA_H
