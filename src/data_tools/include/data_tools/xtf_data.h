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
#include <Eigen/Dense>
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
/**
	make_waterfall_image can average over rows and columns to make image fit a given width and height.  If the number of elements is not divisible by the width then the furthest returns are just dropped to make it fit.  The ping intensities are often not well normalized.  Here we give default of 2^16 as the default maxPingIntensity.  All returns higher than maxPingIntensity will be capped at that value.  Similarly sometimes it seems the intesities are signed inetgers?  Not sure that happens but if it does change the minPingIntensity from its default 0.  
*/
cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings, long width, long height=0, long maxPingIntensity=65535, long minPingIntensity=0);
/**
	normalize_waterfall can average over rows and columns to make image fit a given width and height.  If the number of elements is not divisible by the width then the furthest returns are just dropped to make it fit. 

Here you can also just conver part of the pings by specifing a start ping and end ping.  Specifying 0 for these uses all the pings.

 The ping intensities are often not well normalized. Here we normalize each column to have its intensities cover 0 to 255 better.
The params = [width, height, startping, endping]
*/
cv::Mat  normalize_waterfall(const xtf_sss_ping::PingsT& pings, long* params);
Eigen::MatrixXd make_eigen_waterfall_image(const xtf_sss_ping::PingsT& pings);
void show_waterfall_image(const xtf_sss_ping::PingsT& pings);

/**
void  regularize_pings( xtf_sss_ping::PingsT& pings, const long * port_nadir, const long * stbd_nadir, double nadir_angle=0.27925268);

This will use the two points along the nadir to define a flat bottom and then apply a cotan(incident angle) adjustment to the intensities of the pings.  One needs to call findNadirPort and findNadirStbd first and also give a value for the nadir angle. 

*/

void  regularize_pings( xtf_sss_ping::PingsT& pings, const long * port_nadir, const long * stbd_nadir, double nadir_angle=0.27925268);
/**
 RemoveLineArtifact_port(xtf_sss_ping::PingsT& pings, long * nadir, double minArtifactRange minr=30,  double minArtifactRange maxr=90, bool setzero=false)

Our Port side Sidescan has an artifact that appears as a bright spot in about 25 cm of bins.  Which bins varies continously and smoothly in time accross bins.
You can give some hints as to the range that the artifact wanders over and choose to set the values to 0 instead of trying to fill them with 'average' values nearby plus noise.

nadir -  the array returned from calling findNadirPort. This array will be changed to contain the detected bin of the artifact.  

 **/


void removeLineArtifact_port(xtf_sss_ping::PingsT& pings, long * nadir, const double minArtifactRange=30,  const double maxArtifactRange=90, const bool setzero=false);
/**
 RemoveLineArtifact_stbd(xtf_sss_ping::PingsT& pings, long * nadir, double minArtifactRange minr=30,  double minArtifactRange maxr=90, bool setzero=false)

Our Stbd side Sidescan has an artifact that appears as a bright spot in about 25 cm of bins.  Which bins varies continously and smoothly in time accross bins.
You can give some hints as to the range that the artifact wanders over and choose to set the values to 0 instead of trying to fill them with 'average' values nearby plus noise.

nadir -  the array returned from calling findNadirStbd. This array will be changed to contain the detected bin of the artifact.  

 **/
void removeLineArtifact_stbd(xtf_sss_ping::PingsT& pings, long * nadir, const double minArtifactRange=30,  const double maxArtifactRange=90, const bool setzero=false);
/**
This will look thru the pings on the port side and locate the index to the nadir.
It will also set to 0 all intensities less then the minalt.  It also sets any intensities below zero or above 2^29 to 0.
pings - the object to check
nadir - a long array of length pings.size() that will hold the index to the nadir for each port side ping,
minalt -  This should be the minimum altitude in m that the sone will see.  All data nearer than this will be set to 0.
minintensityatnadir - This allow you to adjust things if it fails to find the nadir.
*/
 void findNadirPort(xtf_sss_ping::PingsT& pings, long * nadir, double minalt=10, long minintensityatnadir=500);

/**
This will look thru the pings on the starboard side and locate the index to the nadir.
It will also set to 0 all intensities less then the minalt.  It also sets any intensities below zero or above 2^29 to 0.
pings - the object to check
nadir - a long array of length pings.size() that will hold the index to the nadir for each port side ping,
minalt -  This should be the minimum altitude in m that the sone will see.  All data nearer than this will be set to 0.
minintensityatnadir - This allow you to adjust things if it fails to find the nadir.
*/
 void findNadirStbd(xtf_sss_ping::PingsT& pings, long * nadir, double minalt=10, long minintensityatnadir=500);

xtf_sss_ping::PingsT correct_sensor_offset(const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& sensor_offset);
xtf_sss_ping::PingsT match_attitudes(const xtf_sss_ping::PingsT& pings, const std_data::attitude_entry::EntriesT& entries);

} // namespace xtf_data

namespace std_data {

template <>
xtf_data::xtf_sss_ping::PingsT parse_file(const boost::filesystem::path& file);

}

#endif // XTF_DATA_H
