#ifndef XTF_DATA_H
#define XTF_DATA_H

#include <data_tools/navi_data.h>
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

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

template <>
xtf_sss_ping::PingsT parse_file(const boost::filesystem::path& file);

cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings);

#endif // XTF_DATA_H
