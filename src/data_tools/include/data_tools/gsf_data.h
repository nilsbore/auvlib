#ifndef GSF_DATA_H
#define GSF_DATA_H

#include <data_tools/data_structures.h>
#include <data_tools/navi_data.h>

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

template <>
gsf_nav_entry::EntriesT parse_file(const boost::filesystem::path& file);

template <>
gsf_sound_speed::SpeedsT parse_file(const boost::filesystem::path& file);

template <>
gsf_mbes_ping::PingsT parse_file(const boost::filesystem::path& file);

void match_sound_speeds(gsf_mbes_ping::PingsT& pings, gsf_sound_speed::SpeedsT& speeds);

mbes_ping::PingsT convert_matched_entries(gsf_mbes_ping::PingsT& pings, gsf_nav_entry::EntriesT& entries);

mbes_ping::PingsT convert_pings(gsf_mbes_ping::PingsT& pings);

#endif // GSF_DATA_H
