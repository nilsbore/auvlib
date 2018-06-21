#include <data_tools/gsf_data.h>

#include <boost/date_time.hpp>
#include <gsf.h>

using namespace std;

/*
% VEHICLE_POSE_FILE VERSION 3
% 
% File:          dr_pose_est.data
% Date Created:  Thu Jun  9 22:06:01 2011
% Created from:  bpslam
% 
% Timing Statistics: 
%     Program took 0.533 minutes to process mission.
%     Total Mission Time: 157.063 minutes.
%     Start Nav Time: 1224108800.202
%     Stop Time:  1224118223.970
%     Start Map Time: 1224109095.920
%     Loop closure detection took 0.058 minutes to process.
%     Particle Weighting took -0.000 minutes to process.
%     GP Raytracing took 0.000 minutes to process.
%     GP Learning took 0.000 minutes to process.
% SLAM Statistics: 
%     Number of particles:    320
%     Average number of particles:    320.000
%     Number of resampling events passed:    0
%     Number of resampling events prevented: 0
%     Number of resampling events other: 0
%     Number of multibeam poses available: 51416
%     Number of multibeam poses stored: 11849
%     Number of multibeam poses sampled: 2954
%     Percentage of multibeam stored: 23.045%
% 
%     Percentage of multibeam sampled: 5.745%
% 
% 
% Pose Statistics:
% 	Number of Poses Written: 11849
% 
% 
% Each line of this file describes the pose of the vehicle relative to the local
% navigation frame. The vehicle poses may have been estimated since they are the
% locations at which stereo images or multibeam sonar data were acquired.
% 
% If a pose was estimated because it was the location images were acquired,
% additional information for that pose can be found in the file
% stereo_pose_est.data. The pose identifier can be used to locate matching
% poses.
% 
% The X and Y coordinates are produced using a local transverse Mercator 
% projection using the WGS84 ellipsoid and a central meridian at the origin
% latitude. You will probably want to use the provided latitude and longitude to
% produce coordinates in what map projection you require.
% 
% The first two lines of the data contain the latitude and longitude of the
% origin.
% 
% Each line contains the following items describing the pose of the vehicle:
% 
% 1) Pose identifier                   - integer value
% 2) Timestamp                         - in seconds
% 3) Latitude                          - in degrees
% 4) Longitude                         - in degrees
% 5) X position (Northing)             - in meters, relative to local nav frame
% 6) Y position (Easting)              - in meters, relative to local nav frame
% 7) Z position (Depth)                - in meters, relative to local nav frame
% 8) X-axis Euler angle (Roll)         - in radians, relative to local nav frame
% 9) Y-axis Euler angle (Pitch)        - in radians, relative to local nav frame
% 10) Z-axis Euler angle (Yaw/Heading) - in radians, relative to local nav frame
% 11) Altitude                         - in meters. (0 when unknown)
*/
template <>
gsf_nav_entry::EntriesT parse_file(const boost::filesystem::path& file)
{
    gsf_nav_entry::EntriesT entries;

    gsf_nav_entry entry;
    double x, y, z;
    double time_seconds;
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
	
    string line;
    std::ifstream infile(file.string());
    while (std::getline(infile, line))  // this does the checking!
    {
        if (line.empty() || line[0] == '%' || line[0] == '\n') {
            continue;
        }
        istringstream iss(line);

		iss >> entry.id_ >> time_seconds >> entry.lat_ >> entry.long_ >> x >> y >> z >> entry.roll_ >> entry.pitch_ >> entry.yaw_ >> entry.altitude;
        entry.pos_ = Eigen::Vector3d(x, y, z);

        entry.time_stamp_ = (long long)(1000. * time_seconds); // double seconds to milliseconds

        boost::posix_time::ptime t = epoch + boost::posix_time::milliseconds(entry.time_stamp_);

        stringstream time_ss;
        time_ss << t;
        entry.time_string_ = time_ss.str();

		entries.push_back(entry);
    }

	return entries;
}

/*
% SOUND_SPEED_FILE VERSION 1
% 
% Produced by mk_sound_speed
% 
% 
% Each line of this file describes the sound speed measured 
% at the time indicated.
% 
% On each line of the file are 4 items:
% 
% 1) Record identifier                  - integer value
% 2) Timestamp                        - in seconds
% 3) Sound speed at vehicle           - in meters/second
% 4) Mean sound speed beneath vehicle - in meters/second (best guess)
*/
template <>
gsf_sound_speed::SpeedsT parse_file(const boost::filesystem::path& file)
{
    gsf_sound_speed::SpeedsT speeds;

    gsf_sound_speed speed;
    double time_seconds;
    int id_;
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
	
    string line;
    std::ifstream infile(file.string());
    while (std::getline(infile, line))  // this does the checking!
    {
        if (line.empty() || line[0] == '%' || line[0] == '\n') {
            continue;
        }
        istringstream iss(line);

		iss >> id_ >> time_seconds >> speed.near_speed >> speed.below_speed;

        speed.time_stamp_ = (long long)(1000. * time_seconds); // double seconds to milliseconds

        boost::posix_time::ptime t = epoch + boost::posix_time::milliseconds(speed.time_stamp_);

        stringstream time_ss;
        time_ss << t;
        speed.time_string_ = time_ss.str();

		speeds.push_back(speed);
    }

	return speeds;
}

// reads multibeam swaths from a .gsf file
template <>
gsf_mbes_ping::PingsT parse_file(const boost::filesystem::path& file)
{
    gsf_mbes_ping::PingsT pings;
    if (boost::filesystem::extension(file) != ".gsf") {
        return pings;
    }

    if (!boost::filesystem::exists(file)) {
        cout << "File " << file << " does not exist, exiting..." << endl;
        exit(0);
    }
    int handle;
    //gsfOpen(file.string().c_str(), GSF_READONLY, &handle);
    if (gsfOpen(file.string().c_str(), GSF_READONLY, &handle) != 0 || handle < 0)
    {
        cout << "File " << file << " could not be opened!" << endl;
        return pings;
        //exit(0);
    }
    //cout << "Result: " << result << ", handle: " << handle << endl;

    gsfDataID data_id;
    gsfRecords records;
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");

    while (gsfRead(handle, GSF_NEXT_RECORD, &data_id, &records, nullptr, 0) != -1) {
        if (data_id.recordID == GSF_RECORD_SWATH_BATHYMETRY_PING) {
            gsf_mbes_ping ping;
            for (int i = 0; i < records.mb_ping.number_beams; ++i) {
                ping.travel_times.push_back(records.mb_ping.travel_time[i]);
                ping.beam_angles.push_back(records.mb_ping.beam_angle[i]);
            }

            long long sec = records.mb_ping.ping_time.tv_sec;
            long long nsec = records.mb_ping.ping_time.tv_nsec;
            ping.time_stamp_ = 1000*sec + nsec/1000000;
            boost::posix_time::ptime t = epoch + boost::posix_time::milliseconds(ping.time_stamp_);

            stringstream time_ss;
            time_ss << t;
            ping.time_string_ = time_ss.str();

            pings.push_back(ping);
        }
    }

    gsfClose(handle);

    return pings;
}

void match_sound_speeds(gsf_mbes_ping::PingsT& pings, gsf_sound_speed::SpeedsT& speeds)
{

    std::stable_sort(pings.begin(), pings.end(), [](const gsf_mbes_ping& ping1, const gsf_mbes_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });
    std::stable_sort(speeds.begin(), speeds.end(), [](const gsf_sound_speed& speed1, const gsf_sound_speed& speed2) {
        return speed1.time_stamp_ < speed2.time_stamp_;
    });

    auto pos = speeds.begin();
    for (gsf_mbes_ping& ping : pings) {
        pos = std::find_if(pos, speeds.end(), [&](const gsf_sound_speed& speed) {
            return speed.time_stamp_ > ping.time_stamp_;
        });
        double ss;
        if (pos == speeds.end()) {
            ss = speeds.back().below_speed;
        }
        else {
            ss = pos->below_speed;
        }
        for (int i = 0; i < ping.travel_times.size(); ++i) {
            ping.distances.push_back(0.5*ss*ping.travel_times[i]);
        }
    }

}

mbes_ping::PingsT convert_matched_entries(gsf_mbes_ping::PingsT& pings, gsf_nav_entry::EntriesT& entries)
{
    mbes_ping::PingsT new_pings;

    std::stable_sort(entries.begin(), entries.end(), [](const gsf_nav_entry& entry1, const gsf_nav_entry& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });

    auto pos = entries.begin();
    for (gsf_mbes_ping& ping : pings) {
        pos = std::find_if(pos, entries.end(), [&](const gsf_nav_entry& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });

        mbes_ping new_ping;
        new_ping.time_stamp_ = ping.time_stamp_;
        new_ping.time_string_ = ping.time_string_;
        if (pos == entries.end()) {
            new_ping.pos_ = entries.back().pos_;
            new_ping.heading_ = entries.back().yaw_;
            new_ping.pitch_ = entries.back().pitch_;
            new_ping.roll_ = entries.back().roll_;
        }
        else {
            new_ping.pos_ = pos->pos_;
            new_ping.heading_ = pos->yaw_;
            new_ping.pitch_ = pos->pitch_;
            new_ping.roll_ = pos->roll_;
        }

        for (int i = 0; i < ping.distances.size(); ++i) {
            double d = ping.distances[i];
            if (d < 0.1) {
                continue;
            }
            double th = M_PI/180.*ping.beam_angles[i];
            Eigen::Vector3d p = new_ping.pos_;
            Eigen::Vector3d beam(0., d*sin(th), -d*cos(th));
            Eigen::Matrix3d Rx = Eigen::AngleAxisd(new_ping.roll_, Eigen::Vector3d::UnitX()).matrix();
            Eigen::Matrix3d Ry = Eigen::AngleAxisd(new_ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
            Eigen::Matrix3d Rz = Eigen::AngleAxisd(new_ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
            Eigen::Matrix3d R = Rz*Ry*Rx;
            new_ping.beams.push_back(p + R*beam);
        }

        new_pings.push_back(new_ping);
    }

    return new_pings;
}
