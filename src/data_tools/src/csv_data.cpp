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

#include <data_tools/csv_data.h>
//#include <data_tools/xtf_data.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <ctype.h>

using namespace std;

template<typename T, typename Char, typename Traits>
inline std::basic_istream<Char, Traits>& skip(std::basic_istream<Char, Traits>& stream) {
    T unused;
    return stream >> unused;
}

namespace csv_data {

using namespace std_data;

std_data::sss_ping::PingsT convert_matched_entries_pitch(std_data::sss_ping::PingsT& pings, csv_nav_entry::EntriesT& entries)
{
    std_data::sss_ping::PingsT new_pings;

    std::stable_sort(entries.begin(), entries.end(), [](const csv_nav_entry& entry1, const csv_nav_entry& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });

    auto pos = entries.begin();
    for (std_data::sss_ping& ping : pings) {
        pos = std::find_if(pos, entries.end(), [&](const csv_nav_entry& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });

        std_data::sss_ping new_ping = ping;
        if (pos == entries.end()) {
            new_ping.pitch_ = entries.back().pitch_;
        }
        else {
            if (pos == entries.begin()) {
                new_ping.pitch_ = pos->pitch_;
            }
            else {
                csv_nav_entry& previous = *(pos - 1);
                double ratio = double(ping.time_stamp_ - previous.time_stamp_)/double(pos->time_stamp_ - previous.time_stamp_);
                new_ping.pitch_ = previous.pitch_ + ratio*(pos->pitch_ - previous.pitch_);
            }
        }

        new_pings.push_back(new_ping);
    }

    return new_pings;
}

std_data::sss_ping::PingsT convert_matched_entries(std_data::sss_ping::PingsT& pings, csv_nav_entry::EntriesT& entries)
{
    std_data::sss_ping::PingsT new_pings;

    std::stable_sort(entries.begin(), entries.end(), [](const csv_nav_entry& entry1, const csv_nav_entry& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });

    std::stable_sort(pings.begin(), pings.end(), [](const std_data::sss_ping& ping1, const std_data::sss_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });

    auto pos = entries.begin();
    int bcount = 0;
    int ecount = 0;
    int mcount = 0;
    for (std_data::sss_ping& ping : pings) {
        pos = std::find_if(pos, entries.end(), [&](const csv_nav_entry& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });

        std_data::sss_ping new_ping = ping;
        if (pos == entries.end()) {
            new_ping.roll_ = entries.back().roll_;
            new_ping.pitch_ = entries.back().pitch_;
            //new_ping.heading_ = entries.back().heading_;
            new_ping.pos_ = entries.back().pos_;
            ++ecount;
        }
        else {
            if (pos == entries.begin()) {
                new_ping.roll_ = pos->roll_;
                new_ping.pitch_ = pos->pitch_;
                //new_ping.heading_ = pos->heading_;
                new_ping.pos_ = pos->pos_;
                ++bcount;
            }
            else {
                csv_nav_entry& previous = *(pos - 1);
                double ratio = double(ping.time_stamp_ - previous.time_stamp_)/double(pos->time_stamp_ - previous.time_stamp_);
                new_ping.roll_ = previous.roll_ + ratio*(pos->roll_ - previous.roll_);
                new_ping.pitch_ = previous.pitch_ + ratio*(pos->pitch_ - previous.pitch_);
                //new_ping.heading_ = previous.heading_ + ratio*(pos->heading_ - previous.heading_);
                new_ping.pos_ = previous.pos_ + ratio*(pos->pos_ - previous.pos_);
                ++mcount;
            }
        }
        
        //Eigen::Matrix3d Rz = Eigen::AngleAxisd(new_ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
        // these are my estimated values for the
        // sidescan offset from the center of motion
        //new_ping.pos_.array() += (2.*Rz.col(0) + -1.5*Rz.col(1)).array();
        new_ping.pos_[2] = ping.pos_[2];

        new_pings.push_back(new_ping);
    }
    cout << "Got " << bcount << " at beginning, " << ecount << " at end and " << mcount << " in the middle" << endl;

    return new_pings;
}

} // namespace csv_data

namespace std_data {

using namespace csv_data;

/*
**************************************************************************************************************************
 POS Export Utility                                    
 Copyright (C) 1997-2018 by Applanix Corporation      [May 17 2018] 
 All rights reserved.                                        
**************************************************************************************************************************
 
 Parameter setup:
 POSPROC SBET file: E:\KTH\3-Process\POSPac\KTH_20180807_0638-1508\Mission 1\Proc\sbet_Mission 1.out 
 Camera mid-exposure event file: E:\KTH\3-Process\POSPac\KTH_20180807_0638-1508\Mission 1\Extract\event1_Mission 1.dat 
   Event time shift: 0.000000 sec 
 Photo ID file:  
   Photo ID file format: 2 Fields (Time, Photo ID) Format 
   Offset between PHOTO ID and EVENT file times: 0.000000 sec 
   PHOTO ID time tolerance: 0.300000 sec 
 Mission Start Time:
   Date of Mission: 2018-08-07
   Start Time: 06:38:26
 Mapping frame epoch: 2018.597260
 Mapping frame datum: WGS84  ; Mapping frame projection : TM;
 central meridian = 9.000000 deg;
 latitude of the grid origin = 0.000000 deg; grid scale factor = 0.999600: 
 false easting = 500000.000000 m; false northing = 0.000000 m; 
 Boresight values: tx =        0.0000 arc min, ty =        0.0000 arc min, tz =        0.0000 arc min. 
 Lever arm values: lx =        0.0000 m, ly =        0.0000 m, lz =        0.0000 m. 
 TIME, DISTANCE, EASTING, NORTHING, ELLIPSOID HEIGHT, LATITUDE, LONGITUDE, ELLIPSOID HEIGHT, ROLL, PITCH, HEADING, EAST VELOCITY, NORTH VELOCITY, UP VELOCITY, EAST SD, NORTH SD, HEIGHT SD, ROLL SD, PITCH SD, HEADING SD

  (time in Sec, distance in Meters, position in Meters, lat, long in Degrees, orientation angles and SD in Degrees, velocity in Meter/Sec, position SD in Meters)  
*/

template <>
csv_nav_entry::EntriesT parse_file(const boost::filesystem::path& file)
{
    csv_nav_entry::EntriesT entries;

    csv_nav_entry entry;
    double distance, x, y, z, x_std, y_std, z_std, vx, vy, vz;
    double roll, pitch, heading, roll_std, pitch_std, heading_std;
    double time_seconds;
    // NOTE: this is the Sunday before the survey, i.e. the start of the GPS week time
    const boost::posix_time::ptime gps_epoch = boost::posix_time::time_from_string("2018-08-05 00:00:00.000");
    const boost::posix_time::ptime unix_epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
	
    string line;
    std::ifstream infile(file.string());
    while (std::getline(infile, line))  // this does the checking!
    {
        line.erase(0, line.find_first_not_of(" ")); // remove initial space if there
        if (line.empty() || line[0] == '\n' || !isdigit(line[0])) {
            continue;
        }
        istringstream iss(line);

		iss >> time_seconds >> distance >> x >> y >> z >> entry.lat_ >> entry.long_ >> entry.altitude >>
               roll >> pitch >> heading >> vx >> vy >> vz >> x_std >> y_std >> z_std >> roll_std >> pitch_std >> heading_std;
        entry.pos_ = Eigen::Vector3d(x, y, z);
        entry.vel_ = Eigen::Vector3d(vx, vy, vz);
        entry.heading_ = M_PI/180.*heading;
        entry.heading_ = 0.5*M_PI-entry.heading_; // TODO: need to keep this for old data. basically heading -> yaw
        entry.pitch_ = M_PI/180.*pitch;
        entry.roll_ = M_PI/180.*roll;
        entry.heading_std_ = M_PI/180.*heading_std;
        entry.pitch_std_ = M_PI/180.*pitch_std;
        entry.roll_std_ = M_PI/180.*roll_std;

        entry.time_stamp_ = (long long)(1000. * time_seconds); // double seconds to milliseconds

        boost::posix_time::ptime t = gps_epoch + boost::posix_time::milliseconds(entry.time_stamp_);
        boost::posix_time::time_duration duration = t - unix_epoch;
        entry.time_stamp_ = (long long)(duration.total_milliseconds());

        stringstream time_ss;
        time_ss << t;
        entry.time_string_ = time_ss.str();

		entries.push_back(entry);
    }

	return entries;
}

template <>
csv_asvp_sound_speed::EntriesT parse_file(const boost::filesystem::path& file)
{
    csv_asvp_sound_speed entry;

    // NOTE: this is the Sunday before the survey, i.e. the start of the GPS week time
    const boost::posix_time::ptime unix_epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
	
    string line;
    std::ifstream infile(file.string());

    if (!std::getline(infile, line)) {
        cout << "File " << file << " did not contain any header!" << endl;
        exit(-1);
    }
    else {
        istringstream iss(line.substr(1, line.size()-1));


        cout << "ISS: " << iss.str() << endl;

        string date_string;
		iss >> skip<string> >> skip<double> >> skip<double> >> date_string >> entry.lat_ >> entry.long_;

        cout << "Date string: " << date_string << endl;
    
        const std::locale loc = std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("%Y%m%d%H%M%S"));

        std::istringstream is(date_string);
        is.imbue(loc);
        boost::posix_time::ptime t;
        is >> t;
        boost::posix_time::time_duration const diff = t - unix_epoch;
        entry.time_stamp_ = diff.total_milliseconds();
        stringstream time_ss;
        time_ss << t;
        cout << "Time string: " << time_ss.str() << endl;
        entry.time_string_ = time_ss.str();
    }

    double dbar, vel;
    entry.dbars.resize(100);
    entry.vels.resize(100);
    int counter = 0;
    while (std::getline(infile, line))  // this does the checking!
    {
        line.erase(0, line.find_first_not_of(" ")); // remove initial space if there
        if (line.empty() || line[0] == '\n' || !isdigit(line[0])) {
            continue;
        }
        istringstream iss(line);
		iss >> dbar >> vel;

        if (counter >= entry.dbars.rows()) {
            entry.dbars.conservativeResize(counter+1000);
            entry.vels.conservativeResize(counter+1000);
        }
        entry.dbars(counter) = dbar;
        entry.vels(counter) = vel;
        ++counter;
    }
    entry.dbars.conservativeResize(counter);
    entry.vels.conservativeResize(counter);

	return { entry };
}

} // namespace std_data
