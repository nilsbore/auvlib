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

#include <data_tools/navi_data.h>
#include <data_tools/colormap.h>
#include <data_tools/transforms.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <tuple>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <cereal/archives/json.hpp>

using namespace std;
using namespace Eigen;

namespace navi_data {

using namespace std_data;

void match_timestamps(mbes_ping::PingsT& pings, nav_entry::EntriesT& entries)
{

    std::stable_sort(pings.begin(), pings.end(), [](const mbes_ping& ping1, const mbes_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });
    std::sort(entries.begin(), entries.end(), [](const nav_entry& entry1, const nav_entry& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });

    // NOTE: this works, but is terribly slow
    /*
    for (mbes_ping& ping : pings) {
        auto entry = std::min_element(entries.begin(), entries.end(), [&](const nav_entry& entry1, const nav_entry& entry2) {
            return abs(entry1.time_stamp_ - ping.time_stamp_) < abs(entry2.time_stamp_ - ping.time_stamp_);
        });
        ping.pos_ = entry->pos_;
        //cout << "Ping " << ping.time_string_ << " closest to Entry " << entry->time_string_ << endl;
    }
    */

    auto pos = entries.begin();
    for (mbes_ping& ping : pings) {
        pos = std::find_if(pos, entries.end(), [&](const nav_entry& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });
        if (pos == entries.end()) {
            //cout << "Ping " << ping.time_string_ << " closest to LAST Entry " << entries.back().time_string_ << endl;
            ping.pos_ = entries.back().pos_;
        }
        else {
            ping.pos_ = pos->pos_;
            //cout << "Ping " << ping.time_string_ << " closest to Entry " << pos->time_string_ << endl;
        }
    }
}


void save_submaps_files(const std_data::mbes_ping::PingsT& pings, const boost::filesystem::path& folder){

    // Save submaps in external files
    string file_name;
    int sm_cnt = 0;
    std::ofstream myfile;
    for (const std_data::mbes_ping& ping_i: pings){
        // If new submap, open new file
        if(ping_i.first_in_file_){
            myfile.close();
            file_name = "/submap_" + std::to_string(sm_cnt) + ".xyz";
            myfile.open(folder.string() + file_name, std::ofstream::out);
            sm_cnt += 1;
        }
        if(myfile.is_open()){
            for(unsigned int i=0; i<ping_i.beams.size(); i++){
                myfile << ping_i.beams.at(i)(0) - 669000.0 << " " << ping_i.beams.at(i)(1) - 6383000.0 << " " << ping_i.beams.at(i)(2) << "\n";
            }
        }
    }
}


double compute_info_in_submap(std_data::mbes_ping::PingsT& submap_pings){

    // Beams z centroid
    double mean_beam = 0;
    int beam_cnt = 0;
    for (const std_data::mbes_ping& ping: submap_pings){
        for(const Eigen::Vector3d& beam_i: ping.beams){
            mean_beam += beam_i[2];
            ++beam_cnt;
        }
    }
    mean_beam = mean_beam / beam_cnt;

    // Condition number of z
    double cond_num = 0;
    for (const std_data::mbes_ping& ping: submap_pings){
        for(const Eigen::Vector3d& beam_i: ping.beams){
            cond_num += std::abs(beam_i[2] - mean_beam);
        }
    }
    return cond_num = cond_num / beam_cnt;
}

void divide_tracks_adaptive(mbes_ping::PingsT& pings)
{
    double info_thres = 0.3;
    // For every line (one line per file)
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });

        mbes_ping::PingsT::iterator first_pos_it = pos;
        Vector3d last_pos;
        double mean_width = 0.; double count = 0.;
        for (auto it = pos; it != next; ++it) {
            last_pos = it->pos_;
            mean_width += 1.7*(it->beams.front() - it->beams.back()).norm();
            count += 1;
        }
        mean_width /= count;
        double length = (last_pos - first_pos_it->pos_).norm();

        int nbr_submaps = int(length/mean_width+2.5);
        double min_submap_length = length / double(nbr_submaps);
        double ext_step = min_submap_length/4;
        double max_submap_length = 3*min_submap_length;

        cout << "Min submap length: " << min_submap_length << ", Growing step: "
             << ext_step << ", Max submap length: " << max_submap_length << endl;

        // For every ping in a line
        mbes_ping::PingsT::iterator latest_pos_it = first_pos_it;
        int steps = 0;
        int counter = 0; // TODO: remove!
        int last_submap_cnt = 0;
        for (auto it = pos; it != next; ++it) {
            // If submap too close to end of line
            if ((last_pos - it->pos_).norm() < min_submap_length) {
                cout << "Too close to end, breaking at " << counter << endl;
                break;
            }
            // If submap reaches max length
            if((latest_pos_it->pos_ - it->pos_).norm() > max_submap_length){
                cout << "Breaking up submap with max length at " << counter
                     << " out of " << std::distance(pos, next) << endl;
                it->first_in_file_ = true;
                latest_pos_it = it;
                steps = 0;
                last_submap_cnt = counter;
            }
            // If submap bigger than min length or min_length + extension * steps
            else if (((latest_pos_it->pos_ - it->pos_).norm() >= min_submap_length && steps == 0) ||
                    (latest_pos_it->pos_ - it->pos_).norm() >= min_submap_length + ext_step*steps) {

                // Compute information quantity on submap
                mbes_ping::PingsT pings_submap(pos+last_submap_cnt, it);
                double info_in_submap = compute_info_in_submap(pings_submap);
                cout << "Info in submap " << info_in_submap << endl;

                // If big, break here
                if(info_in_submap > info_thres){
                    cout << "Breaking up submap with enough info at " << counter
                         << " out of " << std::distance(pos, next) << endl;
                    it->first_in_file_ = true;
                    latest_pos_it = it;
                    steps = 0;
                    last_submap_cnt = counter;
                }
                // If small, keep extending submap
                else{
                    steps++;
                }

            }
            ++counter;
        }
        pos = next;
    }
}


void divide_tracks(mbes_ping::PingsT& pings)
{
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });

        Vector3d first_pos = pos->pos_;
        Vector3d last_pos;
        double mean_width = 0.; double count = 0.;
        for (auto it = pos; it != next; ++it) {
            last_pos = it->pos_;
            mean_width += 1.7*(it->beams.front() - it->beams.back()).norm();
            count += 1;
        }
        mean_width /= count;
        double length = (last_pos - first_pos).norm();

        int nbr_submaps = int(length/mean_width+0.5);
        double submap_length = length / double(nbr_submaps);

        cout << "Mean width: " << mean_width << ", Length: " << length << ",  Nbr submaps: " << nbr_submaps << ", Submap length: " << submap_length << endl;

        Vector3d recent_pos = first_pos;
        int last_submap_cnt = 0;
        int counter = 0; // TODO: remove!
        for (auto it = pos; it != next; ++it) {
            if ((last_pos - it->pos_).norm() < submap_length/2.) {
                cout << "Too close to end, breaking at " << counter << endl;
                break;
            }
            if ((it->pos_ - recent_pos).norm() > submap_length) {
                cout << "Breaking up submap at " << counter << " out of " << std::distance(pos, next) << endl;
                it->first_in_file_ = true;
                recent_pos = it->pos_;
//                mbes_ping::PingsT pings_submap(pos+last_submap_cnt, it);
//                double info_in_submap = computeInfoInSubmap(pings_submap);
//                cout << "Info in submap " << info_in_submap << endl;
                last_submap_cnt = counter;
            }
            ++counter;
        }

        pos = next;
    }
}

void divide_tracks_equal(mbes_ping::PingsT& pings)
{
    Vector2d point1, point2, dir; // first and last point on line
    vector<bool> line_positive_directions;
    double first_line_pos = -1000000;
    double last_line_pos = 1000000;

    cout << "Really First first in file?: " << pings[0].first_in_file_ << endl;

    double mean_width = 0.; double count = 0.;
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });

        Vector2d first_pos = pos->pos_.head<2>();
        Vector2d last_pos;
        for (auto it = pos; it != next; ++it) {
            last_pos = it->pos_.head<2>();
            mean_width += 1.7*(it->beams.front() - it->beams.back()).norm();
            count += 1;
        }
        if (pos == pings.begin()) {
            point1 = first_pos;
            point2 = last_pos;
            dir = point2-point1;
            dir.normalize();
        }

        bool positive_direction = dir.dot(last_pos - first_pos) > 0;
        line_positive_directions.push_back(positive_direction);

        double line_pos1 = dir.dot(first_pos - point1);
        double line_pos2 = dir.dot(last_pos - point1);
        cout << "Number pings: " << std::distance(pos, next) << endl;
        cout << "First == last? " << (pos == next) << endl;
        cout << "First beams: " << pos->beams.size() << " Next beams: " << next->beams.size() << endl;
        cout << "First firs in file?: " << pos->first_in_file_ << ", Next first in file?: " << next->first_in_file_ << endl;
        cout << "First time: " << pos->time_stamp_ << ", Next time: " << next->time_stamp_ << endl;
        cout << "First pos: " << first_pos.transpose() << endl;
        cout << "Last pos: " << last_pos.transpose() << ", Point 1: " << point1.transpose() << "Dir: " << dir.transpose() << endl;
        cout << "Line pos 1: " << line_pos1 << ", Line pos 2: " << line_pos2 << endl;
        if (!positive_direction) {
            std::swap(line_pos1, line_pos2);
        }

        first_line_pos = std::max(first_line_pos, line_pos1);
        last_line_pos = std::min(last_line_pos, line_pos2);

        pos = next;
    }

    mean_width /= count;
    //double length = (point2 - point1).norm();
    double line_pos_length = last_line_pos - first_line_pos;
    int nbr_submaps = int(line_pos_length/mean_width+0.5);
    double submap_length = line_pos_length / double(nbr_submaps);

    cout << "First line pos: " << first_line_pos << ", last line pos: " << last_line_pos << endl;
    cout << "Mean width: " << mean_width << ", Length: " << line_pos_length << ",  Nbr submaps: " << nbr_submaps << ", Submap length: " << submap_length << endl;
    cout << "------------" << endl;
    int track_counter = 0;
    int min_since_last = 5;
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });

        bool positive_direction = line_positive_directions[track_counter];
        //Vector3d recent_pos = pos->pos_;
        //int counter;
        double recent_line_pos = positive_direction?
                    dir.dot(pos->pos_.head<2>() - point1) - first_line_pos:
                    last_line_pos - dir.dot(pos->pos_.head<2>() - point1);
        int time_since_last = 0;
        for (auto it = pos; it != next; ++it) {
            if (std::distance(it, next) < min_since_last) {
                break;
            }
            double line_pos = positive_direction?
                        dir.dot(it->pos_.head<2>() - point1) - first_line_pos :
                        last_line_pos - dir.dot(it->pos_.head<2>() - point1);
            if (line_pos > 0 && line_pos < line_pos_length) {
                if ((recent_line_pos < 0 || recent_line_pos > line_pos_length) &&
                        time_since_last > min_since_last) {
                    it->first_in_file_ = true;
                    time_since_last = 0;
                }
                else if ((int(recent_line_pos/submap_length) < int(line_pos/submap_length)) &&
                        time_since_last > min_since_last) {
                    it->first_in_file_ = true;
                    time_since_last = 0;
                }
            }
            else if ((recent_line_pos > 0 && recent_line_pos < line_pos_length) &&
                    time_since_last > min_since_last) {
                it->first_in_file_ = true;
                time_since_last = 0;
            }
            //++counter;
            recent_line_pos = line_pos;
            ++time_since_last;
        }

        pos = next;
        ++track_counter;
    }
}

tuple<ObsT, TransT, AngsT, MatchesT, BBsT, ObsT> create_submaps(const mbes_ping::PingsT& pings)
{
    ObsT submaps;
    TransT trans;
    AngsT angs;
    MatchesT matches;
    BBsT bounds;
    ObsT tracks;
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        mbes_ping::PingsT track_pings;
        track_pings.insert(track_pings.end(), pos, next);
        cout << "found 1 pos!" << endl;

        if (pos == next) {
            break;
        }

        MatrixXd points(track_pings.size()*track_pings[0].beams.size(), 3);
        MatrixXd track(track_pings.size(), 3);

        // get the direction of the submap as the mean direction
        Vector3d dir = track_pings.back().pos_ - track_pings.front().pos_;
        Vector3d ang; ang << 0., 0., std::atan2(dir(1), dir(0));
        Eigen::Matrix3d RM = data_transforms::euler_to_matrix(ang(0), ang(1), ang(2));

        int counter = 0;
        int ping_counter = 0;
        for (const mbes_ping& ping : track_pings) {
            //cout << "Counter : " << counter << " and size: " << points.rows() << " and new points: " << ping.beams.size() << endl;
            if (counter + ping.beams.size() > points.rows()) {
                points.conservativeResize(counter + ping.beams.size(), 3);
            }
            for (const Vector3d& p : ping.beams) {
                points.row(counter) = p;
                ++counter;
            }
            track.row(ping_counter) = ping.pos_;
            ++ping_counter;
        }
        if (counter == 0) {
            pos = next;
            continue;
        }
        points.conservativeResize(counter, 3);
        trans.push_back(points.colwise().mean().transpose());
        angs.push_back(ang);
        points.array().rowwise() -= trans.back().transpose().array();
        points = points*RM;
        track.array().rowwise() -= trans.back().transpose().array();
        track = track*RM;
        Matrix2d bb;
        bb(0, 0) = points.col(0).minCoeff();
        bb(0, 1) = points.col(1).minCoeff();
        bb(1, 0) = points.col(0).maxCoeff();
        bb(1, 1) = points.col(1).maxCoeff();
        submaps.push_back(points);
        bounds.push_back(bb);
        tracks.push_back(track);

        pos = next;
    }

    // homogenize angles
    /*
    for (int i = 0; i < submaps.size(); ++i) {
        if (fabs(angs[i](2)) > M_PI/2.) {
            submaps[i].leftCols(2).array() *= -1.; // rotate 180 degrees
            tracks[i].leftCols(2).array() *= -1.; // rotate 180 degrees
            Matrix2d bb = bounds[i];
            bounds[i].row(0) = -bb.row(1);
            bounds[i].row(1) = -bb.row(0);
            if (angs[i](2) < -M_PI/2.) {
                angs[i](2) += M_PI;
            }
            else {
                angs[i](2) -= M_PI;
            }
        }
    }
    */

    return make_tuple(submaps, trans, angs, matches, bounds, tracks);
}


} // namespace navi_data

namespace std_data {

// Extract space-separated numbers: Nav files
// 0: Year
// 1: Time (day, hour)
// 2: Second
// 3: Ping num
// 4: Beam num
// 5: X
// 6: Y
// 7: Z
// 8: Tide
// 9: Heading
// 10: Heave
// 11: Pitch
// 12: Roll
template <>
mbes_ping::PingsT parse_file(const boost::filesystem::path& file)
{
    mbes_ping::PingsT pings;

    string line;
    std::ifstream infile(file.string());

    mbes_ping ping;
    string time;
    int beam_id;
    double tide, x, y, z;
    string year_string, date_string, second_string;
    const std::locale loc = std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("%Y %m%d%H%M %S%f"));
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");

    std::getline(infile, line); // throw away first line as it contains description
    int counter = 0;
    while (std::getline(infile, line))  // this does the checking!
    {
        istringstream iss(line);

        iss >> year_string >> date_string >> second_string >> ping.id_ >> beam_id >> x >> y >> z >> tide >> ping.heading_ >> ping.heave_ >> ping.pitch_ >> ping.roll_;

        /*
        if (beam_id != 255 && counter % 10 != 0) {
            ++counter;
            continue;
        }
        */

        ping.beams.push_back(Vector3d(x, y, -z));

        if (beam_id == 255) {
            ping.first_in_file_ = pings.empty();
            std::istringstream is(year_string + " " + date_string + " " + second_string);
            is.imbue(loc);
            boost::posix_time::ptime t;
            is >> t;
            boost::posix_time::time_duration const diff = t - epoch;
            ping.time_stamp_ = diff.total_milliseconds();
            stringstream time_ss;
            time_ss << t;
            ping.time_string_ = time_ss.str();
            //cout << year_string << " " << date_string << " " << second_string << endl;
            //cout << t << endl;

            pings.push_back(ping);
            ping.beams.resize(0);
        }

        ++counter;
    }

    return pings;
}

// Extract space-separated numbers: Nav files
// 0: Day
// 1: Time
// 2: Easting
// 3: Northing
// 4: Depth (given in positive values!)
// 5: Zeros
template <>
nav_entry::EntriesT parse_file(const boost::filesystem::path& file)
{
    nav_entry::EntriesT entries;

    nav_entry entry;
    double x, y, z;
    string date_string, time_string;
    const std::locale loc = std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("%Y.%m.%d %H:%M:%S%f"));
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");

    string line;
    std::ifstream infile(file.string());
    while (std::getline(infile, line))  // this does the checking!
    {
        istringstream iss(line);

        //iss >> entry.year_ >> entry.time_stamp_ >> x >> y >> z;
        iss >> date_string >> time_string >> x >> y >> z;
        entry.pos_ = Vector3d(x, y, -z);

        std::istringstream is(date_string + " " + time_string);
        is.imbue(loc);
        boost::posix_time::ptime t;
        is >> t;
        boost::posix_time::time_duration const diff = t - epoch;
        entry.time_stamp_ = diff.total_milliseconds();
        stringstream time_ss;
        time_ss << t;
        entry.time_string_ = time_ss.str();

        //cout << date_string << " " << time_string << endl;
        //cout << t << endl;
        //cout << ms << endl;
        entry.first_in_file_ = entries.empty();

        entries.push_back(entry);
    }

    return entries;
}
} // namespace std_data
