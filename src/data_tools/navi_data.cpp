#include <data_tools/navi_data.h>
#include <data_tools/colormap.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <tuple>

#include <boost/date_time.hpp>

#include <cereal/archives/json.hpp>

using namespace std;
using namespace Eigen;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

void match_timestamps(vector<mbes_ping>& pings, vector<nav_entry>& entries)
{

    std::sort(pings.begin(), pings.end(), [](const mbes_ping& ping1, const mbes_ping& ping2) {
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

void view_cloud(const vector<mbes_ping>& pings)
{
    CloudT::Ptr cloud(new CloudT);
	Array3d mean(0., 0., 0.);
	double count = 0.;
	for (const mbes_ping& ping : pings) {
		cereal::JSONOutputArchive ar(std::cout);
		ar(ping);
		for (const Vector3d& p : ping.beams) {
			mean += p.array(); count += 1.;
		}
    }
	mean /= count;
	for (const mbes_ping& ping : pings) {
		for (const Vector3d& p : ping.beams) {
		    PointT point; point.r = 255;
			point.getVector3fMap() = (p-mean.matrix()).cast<float>();
		    cloud->push_back(point);
		}
    }
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {

    }
}

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
vector<mbes_ping> read_file(const boost::filesystem::path& file)
{
    vector<mbes_ping> pings;

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

        if (beam_id != 255 && counter % 10 != 0) {
            ++counter;
            continue;
        }

		ping.beams.push_back(Vector3d(x, y, -z));

		if (beam_id == 255) {
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

            ping.first_in_file_ = pings.empty();
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
vector<nav_entry> read_file(const boost::filesystem::path& file)
{
    vector<nav_entry> entries;

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

void divide_tracks(vector<mbes_ping>& pings)
{
    for (auto pos = pings.begin(); ; pos != pings.end()) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });

        if (pos == next) {
            break;
        }

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
        int counter = 0;
        for (auto it = pos; it != next; ++it) {
            if ((last_pos - it->pos_).norm() < submap_length/2.) {
                cout << "Too close to end, breaking at " << counter << endl;
                break;
            }
            if ((it->pos_ - recent_pos).norm() > submap_length) {
                cout << "Breaking up submap at " << counter << " out of " << std::distance(pos, next) << endl;
                it->first_in_file_ = true;
                recent_pos = it->pos_;
            }
            ++counter;
        }

        pos = next;
    }
}

tuple<ObsT, TransT> create_submaps(const vector<mbes_ping>& pings)
{
    ObsT submaps;
    TransT trans;
    for (auto pos = pings.begin(); ; pos != pings.end()) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        vector<mbes_ping> track_pings;
        track_pings.insert(track_pings.end(), pos, next);
        cout << "found 1 pos!" << endl;

        if (pos == next) {
            break;
        }

        MatrixXd points(track_pings.size()*track_pings[0].beams.size(), 3);
        int counter = 0;
        for (const mbes_ping& ping : track_pings) {
            //cout << "Counter : " << counter << " and size: " << points.rows() << " and new points: " << ping.beams.size() << endl;
            if (counter + ping.beams.size() > points.rows()) {
                points.conservativeResize(counter + ping.beams.size(), 3);
            }
            for (const Vector3d& p : ping.beams) {
                points.row(counter) = p;
                ++counter;
            }
        }
        points.conservativeResize(counter, 3);
        trans.push_back(points.colwise().mean().transpose());
        points.array().rowwise() -= trans.back().transpose().array();
        submaps.push_back(points);

        pos = next;
    }
    return make_tuple(submaps, trans);
}

void visualize_submaps(ObsT& submaps, TransT& trans) {

	CloudT::Ptr cloud(new CloudT);

    for (int i = 0; i < submaps.size(); ++i) {
        Vector3f t = trans[i].cast<float>() - trans[0].cast<float>();
        for (int j = 0; j < submaps[i].rows(); ++j) {
            PointT p;
            p.getVector3fMap() = submaps[i].row(j).cast<float>() + t.transpose();
            p.r = colormap[i%43][0];
            p.g = colormap[i%43][1];
            p.b = colormap[i%43][2];
            cloud->push_back(p);
        }
    }

	cout << "Done constructing point cloud, starting viewer..." << endl;

	//... populate cloud
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}

