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

#include <data_tools/all_data.h>
#include <data_tools/lat_long_utm.h>
#include <liball/all.h>
//#include <endian.h>
#include <fstream>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/algorithm/string.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace all_data {

using namespace std_data;

std::tuple<uint8_t, uint8_t, uint8_t> jet(double x)
{
    const double rone = 0.8;
    const double gone = 1.0;
    const double bone = 1.0;
    double r, g, b;

    x = (x < 0 ? 0 : (x > 1 ? 1 : x));

    if (x < 1. / 8.) {
        r = 0;
        g = 0;
        b = bone * (0.5 + (x) / (1. / 8.) * 0.5);
    } else if (x < 3. / 8.) {
        r = 0;
        g = gone * (x - 1. / 8.) / (3. / 8. - 1. / 8.);
        b = bone;
    } else if (x < 5. / 8.) {
        r = rone * (x - 3. / 8.) / (5. / 8. - 3. / 8.);
        g = gone;
        b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
    } else if (x < 7. / 8.) {
        r = rone;
        g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
        b = 0;
    } else {
        r = (rone - (x - 7. / 8.) / (1. - 7. / 8.) * 0.5);
        g = 0;
        b = 0;
    }

    return std::make_tuple(uint8_t(255.*r), uint8_t(255.*g), uint8_t(255.*b));
}

cv::Mat make_waterfall_image(const vector<vector<all_xyz88_datagram_repeat> >& pings)
{
    int rows = pings.size();
    int cols = pings[0].size();
    cv::Mat swath_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
	double dmin = 6000.;
	double dmax = 0.;
    for (int i = 0; i < pings.size(); ++i) {
        for (int j = 0; j < pings[i].size(); ++j) {
		    dmin = std::min(dmin, double(pings[i][j].depth));
		    dmax = std::max(dmax, double(pings[i][j].depth));
		}
	}
    for (int i = 0; i < pings.size(); ++i) {
        for (int j = 0; j < pings[i].size(); ++j) {
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, j);
			double scaled = (pings[i][j].depth - dmin)/dmax;
            tie(p->z, p->y, p->x) = jet(scaled);
        }
    }
    //cv::Mat resized_swath_img;//dst image
    //cv::resize(swath_img, resized_swath_img, cv::Size(rows/8, cols/8));//resize image
    
    return swath_img; // resized_swath_img;
}

/*
template <typename T>
T read_datagram()
{
    T datagram;
	return datagram;
}

template <>
all_xyz88_datagram read_datagram<all_xyz88_datagram, 88>()
{
    T datagram;
	return datagram;
}

*/

/*
template <typename ReturnType>
vector<ReturnType, Eigen::aligned_allocator<ReturnType> > parse_file(const boost::filesystem::path& path)
{
    vector<ReturnType, Eigen::aligned_allocator<ReturnType> > rtn;
	return rtn;
}
*/

template <typename ReturnType, typename AllHeaderType>
ReturnType read_datagram(std::istream& input, const AllHeaderType& header)
{
    ReturnType rtn;
	return rtn;
}

template <typename ReturnType, typename AllHeaderType, int Code>
vector<ReturnType, Eigen::aligned_allocator<ReturnType> > parse_stream_impl(istream& input)
{
	vector<ReturnType, Eigen::aligned_allocator<ReturnType> > returns;

    unsigned int nbr_bytes;
    unsigned char start_id;
    unsigned char data_type;

    // end codes that we need to read
	//unsigned char spare; // Spare (always 0)
	unsigned char end_ident; // End identifier = ETX (Always 03h)
	unsigned short checksum; // Check sum of data between STX and ETX

	int pos_counter = 0;
    int counters[255];
    for (int i = 0; i < 255; ++i) {
        counters[i] = 0;
    }
	while (!input.eof()) {
		//cout << "Trying to read nbr_bytes with size " << sizeof(nbr_bytes) << endl;
		input.read(reinterpret_cast<char*>(&nbr_bytes), sizeof(nbr_bytes));
		//cout << "Number bytes has little endian: " << nbr_bytes << endl;
		//cout << "Trying to read start_id with size " << sizeof(start_id) << endl;
		input.read(reinterpret_cast<char*>(&start_id), sizeof(start_id));
		//cout << "Trying to read data_type with size " << sizeof(data_type) << endl;
		input.read(reinterpret_cast<char*>(&data_type), sizeof(data_type));
		//cout << "Number bytes: " << nbr_bytes << endl;
		//cout << "Start id: " << int(start_id) << endl;
		//cout << "Data type must be " << Code << endl;
		if (data_type == 80) {
		    ++pos_counter;
		}
        counters[data_type] += 1;
	    if (data_type == Code) {
			AllHeaderType header;
		    //cout << "Is a MB reading, code: " << int(data_type) << endl;
		    input.read(reinterpret_cast<char*>(&header), sizeof(header));
			returns.push_back(read_datagram<ReturnType, AllHeaderType>(input, header));
		    //input.read(reinterpret_cast<char*>(&spare), sizeof(spare));
		    input.read(reinterpret_cast<char*>(&end_ident), sizeof(end_ident));
		    input.read(reinterpret_cast<char*>(&checksum), sizeof(checksum));
			//cout << "End identifier: " << end_ident << endl;
            returns.back().first_in_file_ = false;
		}
		else {
		    //cout << "No MB reading, code: " << int(data_type) << endl;
            int skip_bytes = nbr_bytes-sizeof(start_id)-sizeof(data_type);
            input.ignore(skip_bytes);
		}
	}

    /*
    for (int i = 0; i < 255; ++i) {
        cout << std::dec << "Got " << counters[i] << " of type " << i << std::hex <<" with hex 0x"<< i << std::dec << endl;
    }
	cout << "Got " << pos_counter << " position entries" << endl;
    */

    if (!returns.empty()) {
        returns[0].first_in_file_ = true;
    }

	return returns;
}

template <typename ReturnType, typename AllHeaderType, int Code>
vector<ReturnType, Eigen::aligned_allocator<ReturnType> > parse_file_impl(const boost::filesystem::path& path)
{
    if (boost::filesystem::extension(path) != ".all") {
        cout << "Not an .all file, skipping..." << endl;
        return vector<ReturnType, Eigen::aligned_allocator<ReturnType> >(0);
    }

    std::ifstream input;
	input.open(path.string(), std::ios::binary);
	if (input.fail()) {
        cout << "ERROR: Cannot open the file..." << endl;
        exit(0);
    }
    //cout << "Opened " << path << " for reading..." << endl;
    
    return parse_stream_impl<ReturnType, AllHeaderType, Code>(input);
}

pair<long long, string> parse_all_time(unsigned int date, unsigned int time)
{
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    const std::locale loc = std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("YYYYmmdd"));

    boost::posix_time::time_duration time_d = boost::posix_time::milliseconds(time);
    //std::istringstream is(to_string(date));
    //cout << "Date: " << to_string(date) << endl;
    //is.imbue(loc);
    string date_string = to_string(date);
    /*if (date_string.size() != 8) {
        return make_pair(0, "1970-01-01 00:00:00.000");
    }*/
    boost::gregorian::date date_t = boost::gregorian::date_from_iso_string(date_string);
    boost::posix_time::ptime t(date_t, time_d);
    //is >> date_t;
    long long time_stamp_ = (t - epoch).total_milliseconds();
    stringstream time_ss;
    time_ss << t;
    string time_string_ = time_ss.str();
    //cout << "Time string: " << time_string_ << endl;

    return make_pair(time_stamp_, time_string_);
}

template <>
all_mbes_ping read_datagram<all_mbes_ping, all_xyz88_datagram>(std::istream& input, const all_xyz88_datagram& header)
{
	//cout << "Total number of beams: " << header.nbr_beams << endl;
    //cout << "Valid number of beams: " << header.nbr_valid << endl;
	all_mbes_ping new_ping;
	new_ping.id_ = header.ping_count;
	//new_ping.heading_ = header.heading;
    new_ping.heading_ = M_PI/180.*double(header.heading)*0.01;
    new_ping.heading_ = 0.5*M_PI-new_ping.heading_; // this basically converts to yaw, should have that as marker instead
	new_ping.sound_vel_ = header.sound_vel;
	new_ping.transducer_depth_ = header.transducer_depth;
    tie(new_ping.time_stamp_, new_ping.time_string_) = parse_all_time(header.date, header.time);
	vector<all_xyz88_datagram_repeat> pings;
	all_xyz88_datagram_repeat ping;
    static unsigned char mask[] = {128, 64, 32, 16, 8, 4, 2, 1};
    int nbr_valid = 0;
	for (int i = 0; i < header.nbr_beams; ++i) {
		input.read(reinterpret_cast<char*>(&ping), sizeof(ping));
        //if (short(le16toh(ping.rt_cleaning_info)) < 0) { // (ping.detection_info & mask[5]) != 0) {
        if (ping.rt_cleaning_info < 0 || (ping.detection_info & mask[0]) != 0 || ping.depth > 40.) {
            continue;
        }
		pings.push_back(ping);
		Eigen::Vector3d pos(ping.along_track, -ping.across_track, -ping.depth);
		new_ping.beams.push_back(pos);
		new_ping.reflectivities.push_back(ping.reflectivity);
        ++nbr_valid;
	}
    //cout << "Computed nbr valid: " << nbr_valid << endl;

	unsigned char spare; // Spare (always 0)
	input.read(reinterpret_cast<char*>(&spare), sizeof(spare));

	return new_ping;
}

template <>
all_nav_entry read_datagram<all_nav_entry, all_position_datagram>(std::istream& input, const all_position_datagram& header)
{
	//cout << "Got a position datagram, skipping: " << int(header.nbr_bytes_input) << endl;

    //input.ignore(header.nbr_bytes_input);
    char buffer[255];
    input.read(buffer, header.nbr_bytes_input);
    //cout << "Got buffer: " << buffer << endl;
    vector<string> strs;
    boost::split(strs, buffer, boost::is_any_of(","));
    //cout << "Str number 6: " << strs[6] << endl;
	all_nav_entry entry;
	entry.id_ = header.pos_count;
	entry.lat_ = double(header.latitude)/20000000.;
	entry.long_ = double(header.longitude)/10000000.;
    entry.depth_ = stof(strs[6]);
	entry.heading_ = M_PI/180.*double(header.heading)*0.01;
    entry.heading_ = 0.5*M_PI-entry.heading_; // this basically converts to yaw
	entry.course_over_ground_ = double(header.course_over_ground)*0.01;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date, header.time);

	unsigned char spare; // Spare (always 0)
	input.read(reinterpret_cast<char*>(&spare), sizeof(spare));

	return entry;
}

template <>
all_nav_depth read_datagram<all_nav_depth, all_depth_datagram>(std::istream& input, const all_depth_datagram& header)
{
	all_nav_depth entry;
	entry.id_ = header.height_count;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date, header.time);
    
    entry.height = 100.*double(header.height); // Height in cm
    entry.height_type = header.height_type; // Height type
	
    return entry;
}

template <>
all_nav_attitude read_datagram<all_nav_attitude, all_attitude_datagram>(std::istream& input, const all_attitude_datagram& header)
{
	all_nav_attitude entry;
	entry.id_ = header.attitude_count;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date, header.time);

    all_nav_attitude_sample sample;
	all_attitude_datagram_repeat meas;

    entry.samples.reserve(header.nbr_entries);
	for (int i = 0; i < header.nbr_entries; ++i) {
		input.read(reinterpret_cast<char*>(&meas), sizeof(meas));
        sample.ms_since_start = meas.ms_since_start;
        sample.pitch = M_PI/180.*0.01*double(meas.pitch);
        sample.roll = M_PI/180.*0.01*double(meas.roll);
        sample.heading = M_PI/180.*0.01*double(meas.heading);
        sample.heave = 0.01*double(meas.heave);
        entry.samples.push_back(sample);
	}
    
	unsigned char system_desc; // Sensor system descriptor 1U
	input.read(reinterpret_cast<char*>(&system_desc), sizeof(system_desc));
	
    return entry;
}

template <>
all_echosounder_depth read_datagram<all_echosounder_depth, all_echosounder_depth_datagram>(std::istream& input, const all_echosounder_depth_datagram& header)
{
	all_echosounder_depth entry;
	entry.id_ = header.echo_count;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date, header.time);
    
    entry.depth_ = 100.*double(header.echo_depth); // Height in cm
	
    return entry;
}

mbes_ping::PingsT convert_matched_entries(all_mbes_ping::PingsT& pings, all_nav_entry::EntriesT& entries)
{
    mbes_ping::PingsT new_pings;
    new_pings.reserve(pings.size());

    std::stable_sort(entries.begin(), entries.end(), [](const all_nav_entry& entry1, const all_nav_entry& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });
    std::stable_sort(pings.begin(), pings.end(), [](const all_mbes_ping& ping1, const all_mbes_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });

    auto pos = entries.begin();
    for (all_mbes_ping& ping : pings) {
        pos = std::find_if(pos, entries.end(), [&](const all_nav_entry& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });

        mbes_ping new_ping;
        new_ping.time_stamp_ = ping.time_stamp_;
        new_ping.time_string_ = ping.time_string_;
        new_ping.first_in_file_ = ping.first_in_file_;
        new_ping.heading_ = ping.heading_;
        new_ping.pitch_ = 0.;
        new_ping.roll_ = 0.;
        if (pos == entries.end()) {
            double easting, northing;
            string utm_zone;
            tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(entries.back().lat_, entries.back().long_);
            new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
            //new_ping.pos_ = Eigen::Vector3d(easting, northing, -entries.back().depth_);
        }
        else {
            if (pos == entries.begin()) {
                double easting, northing;
                string utm_zone;
                tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(pos->lat_, pos->long_);
                new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
                //new_ping.pos_ = Eigen::Vector3d(easting, northing, -pos->depth_);
            }
            else {
                all_nav_entry& previous = *(pos - 1);
                double ratio = double(ping.time_stamp_ - previous.time_stamp_)/double(pos->time_stamp_ - previous.time_stamp_);
                double lat = previous.lat_ + ratio*(pos->lat_ - previous.lat_);
                double lon = previous.long_ + ratio*(pos->long_ - previous.long_);
                double depth = previous.depth_ + ratio*(pos->depth_ - previous.depth_);
                double easting, northing;
                string utm_zone;
                tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(lat, lon);
                new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
                //cout << "Filtered depth: " << depth << ", transducer depth: " << ping.transducer_depth_ << endl;
                //new_ping.pos_ = Eigen::Vector3d(easting, northing, -depth);
            }
        }

        new_ping.beams.reserve(ping.beams.size());
        new_ping.back_scatter.reserve(ping.beams.size());
        int i = 0;
        //cout << "Ping heading: " << new_ping.heading_ << endl;
        for (const Eigen::Vector3d& beam : ping.beams) {
            /*if (beam(2) > -5. || beam(2) < -25.) {
                ++i;
                continue;
            }*/
            Eigen::Matrix3d Rz = Eigen::AngleAxisd(new_ping.heading_, Eigen::Vector3d::UnitZ()).matrix();

            // it seems it has already been compensated for pitch, roll
            new_ping.beams.push_back(new_ping.pos_ + Rz*beam);
            //new_ping.beams.push_back(new_ping.pos_ + beam);
            new_ping.back_scatter.push_back(ping.reflectivities[i]);
            ++i;
        }

        new_pings.push_back(new_ping);
    }

    return new_pings;
}

mbes_ping::PingsT match_attitude(mbes_ping::PingsT& pings, all_nav_attitude::EntriesT& entries)
{
    struct unfolded_attitude {
        long long time_stamp_; // posix time stamp
        double roll;
        double pitch;
        double heading;
        double heave;
    };

    vector<unfolded_attitude> attitudes;
    unfolded_attitude attitude;
    for (const all_nav_attitude& entry : entries) {
        for (const all_nav_attitude_sample& sample : entry.samples) {
            attitude.roll = sample.roll;
            attitude.pitch = sample.pitch;
            attitude.heading = sample.heading;
            attitude.heave = sample.heave;
            attitude.time_stamp_ = entry.time_stamp_ + sample.ms_since_start;
            attitudes.push_back(attitude);
        }
    }

    mbes_ping::PingsT new_pings = pings;

    std::stable_sort(entries.begin(), entries.end(), [](const all_nav_attitude& entry1, const all_nav_attitude& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });

    auto pos = attitudes.begin();
    for (mbes_ping& ping : new_pings) {
        pos = std::find_if(pos, attitudes.end(), [&](const unfolded_attitude& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });

        ping.pitch_ = 0.;
        ping.roll_ = 0.;
        double heave;
        if (pos == attitudes.end()) {
            ping.pitch_ = attitudes.back().pitch;
            ping.roll_ = attitudes.back().roll;
            heave = attitudes.back().heave;
        }
        else if (pos == attitudes.begin()) {
                ping.pitch_ = pos->pitch;
                ping.roll_ = pos->roll;
                heave = pos->heave;
        }
        else {
            unfolded_attitude& previous = *(pos - 1);
            double ratio = double(ping.time_stamp_ - previous.time_stamp_)/double(pos->time_stamp_ - previous.time_stamp_);
            ping.pitch_ = previous.pitch + ratio*(pos->pitch - previous.pitch);
            ping.roll_ = previous.roll + ratio*(pos->roll - previous.roll);
            heave = previous.heave + ratio*(pos->heave - previous.heave);
        }
        //ping.pitch_ *= -1.;
        //ping.roll_ *= -1.;

        Eigen::Matrix3d Rz = Eigen::AngleAxisd(ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(1.*ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(1.*ping.roll_, Eigen::Vector3d::UnitX()).matrix();
        Eigen::Matrix3d R = Rz*Ry;
        
        for (Eigen::Vector3d& beam : ping.beams) {
            //beam = beam - ping.pos_;
            //Rz = Eigen::AngleAxisd(-ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
            beam = Rz.transpose()*(beam - ping.pos_);
            //Eigen::Matrix3d R = Rx*Ry*Rz;
            beam = ping.pos_ + R*beam; // + Eigen::Vector3d(0., 0., -heave);
        }
    }

    return new_pings;
}

csv_data::csv_asvp_sound_speed::EntriesT convert_sound_speeds(const all_mbes_ping::PingsT& pings)
{
    csv_data::csv_asvp_sound_speed sound_speed;

    auto min = std::min_element(pings.begin(), pings.end(), [](const all_mbes_ping& p1, const all_mbes_ping& p2)
    {
        return p1.transducer_depth_ < p2.transducer_depth_;
    });

    auto max = std::max_element(pings.begin(), pings.end(), [](const all_mbes_ping& p1, const all_mbes_ping& p2)
    {
        return p1.transducer_depth_ < p2.transducer_depth_;
    });

    int nbr_bins = 10;

    Eigen::VectorXd dbars = Eigen::VectorXd::Zero(nbr_bins);
    Eigen::VectorXd vels = Eigen::VectorXd::Zero(nbr_bins);
    Eigen::VectorXd counts = Eigen::VectorXd::Zero(nbr_bins);

    for (int i = 0; i < nbr_bins; ++i) {
        dbars[i] = min->transducer_depth_ + double(i)/double(nbr_bins)*(max->transducer_depth_ - min->transducer_depth_);
    }

    for (const all_mbes_ping& ping : pings) {
        int index = int(double(nbr_bins) * (ping.transducer_depth_ - min->transducer_depth_) / (max->transducer_depth_ - min->transducer_depth_));
        vels[index] += 0.1*ping.sound_vel_;
        counts[index] += 1.;
    }

    sound_speed.dbars = dbars;
    sound_speed.vels = vels.array()/counts.array();
    sound_speed.time_string_ = pings[0].time_string_;
    sound_speed.time_stamp_ = pings[0].time_stamp_;
    sound_speed.lat_ = 0.;
    sound_speed.long_ = 0.;

    return csv_data::csv_asvp_sound_speed::EntriesT { sound_speed };
}

std_data::attitude_entry::EntriesT convert_attitudes(const all_nav_attitude::EntriesT& attitudes)
{
    std_data::attitude_entry::EntriesT entries;
    std_data::attitude_entry entry;
    for (const all_nav_attitude& att : attitudes) {
        for (const all_nav_attitude_sample& sample : att.samples) {
            entry.roll = sample.roll;
            entry.pitch = sample.pitch;
            entry.yaw = sample.heading;
            entry.heave = sample.heave;
            entry.time_stamp_ = att.time_stamp_ + sample.ms_since_start;
            entry.time_string_ = std_data::time_string_from_time_stamp(entry.time_stamp_);
            entries.push_back(entry);
        }
        if (!att.samples.empty()) {
            entries[entries.size() - att.samples.size()].first_in_file_ = att.first_in_file_;
        }
    }

    std::stable_sort(entries.begin(), entries.end(), [](const std_data::attitude_entry& entry1, const std_data::attitude_entry& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });

    return entries;
}

bool StreamParser::parse_packet(const std::string& packet_load)
{
    std::istringstream input(packet_load);

    unsigned char start_id;
    unsigned char data_type;

    input.read(reinterpret_cast<char*>(&start_id), sizeof(start_id));
    input.read(reinterpret_cast<char*>(&data_type), sizeof(data_type));

    if (start_id != 2) {
        cout << "Start id not 2: " << start_id << endl;
        return false;
    }

    if (data_type == 88 && mbes_callback) {
        all_xyz88_datagram header;
        input.read(reinterpret_cast<char*>(&header), sizeof(header));
        mbes_callback(read_datagram<all_mbes_ping, all_xyz88_datagram>(input, header));
    }
    else if (data_type == 80 && nav_entry_callback) {
        all_position_datagram header;
        input.read(reinterpret_cast<char*>(&header), sizeof(header));
        nav_entry_callback(read_datagram<all_nav_entry, all_position_datagram>(input, header));
    }
    else {
        cout << "No callback for datagram: " << data_type << endl;
        return false;
    }

	unsigned char end_ident; // End identifier = ETX (Always 03h)
	unsigned short checksum; // Check sum of data between STX and ETX

    input.read(reinterpret_cast<char*>(&end_ident), sizeof(end_ident));
    input.read(reinterpret_cast<char*>(&checksum), sizeof(checksum));

    if (end_ident != 3) {
        cout << "End id not 3: " << end_ident << endl;
        return false;
    }

    return true;
}

} // namespace all_data

namespace std_data {

using namespace all_data;

template <>
all_mbes_ping::PingsT parse_file<all_mbes_ping>(const boost::filesystem::path& file)
{
    return parse_file_impl<all_mbes_ping, all_xyz88_datagram, 88>(file);
}

template <>
all_nav_entry::EntriesT parse_file<all_nav_entry>(const boost::filesystem::path& file)
{
    return parse_file_impl<all_nav_entry, all_position_datagram, 80>(file);
}

template <>
all_nav_depth::EntriesT parse_file<all_nav_depth>(const boost::filesystem::path& file)
{
    return parse_file_impl<all_nav_depth, all_depth_datagram, 104>(file);
}

template <>
all_nav_attitude::EntriesT parse_file<all_nav_attitude>(const boost::filesystem::path& file)
{
    return parse_file_impl<all_nav_attitude, all_attitude_datagram, 65>(file);
}

template <>
all_echosounder_depth::EntriesT parse_file<all_echosounder_depth>(const boost::filesystem::path& file)
{
    return parse_file_impl<all_echosounder_depth, all_echosounder_depth_datagram, 69>(file);
}

} // namespace std_data
