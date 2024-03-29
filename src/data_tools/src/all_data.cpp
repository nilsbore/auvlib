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
#include <string>

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
ReturnType read_datagram(std::istream& input, const AllHeaderType& header, int ascii_buffer_length = 0)
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
            int ascii_length = nbr_bytes - sizeof(start_id)- sizeof(data_type) - sizeof(header) - sizeof(end_ident)- sizeof(end_ident);
			returns.push_back(read_datagram<ReturnType, AllHeaderType>(input, header, ascii_length));
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
        if (counters[i] != 0){
            cout << std::dec << "Got " << counters[i] << " of type " << i << std::hex <<" with hex 0x"<< i << std::dec << endl;
        }
    }
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
all_mbes_ping read_datagram<all_mbes_ping, all_xyz88_datagram>(std::istream& input, const all_xyz88_datagram& header, int ascii_buffer_length)
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
        if (ping.rt_cleaning_info < 0 || (ping.detection_info & mask[0]) != 0) {
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
all_nav_entry read_datagram<all_nav_entry, all_position_datagram>(std::istream& input, const all_position_datagram& header, int ascii_buffer_length)
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
all_nav_depth read_datagram<all_nav_depth, all_depth_datagram>(std::istream& input, const all_depth_datagram& header, int ascii_buffer_length)
{
	all_nav_depth entry;
	entry.id_ = header.height_count;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date, header.time);
    
    entry.height = 100.*double(header.height); // Height in cm
    entry.height_type = header.height_type; // Height type
	
    return entry;
}

template <>
all_nav_attitude read_datagram<all_nav_attitude, all_attitude_datagram>(std::istream& input, const all_attitude_datagram& header, int ascii_buffer_length)
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
        //sample.heading = 0.5*M_PI-sample.heading; // this basically converts to yaw, should have that as marker instead
        sample.heave = 0.01*double(meas.heave);
        entry.samples.push_back(sample);
	}
    
	unsigned char system_desc; // Sensor system descriptor 1U
	input.read(reinterpret_cast<char*>(&system_desc), sizeof(system_desc));
	
    return entry;
}

template <>
all_echosounder_depth read_datagram<all_echosounder_depth, all_echosounder_depth_datagram>(std::istream& input, const all_echosounder_depth_datagram& header, int ascii_buffer_length)
{
	all_echosounder_depth entry;
	entry.id_ = header.echo_count;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date, header.time);
    
    entry.depth_ = 100.*double(header.echo_depth); // Height in cm
	
    return entry;
}

template <>
all_sound_speed_profile read_datagram<all_sound_speed_profile, all_sound_speed_profile_datagram>(std::istream& input, const all_sound_speed_profile_datagram& header, int ascii_buffer_length)
{
	
    all_sound_speed_profile entry;
    entry.id_ = header.profile_count;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date , header.time);
    all_sound_speed_profile_datagram_repeat one_element;
    for (int i = 0; i < header.nbr_entries; ++i) {
        input.read(reinterpret_cast<char*>(&one_element), sizeof(one_element));
        entry.depth_.push_back(one_element.depth);
        entry.sound_speed_.push_back(one_element.sound_speed);
    }
	unsigned char spare; // Spare (always 0)
	input.read(reinterpret_cast<char*>(&spare), sizeof(spare));
    return entry;
}

template <>
all_raw_range_and_beam_angle read_datagram<all_raw_range_and_beam_angle, all_raw_range_and_beam_angle_datagram>(std::istream& input, const all_raw_range_and_beam_angle_datagram& header, int ascii_buffer_length)
{
    all_raw_range_and_beam_angle raw;
	raw.id_ = header.ping_count;
    tie(raw.time_stamp_, raw.time_string_) = parse_all_time(header.date , header.time);
    raw.sound_vel_ = header.sound_vel;
    raw.D_scale_ = header.D_scale;
	all_raw_range_and_beam_angle_datagram_repeat_transmit transmit_tep;
    for (int i = 0; i < header.transmit_sector_nbr; ++i) {
		input.read(reinterpret_cast<char*>(&transmit_tep), sizeof(transmit_tep));
    }
    
    
    all_raw_range_and_beam_angle_datagram_repeat_received beam;
	for (int i = 0; i < header.received_beam_nbr; ++i) {
		input.read(reinterpret_cast<char*>(&beam), sizeof(beam));
        received_beam tep;
        tep.beam_pointing_angle_ = beam.beam_pointing_angle;
        tep.transmit_sector_number_ = beam.transmit_sector_number;
        tep.detection_info_ = beam.detection_info;
        tep.quality_factor_ = beam.quality_factor;
        tep.D_corr_ = beam.D_corr;
        tep.two_way_tranvel_time_ = beam.two_way_tranvel_time;
        tep.reflectivity_ = beam.reflectivity;
		raw.received_beam_.push_back(tep);
	}
    
    unsigned char spare; // Spare (always 0)
	input.read(reinterpret_cast<char*>(&spare), sizeof(spare));

	return raw;
}

template <>
all_installation_param read_datagram<all_installation_param, all_installation_para_datagram>(std::istream& input, const all_installation_para_datagram& header, int ascii_buffer_length)
{
    all_installation_param param;
	param.id_ = header.installation_datagram_count;
    tie(param.time_stamp_, param.time_string_) = parse_all_time(header.date , header.time);
    param.system_serial_number_ = header.serial_nbr;
    param.secondary_system_serial_number_ = header.secondary_serial_nbr;
    char * buffer = new char [ascii_buffer_length];
    input.read (buffer, ascii_buffer_length);
    stringstream str(buffer); 
    string x;
    while (getline(str, x, ',')) { 
        size_t split_pos = x.find("=");
        if (split_pos == std::string::npos) {
            continue;
        }
        string key = x.substr(0, split_pos);
        string value = x.substr(split_pos + 1);
        param.param_[key] = value;

        // cout<< x <<", " << key <<", " << value << endl;
    } 
	return param;
}


mbes_ping::PingsT convert_matched_entries(all_mbes_ping::PingsT& pings, all_nav_entry::EntriesT& entries, float roll=0.)
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
        if (pos == entries.end() || pos == entries.begin()) {
            double easting, northing;
            string utm_zone;
            tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(pos->lat_, pos->long_);
            new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
            //new_ping.pos_ = Eigen::Vector3d(easting, northing, -entries.back().depth_);
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

        new_ping.beams.reserve(ping.beams.size());
        new_ping.back_scatter.reserve(ping.beams.size());
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(new_ping.heading_, Eigen::Vector3d::UnitZ()).matrix();

        int i = 0;
        double step_roll = 2.*roll/ping.beams.size();
        double current_roll = -roll;
        for (const Eigen::Vector3d& beam : ping.beams) {
            Eigen::Matrix3d Rx = Eigen::AngleAxisd(M_PI/180*current_roll, Eigen::Vector3d::UnitX()).matrix();
            new_ping.beams.push_back(new_ping.pos_ + Rz*Rx*beam);
            new_ping.back_scatter.push_back(ping.reflectivities[i]);
            ++i;
            current_roll += step_roll;
        }

        new_pings.push_back(new_ping);
    }

    return new_pings;
}

mbes_ping::PingsT convert_matched_nav_and_attitude_entries(all_mbes_ping::PingsT& pings, all_nav_entry::EntriesT& nav_entries, all_nav_attitude::EntriesT& attitude_entries)
{

    mbes_ping::PingsT new_pings;
    new_pings.reserve(pings.size());
    vector<unfolded_attitude> attitudes = convert_attitude_timestamps(attitude_entries);

    // sort everything according to their timestamps
    std::stable_sort(attitudes.begin(), attitudes.end(), [](const unfolded_attitude& entry1, const unfolded_attitude& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });
    std::stable_sort(nav_entries.begin(), nav_entries.end(), [](const all_nav_entry& entry1, const all_nav_entry& entry2) {
        return entry1.time_stamp_ < entry2.time_stamp_;
    });
    std::stable_sort(pings.begin(), pings.end(), [](const all_mbes_ping& ping1, const all_mbes_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });

    auto nav_entry = nav_entries.begin();
    auto attitude_entry = attitudes.begin();

    for (all_mbes_ping& ping : pings) {
        mbes_ping new_ping;
        new_ping.time_stamp_ = ping.time_stamp_;
        new_ping.time_string_ = ping.time_string_;
        new_ping.first_in_file_ = ping.first_in_file_;
        new_ping.heading_ = ping.heading_;
        new_ping.pitch_ = 0.;
        new_ping.roll_ = 0.;

        // find the first nav_entry whose timestamp is after the ping
        nav_entry = std::find_if(nav_entry, nav_entries.end(), [&](const all_nav_entry& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });
        // ping position = nav_entry's position
        if (nav_entry == nav_entries.end() || nav_entry == nav_entries.begin()) {
            double easting, northing;
            string utm_zone;
            tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(nav_entry->lat_, nav_entry->long_);
            new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
        } else {
            all_nav_entry& previous = *(nav_entry - 1);
            double ratio = double(ping.time_stamp_ - previous.time_stamp_)/double(nav_entry->time_stamp_ - previous.time_stamp_);
            double lat = previous.lat_ + ratio*(nav_entry->lat_ - previous.lat_);
            double lon = previous.long_ + ratio*(nav_entry->long_ - previous.long_);
            double depth = previous.depth_ + ratio*(nav_entry->depth_ - previous.depth_);
            double easting, northing;
            string utm_zone;
            tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(lat, lon);
            new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
        }

        // find the first attitude_entry whose timestamp is after the ping
        attitude_entry = std::find_if(attitude_entry, attitudes.end(), [&](const unfolded_attitude& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });
        if (attitude_entry == attitudes.begin() || attitude_entry == attitudes.end()) {
            new_ping.pitch_ = attitude_entry->pitch;
            new_ping.roll_ = attitude_entry->roll;
            new_ping.heading_ = attitude_entry->heading;
        } else {
            unfolded_attitude& previous = *(attitude_entry - 1);
            double ratio = double(new_ping.time_stamp_ - previous.time_stamp_)/double(attitude_entry->time_stamp_ - previous.time_stamp_);
            new_ping.pitch_ = previous.pitch + ratio*(attitude_entry->pitch - previous.pitch);
            new_ping.roll_ = previous.roll + ratio*(attitude_entry->roll - previous.roll);
            new_ping.heading_ = previous.heading + ratio*(attitude_entry->heading - previous.heading);
        }

        new_ping.beams.reserve(ping.beams.size());
        new_ping.back_scatter.reserve(ping.beams.size());

        Eigen::Matrix3d Rz = Eigen::AngleAxisd(new_ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(new_ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(new_ping.roll_, Eigen::Vector3d::UnitX()).matrix();
        Eigen::Matrix3d R = Rz * Ry * Rx;

        int i = 0;
        for (Eigen::Vector3d& beam : ping.beams) {
            //beam += Eigen::Vector3d(-1.086, -0.001, -0.414);
            new_ping.beams.push_back(new_ping.pos_ + R*beam);
            new_ping.back_scatter.push_back(ping.reflectivities[i]);
            ++i;
        }
        new_pings.push_back(new_ping);
    }
    return new_pings;
}

vector<unfolded_attitude> convert_attitude_timestamps(all_nav_attitude::EntriesT& entries)
{
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
    return attitudes;
}

mbes_ping::PingsT match_attitude(mbes_ping::PingsT& pings, all_nav_attitude::EntriesT& entries)
{

    vector<unfolded_attitude> attitudes = convert_attitude_timestamps(entries);
    mbes_ping::PingsT new_pings = pings;

    std::stable_sort(attitudes.begin(), attitudes.end(), [](const unfolded_attitude& entry1, const unfolded_attitude& entry2) {
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

template <>
all_sound_speed_profile::EntriesT parse_file<all_sound_speed_profile>(const boost::filesystem::path& file)
{
    return parse_file_impl<all_sound_speed_profile, all_sound_speed_profile_datagram, 85>(file);
}

template <>
all_raw_range_and_beam_angle::EntriesT parse_file<all_raw_range_and_beam_angle>(const boost::filesystem::path& file)
{
    return parse_file_impl<all_raw_range_and_beam_angle, all_raw_range_and_beam_angle_datagram, 78>(file);
}

template <>
all_installation_param::EntriesT parse_file<all_installation_param>(const boost::filesystem::path& file)
{   
    // actually there will be only one all_installation_param, so return size should be one
    return parse_file_impl<all_installation_param, all_installation_para_datagram, 73>(file);
}

} // namespace std_data
