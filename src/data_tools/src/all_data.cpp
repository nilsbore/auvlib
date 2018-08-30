#include <data_tools/all_data.h>
#include <data_tools/lat_long_utm.h>
#include <liball/all.h>
#include <endian.h>
#include <fstream>
#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

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
ReturnType read_datagram(std::ifstream& input, const AllHeaderType& header)
{
    ReturnType rtn;
	return rtn;
}

template <typename ReturnType, typename AllHeaderType, int Code>
vector<ReturnType, Eigen::aligned_allocator<ReturnType> > parse_file_impl(const boost::filesystem::path& path)
{
    std::ifstream input;
	input.open(path.string(), std::ios::binary);
	if (input.fail()) {
        cout << "ERROR: Cannot open the file..." << endl;
        exit(0);
    }
    cout << "Opened " << path << " for reading..." << endl;

    unsigned int nbr_bytes;
    unsigned char start_id;
    unsigned char data_type;

    // end codes that we need to read
	unsigned char spare; // Spare (always 0)
	unsigned char end_ident; // End identifier = ETX (Always 03h)
	unsigned short checksum; // Check sum of data between STX and ETX

    //vector<vector<all_xyz88_datagram_repeat> > many_pings;

	int pos_counter = 0;
	vector<ReturnType, Eigen::aligned_allocator<ReturnType> > returns;
	while (!input.eof()) {
		cout << "Trying to read nbr_bytes with size " << sizeof(nbr_bytes) << endl;
		input.read(reinterpret_cast<char*>(&nbr_bytes), sizeof(nbr_bytes));
		cout << "Number bytes has little endian: " << nbr_bytes << endl;
		cout << "Trying to read start_id with size " << sizeof(start_id) << endl;
		input.read(reinterpret_cast<char*>(&start_id), sizeof(start_id));
		cout << "Trying to read data_type with size " << sizeof(data_type) << endl;
		input.read(reinterpret_cast<char*>(&data_type), sizeof(data_type));
		cout << "Number bytes: " << nbr_bytes << endl;
		cout << "Start id: " << int(start_id) << endl;
		cout << "Data type must be " << 88 << endl;
		if (data_type == 80) {
		    ++pos_counter;
		}
	    if (data_type == Code) {
	        //all_xyz88_datagram mbes_header;
			AllHeaderType header;
		    cout << "Is a MB reading, code: " << int(data_type) << endl;
		    input.read(reinterpret_cast<char*>(&header), sizeof(header));
			returns.push_back(read_datagram<ReturnType, AllHeaderType>(input, header));
		    input.read(reinterpret_cast<char*>(&spare), sizeof(spare));
		    input.read(reinterpret_cast<char*>(&end_ident), sizeof(end_ident));
		    input.read(reinterpret_cast<char*>(&checksum), sizeof(checksum));
			cout << "End identifier: " << end_ident << endl;
			//many_pings.push_back(pings);
		}
		else {
		    cout << "No MB reading, code: " << int(data_type) << endl;
            int skip_bytes = nbr_bytes-sizeof(start_id)-sizeof(data_type);
            input.ignore(skip_bytes);
		}
	}

	cout << "Got " << pos_counter << " position entries" << endl;

	/*
    cv::Mat img = make_waterfall_image(many_pings);
	cv::imshow("My image", img);
	cv::waitKey();
	*/

	return returns;
}

pair<long long, string> parse_all_time(unsigned int date, unsigned int time)
{
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    const std::locale loc = std::locale(std::locale::classic(), new boost::posix_time::time_input_facet("YYYYmmdd"));

    boost::posix_time::time_duration time_d = boost::posix_time::milliseconds(time);
    //std::istringstream is(to_string(date));
    cout << "Date: " << to_string(date) << endl;
    //is.imbue(loc);
    boost::gregorian::date date_t = boost::gregorian::date_from_iso_string(to_string(date));
    boost::posix_time::ptime t(date_t, time_d);
    //is >> date_t;
    long long time_stamp_ = (t - epoch).total_milliseconds();
    stringstream time_ss;
    time_ss << t;
    string time_string_ = time_ss.str();
    cout << "Time string: " << time_string_ << endl;

    return make_pair(time_stamp_, time_string_);
}

template <>
all_mbes_ping read_datagram<all_mbes_ping, all_xyz88_datagram>(std::ifstream& input, const all_xyz88_datagram& header)
{
	cout << "Total number of beams: " << header.nbr_beams << endl;
	all_mbes_ping new_ping;
	new_ping.id_ = header.ping_count;
	//new_ping.heading_ = header.heading;
    new_ping.heading_ = M_PI/180.*double(header.heading);
    new_ping.heading_ = 0.5*M_PI-new_ping.heading_; // TODO: need to keep this for old data
	new_ping.sound_vel_ = header.sound_vel;
	new_ping.transducer_depth_ = header.transducer_depth;
    tie(new_ping.time_stamp_, new_ping.time_string_) = parse_all_time(header.date, header.time);
	vector<all_xyz88_datagram_repeat> pings;
	all_xyz88_datagram_repeat ping;
	for (int i = 0; i < header.nbr_beams; ++i) {
		input.read(reinterpret_cast<char*>(&ping), sizeof(ping));
		pings.push_back(ping);
		Eigen::Vector3d pos(ping.along_track, ping.across_track, -ping.depth);
		new_ping.beams.push_back(pos);
		new_ping.reflectivities.push_back(ping.reflectivity);
	}
	return new_ping;
}

template <>
all_nav_entry read_datagram<all_nav_entry, all_position_datagram>(std::ifstream& input, const all_position_datagram& header)
{
	cout << "Got a position datagram, skipping: " << int(header.nbr_bytes_input) << endl;
    input.ignore(header.nbr_bytes_input);
	all_nav_entry entry;
	entry.id_ = header.pos_count;
	entry.lat_ = double(header.latitude)/20000000.;
	entry.long_ = double(header.longitude)/10000000.;
	entry.heading_ = double(header.heading)*0.01;
	entry.course_over_ground_ = double(header.course_over_ground)*0.01;
    tie(entry.time_stamp_, entry.time_string_) = parse_all_time(header.date, header.time);
	return entry;
}

template <>
all_mbes_ping::PingsT parse_file<all_mbes_ping>(const boost::filesystem::path& path) //, int code)
{
    return parse_file_impl<all_mbes_ping, all_xyz88_datagram, 88>(path);
}

template <>
all_nav_entry::EntriesT parse_file<all_nav_entry>(const boost::filesystem::path& path) //, int code)
{
    return parse_file_impl<all_nav_entry, all_position_datagram, 80>(path);
}

mbes_ping::PingsT convert_matched_entries(all_mbes_ping::PingsT& pings, all_nav_entry::EntriesT& entries)
{
    mbes_ping::PingsT new_pings;

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
        new_ping.heading_ = -ping.heading_;
        new_ping.pitch_ = 0.;
        new_ping.roll_ = 0.;
        if (pos == entries.end()) {
            double easting, northing;
            string utm_zone;
            tie(northing, easting, utm_zone) = lat_long_to_UTM(entries.back().lat_, entries.back().long_);
            new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
        }
        else {
            if (pos == entries.begin()) {
                double easting, northing;
                string utm_zone;
                tie(northing, easting, utm_zone) = lat_long_to_UTM(pos->lat_, pos->long_);
                new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
            }
            else {
                all_nav_entry& previous = *(pos - 1);
                double ratio = double(ping.time_stamp_ - previous.time_stamp_)/double(pos->time_stamp_ - previous.time_stamp_);
                double lat = previous.lat_ + ratio*(pos->lat_ - previous.lat_);
                double lon = previous.long_ + ratio*(pos->long_ - previous.long_);
                double easting, northing;
                string utm_zone;
                tie(northing, easting, utm_zone) = lat_long_to_UTM(lat, lon);
                new_ping.pos_ = Eigen::Vector3d(easting, northing, -ping.transducer_depth_);
            }
        }

        int i = 0;
        for (const Eigen::Vector3d& beam : ping.beams) {
            /*if (beam(2) > -5. || beam(2) < -25.) {
                ++i;
                continue;
            }*/
            Eigen::Matrix3d Rz = Eigen::AngleAxisd(new_ping.heading_, Eigen::Vector3d::UnitZ()).matrix();

            // it seems it has already been compensated for pitch, roll
            new_ping.beams.push_back(new_ping.pos_ + Rz*beam);
            new_ping.back_scatter.push_back(ping.reflectivities[i]);
            ++i;
        }

        new_pings.push_back(new_ping);
    }

    return new_pings;
}
