#include <data_tools/all_data.h>
#include <liball/all.h>
#include <endian.h>
#include <fstream>

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

template <>
all_mbes_ping read_datagram<all_mbes_ping, all_xyz88_datagram>(std::ifstream& input, const all_xyz88_datagram& header)
{
	cout << "Total number of beams: " << header.nbr_beams << endl;
	all_mbes_ping new_ping;
	new_ping.id_ = header.ping_count;
	new_ping.heading_ = header.heading;
	new_ping.sound_vel_ = header.sound_vel;
	new_ping.transducer_depth_ = header.transducer_depth;
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
