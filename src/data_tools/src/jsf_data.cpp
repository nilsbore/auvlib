/* Copyright 2018 Nils Bore (nbore@kth.se), Yiping Xie (yipingx@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <data_tools/jsf_data.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <fcntl.h>
#ifdef _MSC_VER
  #include <io.h>
#else
  #include <unistd.h>
#endif
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bitset>
#include <data_tools/lat_long_utm.h>

# define SONAR_MESSAGE_HEADER_START 0X1601
# define SONAR_DATA_TYPE 0X0050
# define DVL_DATA_TYPE 0X0820
# define SITUATION_DATA_TYPE 0X082A

using namespace std;

namespace jsf_data{

    jsf_sss_ping::PingsT filter_frequency(const jsf_sss_ping::PingsT& pings, int desired_freq)
    {
        jsf_sss_ping::PingsT filtered_pings;

        std::copy_if(pings.begin(), pings.end(), std::back_inserter(filtered_pings), [&](const jsf_sss_ping& ping){ return ping.frequency == desired_freq; });

        return filtered_pings;
    }

cv::Mat make_waterfall_image(const jsf_sss_ping::PingsT& pings)
{
    int rows = pings.size();
    int cols = pings[0].port.pings.size() + pings[0].stbd.pings.size();
    cv::Mat swath_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < pings.size(); ++i) {
        for (int j = 0; j < pings[i].port.pings.size(); ++j) {
            uchar intensity = uchar(std::min(std::max(255.*.005*pings[i].port.pings[j], 0.), 255.));
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, pings[0].stbd.pings.size()+j);
            p->z = intensity;
            p->y = intensity;
            p->x = intensity;
        }
        for (int j = 0; j < pings[i].stbd.pings.size(); ++j) {
            uchar intensity = uchar(std::min(std::max(255.*.005*pings[i].stbd.pings[j], 0.), 255.));
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, pings[0].stbd.pings.size()-j-1);
            p->z = intensity;
            p->y = intensity;
            p->x = intensity;
        }
    }
    cv::Mat resized_swath_img;//dst image
    cv::resize(swath_img, resized_swath_img, cv::Size(512, rows));//resize image
    
    return resized_swath_img;
}

void show_waterfall_image(const jsf_sss_ping::PingsT& pings)
{
    cv::Mat waterfall_image = make_waterfall_image(pings);
    cv::imshow("Waterfall image", waterfall_image);
    cv::waitKey();
}

// skip data 
void skip_data(ifstream& input, jsf_msg_header jsf_hdr)
{
    int nbr_bytes;
    for (int i = 0; i < jsf_hdr.following_bytes; i++) {
        input.read(reinterpret_cast<char*>(&nbr_bytes), sizeof(char));
    }
}


jsf_sss_ping_side process_side_scan_ping_side(ifstream& input,  jsf_msg_header& jsf_hdr,  jsf_sonar_data_msg_header& jsf_sonar_data_hdr)
{
    jsf_sss_ping_side ping_side;

    ping_side.time_duration = double(jsf_sonar_data_hdr.spls_num_in_pkt - 1)*double(jsf_sonar_data_hdr.spl_intvl_in_ns)*1e-9;


    //cout << "Freq: " << jsf_sonar_data_hdr.spl_freq_in_hz << endl;

    if (jsf_sonar_data_hdr.data_format==0) {
        int16_t env_data;
        for (int i = 0; i < jsf_sonar_data_hdr.spls_num_in_pkt; ++i) {
            input.read(reinterpret_cast<char*>(&env_data), sizeof(env_data));
            ping_side.pings.push_back(ldexpf((float)env_data, -jsf_sonar_data_hdr.weighting_factor_n));

        }
    }
    else if (jsf_sonar_data_hdr.data_format==1) {
        cout << "Data format: " << jsf_sonar_data_hdr.data_format << "has real and imginary part, stored in pings and pings_phase respectively" << endl;
        int16_t analytic_sig_data[2];

        for (int i = 0; i < jsf_sonar_data_hdr.spls_num_in_pkt; ++i) {
            input.read(reinterpret_cast<char*>(&analytic_sig_data), sizeof(analytic_sig_data));
            ping_side.pings.push_back(ldexpf((float)analytic_sig_data[0], -jsf_sonar_data_hdr.weighting_factor_n));
            ping_side.pings_phase.push_back(ldexpf((float)analytic_sig_data[1], -jsf_sonar_data_hdr.weighting_factor_n));
        }
    }
    else{
        cout << "Skip data format: " << jsf_sonar_data_hdr.data_format << endl;
        for (int i = 0; i < jsf_hdr.following_bytes-sizeof(jsf_sonar_data_hdr); i++) {
            int16_t nbr_bytes;
            input.read(reinterpret_cast<char*>(&nbr_bytes), sizeof(char));
        }
    }
    return ping_side;
 }



template <typename ReturnType, typename JsfHeaderType>
ReturnType read_datagram(std::ifstream& input,  JsfHeaderType& header,  jsf_msg_header& jsf_hdr)
{
    ReturnType rtn;
	return rtn;
}

template <typename ReturnType, typename JsfHeaderType, int Code>
vector<ReturnType, Eigen::aligned_allocator<ReturnType> > parse_file_impl(const boost::filesystem::path& path)
{
    vector<ReturnType, Eigen::aligned_allocator<ReturnType> > returns;
    if (boost::filesystem::extension(path) != ".JSF" && boost::filesystem::extension(path) != ".jsf") {
        cout << "Not an .JSF file, skipping..." << endl;
        cout << "Extension: " << boost::filesystem::extension(path) << endl;
        return returns;
    }

    ifstream input;
    input.open(path.string(), ios::binary);
    if (input.fail()) {
        cout << "ERROR: Cannot open the file..." << endl;
        exit(0);
        return returns;
    }
    while (!input.eof()) {
        jsf_msg_header jsf_hdr;
        input.read(reinterpret_cast<char*>(&jsf_hdr),sizeof(jsf_hdr));
        if (jsf_hdr.start_marker != SONAR_MESSAGE_HEADER_START) {
            cout << "Invalid file format! start marker: " << jsf_hdr.start_marker << endl;
            break;
        }

        if (jsf_hdr.msg_type == Code) {
            JsfHeaderType header;
            input.read(reinterpret_cast<char*>(&header), sizeof(header));
            returns.push_back(read_datagram<ReturnType, JsfHeaderType>(input, header, jsf_hdr));
            returns.back().first_in_file_ = false;
        }
        else {
            skip_data(input,jsf_hdr);
        }
    }

    if (!returns.empty()) {
        returns[0].first_in_file_ = true;
    }

	return returns;
    
}

template <>
jsf_sss_ping read_datagram<jsf_sss_ping, jsf_sonar_data_msg_header>(std::ifstream& input,  jsf_sonar_data_msg_header& jsf_sonar_data_hdr,  jsf_msg_header& jsf_hdr)
{
    jsf_sss_ping ping;
    jsf_sss_ping_side ping_side;
    ping.frequency = jsf_sonar_data_hdr.spl_freq_in_hz;
    ping.sound_vel = jsf_sonar_data_hdr.sound_speed_in_m_per_s;
    ping.rpy = Eigen::Vector3d(jsf_sonar_data_hdr.roll, jsf_sonar_data_hdr.pitch, jsf_sonar_data_hdr.compass_heading);
    ping.rpy.head<2>() = M_PI/32768.*ping.rpy.head<2>();
    ping.rpy[2] = .5*M_PI - M_PI/180.*0.01*ping.rpy[2];

    // NOTE: this is only valid if coord_units == 2
    ping.lat_ = 0.0001/60.*double(jsf_sonar_data_hdr.y_coord);
    ping.long_ = 0.0001/60.*double(jsf_sonar_data_hdr.x_coord);
    double easting, northing;
    string utm_zone;
    tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(ping.lat_, ping.long_);
    //cout << "UTM ZONE: " << utm_zone << endl;
    ping.utm_zone = utm_zone;
    ping.pos_ = Eigen::Vector3d(easting, northing, -0.001*jsf_sonar_data_hdr.depth_in_mm);
    ping_side = process_side_scan_ping_side(input, jsf_hdr, jsf_sonar_data_hdr);

    //cout << "Coord units: " << jsf_sonar_data_hdr.coord_units << endl;

    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime data_time;

    if (jsf_hdr.prot_ver >= 8) {
        data_time = epoch + boost::posix_time::seconds(jsf_sonar_data_hdr.ping_time_in_sec) + boost::posix_time::milliseconds(jsf_sonar_data_hdr.today_in_ms%1000);
    }
    else {
        boost::posix_time::ptime data_time(boost::gregorian::date(jsf_sonar_data_hdr.cpu_year, 1, 1), boost::posix_time::hours(jsf_sonar_data_hdr.cpu_day*24-24)+boost::posix_time::minutes(0)+boost::posix_time::seconds(0)+boost::posix_time::milliseconds(jsf_sonar_data_hdr.today_in_ms)); 
    }
    stringstream time_ss;
    time_ss << data_time;
    ping.time_string_ = time_ss.str();
    boost::posix_time::time_duration const diff = data_time - epoch;
    ping.time_stamp_ = diff.total_milliseconds();

    if (jsf_hdr.channel_num == 0) {
        ping.port = ping_side;
    }
    else {
        ping.stbd = ping_side;
    }
  
    ping.first_in_file_ = false;

    return ping;

}

template <>
jsf_dvl_ping read_datagram<jsf_dvl_ping, jsf_dvl_msg_header>(std::ifstream& input,  jsf_dvl_msg_header& jsf_dvl_msg,  jsf_msg_header& jsf_hdr)
{
    jsf_dvl_ping ping;
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime data_time;

    data_time = epoch + boost::posix_time::seconds(jsf_dvl_msg.time_in_sec) + boost::posix_time::milliseconds(jsf_dvl_msg.ms_in_cur_sec);
    
    stringstream time_ss;
    time_ss << data_time;
    ping.time_string_ = time_ss.str();
    boost::posix_time::time_duration const diff = data_time - epoch;
    ping.time_stamp_ = diff.total_milliseconds();

    ping.vel_wrt_bottom_ = Eigen::Vector3d(jsf_dvl_msg.x_vel_wrt_bottom/1000., jsf_dvl_msg.y_vel_wrt_bottom/1000., jsf_dvl_msg.z_vel_wrt_bottom/1000.);
    ping.vel_wrt_water_ = Eigen::Vector3d(jsf_dvl_msg.x_vel_wrt_water/1000., jsf_dvl_msg.y_vel_wrt_water/1000., jsf_dvl_msg.z_vel_wrt_water/1000.);
    ping.dist_to_bottom_ = Eigen::Vector4d(jsf_dvl_msg.dist_to_bottom_in_cm[0]/100., jsf_dvl_msg.dist_to_bottom_in_cm[1]/100,jsf_dvl_msg.dist_to_bottom_in_cm[2]/100.,jsf_dvl_msg.dist_to_bottom_in_cm[3]/100.);
    ping.depth_ = jsf_dvl_msg.depth_in_dm/10.;
    ping.pitch_ = jsf_dvl_msg.pitch/100./180.*M_PI;
    ping.roll_ = jsf_dvl_msg.roll/100./180.*M_PI;
    ping.heading_ = jsf_dvl_msg.heading/100./180.*M_PI;
    ping.salinity_ = jsf_dvl_msg.salinity;
    ping.temp_ = jsf_dvl_msg.temp/100.;
    ping.sound_vel_ = jsf_dvl_msg.sound_vel;

    // initialize the flag
    ping.flag_["Vxy"] = false; // bit 0
    ping.flag_["Vz"] = false; // bit 2
    ping.flag_["Vxy_water"] = false; // bit 3
    ping.flag_["Vz_water"] = false; // bit 4
    ping.flag_["dist_bottom"] = false; // bit 5
    ping.flag_["heading"] = false; // bit 6
    ping.flag_["pitch"] = false; // bit 7
    ping.flag_["roll"] = false; // bit 8
    ping.flag_["temp"] = false; // bit 9
    ping.flag_["depth"] = false; // bit 10
    ping.flag_["salinity"] = false; // bit 11
    ping.flag_["sound_vel"] = false; // bit 12

    ping.ship_coord_ = false; // bit 1
    ping.error_ = false; // bit 31

    // bit 0
    if (jsf_dvl_msg.flag & 0x01) ping.flag_["Vxy"] = true;
     
    // bit 1
    if (jsf_dvl_msg.flag & 0x02) ping.ship_coord_ = true;
    

    // bit 2
    if (jsf_dvl_msg.flag & 0x04) ping.flag_["Vz"] = true;

    // bit 3
    if (jsf_dvl_msg.flag & 0x08) ping.flag_["Vxy_water"] = true;
    
    // bit 4
    if (jsf_dvl_msg.flag & 0x10) ping.flag_["Vz_water"] = true;

    // bit 5
    if (jsf_dvl_msg.flag & 0x20) ping.flag_["dist_bottom"] = true;
    
    // bit 6
    if (jsf_dvl_msg.flag & 0x40) ping.flag_["heading"] = true;
    
    // bit 7
    if (jsf_dvl_msg.flag & 0x80) ping.flag_["pitch"] = true;
    
    // bit 8
    if (jsf_dvl_msg.flag & 0x100) ping.flag_["roll"] = true;
    
    // bit 9
    if (jsf_dvl_msg.flag & 0x200) ping.flag_["temp"] = true;

    // bit 10
    if (jsf_dvl_msg.flag & 0x400) ping.flag_["depth"] = true;

    // bit 11
    if (jsf_dvl_msg.flag & 0x800) ping.flag_["salinity"] = true;

    // bit 12
    if (jsf_dvl_msg.flag & 0x1000) ping.flag_["sound_vel"] = true; 

    // bit 31
    if(jsf_dvl_msg.flag & 0x100000000){
        cout << "Error detected! flag in decimal: " << fixed << jsf_dvl_msg.flag << endl;
        ping.error_ = true;
    }

    return ping;

}

std_data::sss_ping::PingsT convert_to_xtf_pings(const jsf_sss_ping::PingsT& pings)
{
    std_data::sss_ping::PingsT converted;
    converted.reserve(pings.size());

    for (const jsf_sss_ping& ping : pings) {
        std_data::sss_ping cping;
        cping.time_stamp_ = ping.time_stamp_;
        cping.time_string_ = ping.time_string_;
        cping.first_in_file_ = ping.first_in_file_;
        cping.roll_ = ping.rpy[0];
        cping.pitch_ = ping.rpy[1];
        cping.heading_ = ping.rpy[2];
        cping.lat_ = ping.lat_;
        cping.long_ = ping.long_;
        cping.pos_ = ping.pos_;
        cping.sound_vel_ = ping.sound_vel;

        cping.stbd.time_duration = ping.stbd.time_duration;
        cping.stbd.pings.resize(ping.stbd.pings.size());
        for (int i = 0; i < ping.stbd.pings.size(); ++i)
        {
            cping.stbd.pings[i] = int(2.*32767.*(.005*ping.stbd.pings[i] - .5));
        }

        cping.port.time_duration = ping.port.time_duration;
        cping.port.pings.resize(ping.port.pings.size());
        for (int i = 0; i < ping.port.pings.size(); ++i)
        {
            cping.port.pings[i] = int(2.*32767.*(.005*ping.port.pings[i] - .5));
        }

        converted.push_back(cping);
    }

    return converted;
}


} // namespace jsf_data



namespace std_data {

using namespace jsf_data;

template <>
jsf_sss_ping::PingsT parse_file<jsf_sss_ping>(const boost::filesystem::path& file)
{
    jsf_sss_ping::PingsT pings = parse_file_impl<jsf_sss_ping, jsf_sonar_data_msg_header, 80>(file);
    jsf_sss_ping::PingsT fixed_pings;
    int start;
    if ((pings[0].time_stamp_ - pings[1].time_stamp_) < 2) {
        start = 0;
    }
    else if ((pings[1].time_stamp_ - pings[2].time_stamp_) < 2) {
        start = 1;
    }
    for (int i = start; i < pings.size()-1; i+=2) {
        jsf_sss_ping fixed_ping;
        if (pings[i].port.pings.size() != 0 && pings[i+1].stbd.pings.size() != 0) {

            fixed_ping = pings[i];
            fixed_ping.stbd = pings[i+1].stbd;
            fixed_pings.push_back(fixed_ping);
        }
        else if (pings[i].stbd.pings.size() != 0 && pings[i+1].port.pings.size() != 0) {
            fixed_ping = pings[i];
            fixed_ping.port = pings[i+1].port;
            fixed_pings.push_back(fixed_ping);
        }
        else {
            cout << "Invalid data format! Channel numbers are not 0 and 1!" << endl;
            break;
        }
    }
    return fixed_pings;
}

template <>
jsf_dvl_ping::PingsT parse_file<jsf_dvl_ping>(const boost::filesystem::path& file)
{
    return parse_file_impl<jsf_dvl_ping, jsf_dvl_msg_header, 2080>(file);
}

} // namespace std_data
