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

# define SONAR_MESSAGE_HEADER_START 0X1601
# define SONAR_DATA_TYPE 0X0050
# define DVL_DATA_TYPE 0X0820
# define SITUATION_DATA_TYPE 0X082A

using namespace std;

namespace jsf_data{


cv::Mat make_waterfall_image(const jsf_sss_ping::PingsT& pings)
{
    int rows = pings.size();
    int cols = pings[0].port.pings.size() + pings[0].stbd.pings.size();
    cv::Mat swath_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < pings.size(); ++i) {
        for (int j = 0; j < pings[i].port.pings.size(); ++j) {
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, pings[0].stbd.pings.size()+j);
            p->z = uchar(255.*pings[i].port.pings[j]);
            p->y = uchar(255.*pings[i].port.pings[j]);
            p->x = uchar(255.*pings[i].port.pings[j]);
        }
        for (int j = 0; j < pings[i].stbd.pings.size(); ++j) {
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, pings[0].stbd.pings.size()-j-1);
            p->z = uchar(255.*pings[i].stbd.pings[j]);
            p->y = uchar(255.*pings[i].stbd.pings[j]);
            p->x = uchar(255.*pings[i].stbd.pings[j]);
        }
    }
    cv::Mat resized_swath_img;//dst image
    cv::resize(swath_img, resized_swath_img, cv::Size(cols/32, rows/4));//resize image
    
    return resized_swath_img;
}




void show_waterfall_image(const jsf_sss_ping::PingsT& pings)
{
    cv::Mat waterfall_image = make_waterfall_image(pings);
    cv::imshow("Waterfall image", waterfall_image);
    cv::waitKey();
}

// skip data 
void skip_data(ifstream& input, jsf_msg_header jsf_hdr){
    int nbr_bytes;
    for (int i = 0; i < jsf_hdr.following_bytes; i++){   
        input.read(reinterpret_cast<char*>(&nbr_bytes), sizeof(char));
    }
}


jsf_sss_ping_side process_side_scan_ping_side(ifstream& input,  jsf_msg_header& jsf_hdr,  jsf_sonar_data_msg_header& jsf_sonar_data_hdr){
    jsf_sss_ping_side ping_side;

    if (jsf_sonar_data_hdr.data_format==0){
        int16_t env_data;
        for (int i=0; i<jsf_sonar_data_hdr.spls_num_in_pkt; ++i){
            input.read(reinterpret_cast<char*>(&env_data), sizeof(env_data));
            ping_side.pings.push_back(ldexpf((float)env_data, -jsf_sonar_data_hdr.weighting_factor_n));

        }
    }
    else if ((jsf_sonar_data_hdr.data_format==1)){
        cout << "Data format: " << jsf_sonar_data_hdr.data_format << "has real and imginary part, stored in pings and pings_phase respectively" << endl;
        int16_t analytic_sig_data[2];

        for (int i=0; i<jsf_sonar_data_hdr.spls_num_in_pkt; ++i){
            input.read(reinterpret_cast<char*>(&analytic_sig_data), sizeof(analytic_sig_data));
            ping_side.pings.push_back(ldexpf((float)analytic_sig_data[0], -jsf_sonar_data_hdr.weighting_factor_n));
            ping_side.pings_phase.push_back(ldexpf((float)analytic_sig_data[1], -jsf_sonar_data_hdr.weighting_factor_n));

            }
    }
    else{
        cout << "Skip data format: " << jsf_sonar_data_hdr.data_format << endl;
        for (int i = 0; i < jsf_hdr.following_bytes-sizeof(jsf_sonar_data_hdr); i++){   
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
    if (boost::filesystem::extension(path) != ".JSF") {
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
    while (!input.eof()){
        jsf_msg_header jsf_hdr;
        input.read(reinterpret_cast<char*>(&jsf_hdr),sizeof(jsf_hdr));
        if (jsf_hdr.start_marker != SONAR_MESSAGE_HEADER_START){
            cout << "Invalid file format! start marker: " << jsf_hdr.start_marker << endl;
            break;
        }

        if (jsf_hdr.msg_type==Code){
            JsfHeaderType header;
            input.read(reinterpret_cast<char*>(&header), sizeof(header));
            returns.push_back(read_datagram<ReturnType, JsfHeaderType>(input, header, jsf_hdr));
            returns.back().first_in_file_ = false;
        }
        else{
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
    ping_side = process_side_scan_ping_side(input, jsf_hdr, jsf_sonar_data_hdr);

    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime data_time;

    if (jsf_hdr.prot_ver>=8){
        data_time = epoch + boost::posix_time::seconds(jsf_sonar_data_hdr.ping_time_in_sec) + boost::posix_time::milliseconds(jsf_sonar_data_hdr.today_in_ms%1000);
    }
    else{
        boost::posix_time::ptime data_time(boost::gregorian::date(jsf_sonar_data_hdr.cpu_year, 1, 1), boost::posix_time::hours(jsf_sonar_data_hdr.cpu_day*24-24)+boost::posix_time::minutes(0)+boost::posix_time::seconds(0)+boost::posix_time::milliseconds(jsf_sonar_data_hdr.today_in_ms)); 
    }
    stringstream time_ss;
    time_ss << data_time;
    ping.time_string_ = time_ss.str();
    boost::posix_time::time_duration const diff = data_time - epoch;
    ping.time_stamp_ = diff.total_milliseconds();


    ping.channel_num=jsf_hdr.channel_num;

    if(jsf_hdr.channel_num==0){
        ping.port = ping_side;
    }
    else{
        ping.stbd = ping_side;
    }
  
    ping.first_in_file_ = false;
    return ping;

}

template <>
dvl_reading read_datagram<dvl_reading, jsf_dvl_msg_header>(std::ifstream& input,  jsf_dvl_msg_header& jsf_dvl_msg,  jsf_msg_header& jsf_hdr)
{
    dvl_reading reading;
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime data_time;

    data_time = epoch + boost::posix_time::seconds(jsf_dvl_msg.time_in_sec) + boost::posix_time::milliseconds(jsf_dvl_msg.ms_in_cur_sec);
    
    stringstream time_ss;
    time_ss << data_time;
    reading.time_string_ = time_ss.str();
    boost::posix_time::time_duration const diff = data_time - epoch;
    reading.time_stamp_ = diff.total_milliseconds();


    reading.vel_wrt_bottom_ = Eigen::Vector3d(jsf_dvl_msg.x_vel_wrt_bottom/1000., jsf_dvl_msg.y_vel_wrt_bottom/1000., jsf_dvl_msg.z_vel_wrt_bottom/1000.);
    reading.vel_wrt_water_ = Eigen::Vector3d(jsf_dvl_msg.x_vel_wrt_water/1000., jsf_dvl_msg.y_vel_wrt_water/1000., jsf_dvl_msg.z_vel_wrt_water/1000.);
    reading.dist_to_bottom_ = Eigen::Vector4d(jsf_dvl_msg.dist_to_bottom_in_cm[0]/100., jsf_dvl_msg.dist_to_bottom_in_cm[1]/100,jsf_dvl_msg.dist_to_bottom_in_cm[2]/100.,jsf_dvl_msg.dist_to_bottom_in_cm[3]/100.);
    reading.depth_ = jsf_dvl_msg.depth_in_dm/10.;
    reading.pitch_ = jsf_dvl_msg.pitch/100./180.*M_PI;
    reading.roll_ = jsf_dvl_msg.roll/100./180.*M_PI;
    reading.heading_ = jsf_dvl_msg.heading/100./180.*M_PI;
    reading.salinity_ = jsf_dvl_msg.salinity;
    reading.temp_ = jsf_dvl_msg.temp/100.;
    reading.sound_vel_ = jsf_dvl_msg.sound_vel;

    // initialize the flag
    reading.flag_["Vxy"] = false; // bit 0
    reading.flag_["Vz"] = false; // bit 2
    reading.flag_["Vxy_water"] = false; // bit 3
    reading.flag_["Vz_water"] = false; // bit 4
    reading.flag_["dist_bottom"] = false; // bit 5
    reading.flag_["heading"] = false; // bit 6
    reading.flag_["pitch"] = false; // bit 7
    reading.flag_["roll"] = false; // bit 8
    reading.flag_["temp"] = false; // bit 9
    reading.flag_["depth"] = false; // bit 10
    reading.flag_["salinity"] = false; // bit 11
    reading.flag_["sound_vel"] = false; // bit 12

    reading.ship_coord_ = false; // bit 1
    reading.error_ = false; // bit 31



    // bit 0
    if (jsf_dvl_msg.flag & 0x01) reading.flag_["Vxy"] = true;
     
    // bit 1
    if (jsf_dvl_msg.flag & 0x02) reading.ship_coord_ = true;
    

    // bit 2
    if (jsf_dvl_msg.flag & 0x04) reading.flag_["Vz"] = true;

    // bit 3
    if (jsf_dvl_msg.flag & 0x08) reading.flag_["Vxy_water"] = true;
    
    // bit 4
    if (jsf_dvl_msg.flag & 0x10) reading.flag_["Vz_water"] = true;

    // bit 5
    if (jsf_dvl_msg.flag & 0x20) reading.flag_["dist_bottom"] = true;
    
    // bit 6
    if (jsf_dvl_msg.flag & 0x40) reading.flag_["heading"] = true;
    
    // bit 7
    if (jsf_dvl_msg.flag & 0x80) reading.flag_["pitch"] = true;
    
    // bit 8
    if (jsf_dvl_msg.flag & 0x100) reading.flag_["roll"] = true;
    
    // bit 9
    if (jsf_dvl_msg.flag & 0x200) reading.flag_["temp"] = true;

    // bit 10
    if (jsf_dvl_msg.flag & 0x400) reading.flag_["depth"] = true;

    // bit 11
    if (jsf_dvl_msg.flag & 0x800) reading.flag_["salinity"] = true;

    // bit 12
    if (jsf_dvl_msg.flag & 0x1000) reading.flag_["sound_vel"] = true; 

    // bit 31
    if(jsf_dvl_msg.flag & 0x100000000){
        cout << "Error detected! flag in decimal: " << fixed << jsf_dvl_msg.flag << endl;
        reading.error_ = true;
    }

    return reading;

}


} // namespace jsf_data



namespace std_data{
using namespace jsf_data;

template <>
jsf_sss_ping::PingsT parse_file<jsf_sss_ping>(const boost::filesystem::path& file)
{
    jsf_sss_ping::PingsT pings = parse_file_impl<jsf_sss_ping, jsf_sonar_data_msg_header, 80>(file);
    jsf_sss_ping::PingsT fixed_pings;
    int start;
    if ((pings[0].time_stamp_ - pings[1].time_stamp_)<2) start = 0;
    else if ((pings[1].time_stamp_ - pings[2].time_stamp_)<2) start = 1;
    for (int i = start; i < pings.size()-1; i+=2){
        jsf_sss_ping fixed_ping;

        if ((pings[i].channel_num==0)&&(pings[i+1].channel_num==1)){
            fixed_ping = pings[i];
            fixed_ping.stbd = pings[i+1].stbd;
            fixed_pings.push_back(fixed_ping);
        }
        else if((pings[i].channel_num==1)&&(pings[i+1].channel_num==0)){
            fixed_ping = pings[i];
            fixed_ping.port = pings[i+1].port;
            fixed_pings.push_back(fixed_ping);
        }
        else {
            cout << "Invalid data format! channel numbers are: " << pings[i].channel_num << " and " << pings[i+1].channel_num << endl;
            break;
        }
    }
    return fixed_pings;
}

template <>
dvl_reading::Readings parse_file<dvl_reading>(const boost::filesystem::path& file)
{
    return parse_file_impl<dvl_reading, jsf_dvl_msg_header, 2080>(file);
}

} // namespace std_data