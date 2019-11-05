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

// skip data other than sonar data (type=80)
void skip_data(ifstream& input, jsf_msg_header jsf_hdr){
    int nbr_bytes;
    for (int i = 0; i < jsf_hdr.following_bytes; i++){   
        input.read(reinterpret_cast<char*>(&nbr_bytes), sizeof(char));
    }
}


jsf_sss_ping_side process_side_scan_ping_side(ifstream& input, jsf_msg_header& jsf_hdr, jsf_sonar_data_msg_header& jsf_sonar_data_hdr){
    jsf_sss_ping_side ping_side;
    if(jsf_hdr.start_marker!=SONAR_MESSAGE_HEADER_START){
        cout<<"error start marker: "<<jsf_hdr.start_marker<<endl;
        exit(0);
    }
    if(jsf_hdr.msg_type!= SONAR_DATA_TYPE){
        cout<<"Error message type: "<<jsf_hdr.msg_type<<endl;
        exit(0);
    }

    if (jsf_sonar_data_hdr.data_format==0){
        int16_t env_data;
        float scaled_env_data[jsf_sonar_data_hdr.spls_num_in_pkt];
        for (int i=0; i<jsf_sonar_data_hdr.spls_num_in_pkt; ++i){

            input.read(reinterpret_cast<char*>(&env_data), sizeof(env_data));
            scaled_env_data[i] = ldexpf((float)env_data, -jsf_sonar_data_hdr.weighting_factor_n);
            ping_side.pings.push_back(scaled_env_data[i]);

        }
    }
    else if ((jsf_sonar_data_hdr.data_format==1)){
        cout<<"data format: " <<jsf_sonar_data_hdr.data_format<<endl<<"has real and imginary part, stored in pings and pings_phase respectively";
        int16_t analytic_sig_data[2];
        float scaled_sig_data_real[jsf_sonar_data_hdr.spls_num_in_pkt];
        float scaled_sig_data_img[jsf_sonar_data_hdr.spls_num_in_pkt];

        for (int i=0; i<jsf_sonar_data_hdr.spls_num_in_pkt; ++i){
            input.read(reinterpret_cast<char*>(&analytic_sig_data), sizeof(analytic_sig_data));
            scaled_sig_data_real[i] = ldexpf((float)analytic_sig_data[0], -jsf_sonar_data_hdr.weighting_factor_n);
            scaled_sig_data_img[i] = ldexpf((float)analytic_sig_data[1], -jsf_sonar_data_hdr.weighting_factor_n);

            ping_side.pings.push_back(scaled_sig_data_real[i]);
            ping_side.pings_phase.push_back(scaled_sig_data_img[i]);

            }
    }
    else{
        cout<<"skip data format: " <<jsf_sonar_data_hdr.data_format<<endl;
        for (int i = 0; i < jsf_hdr.following_bytes-sizeof(jsf_sonar_data_hdr); i++){   
            int16_t nbr_bytes;
            input.read(reinterpret_cast<char*>(&nbr_bytes), sizeof(char));
        }
    }
    return ping_side;
 }




jsf_sss_ping process_side_scan_ping(ifstream& input, jsf_msg_header& jsf_hdr, jsf_sonar_data_msg_header& jsf_sonar_data_hdr){

    jsf_sss_ping ping;
    input.read(reinterpret_cast<char*>(&jsf_sonar_data_hdr),sizeof(jsf_sonar_data_hdr));
    jsf_sss_ping_side ping_side_cur;
    ping_side_cur = process_side_scan_ping_side(input, jsf_hdr, jsf_sonar_data_hdr);

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

 
    jsf_msg_header jsf_hdr_next;
    input.read(reinterpret_cast<char*>(&jsf_hdr_next),sizeof(jsf_hdr_next));

    while((jsf_hdr_next.msg_type!=SONAR_DATA_TYPE)&&(!input.eof())){
        skip_data(input, jsf_hdr_next);
        input.read(reinterpret_cast<char*>(&jsf_hdr_next),sizeof(jsf_hdr_next));

        if (jsf_hdr_next.start_marker != SONAR_MESSAGE_HEADER_START){
            cout << "Invalid file format! start marker: "<<jsf_hdr_next.start_marker << endl;
            exit(0);
        }
    }
    
    // continue to read next channel
    jsf_sonar_data_msg_header jsf_sonar_data_hdr_next;    
    input.read(reinterpret_cast<char*>(&jsf_sonar_data_hdr_next),sizeof(jsf_sonar_data_hdr_next));
    jsf_sss_ping_side ping_side_nxt;
    if (jsf_hdr_next.start_marker == SONAR_MESSAGE_HEADER_START){
        ping_side_nxt = process_side_scan_ping_side(input, jsf_hdr_next, jsf_sonar_data_hdr_next);
    }  

        if(jsf_hdr.channel_num==0){
            ping.port = ping_side_cur;
            ping.stbd = ping_side_nxt;
        }
        else{
            ping.stbd = ping_side_cur;
            ping.port = ping_side_nxt;
        }
  

    // determine if it is a bad ping 
    if(((jsf_hdr.channel_num+jsf_hdr_next.channel_num)!=1)||((jsf_sonar_data_hdr.ping_time_in_sec!=jsf_sonar_data_hdr_next.ping_time_in_sec))){
        ping.is_bad = true;
    }
    else{
        ping.is_bad = false;
    }
    ping.first_in_file_ = false;
    return ping;

}





jsf_sss_ping::PingsT read_jsf_file(ifstream& input, jsf_msg_header& jsf_hdr){
    jsf_sss_ping::PingsT pings;
    jsf_sonar_data_msg_header jsf_sonar_data_hdr;
    while (!input.eof()){
        input.read(reinterpret_cast<char*>(&jsf_hdr),sizeof(jsf_hdr));
        if (jsf_hdr.start_marker != SONAR_MESSAGE_HEADER_START){
            cout << "Invalid file format! start marker: "<<jsf_hdr.start_marker << endl;
            break;
        }

        if (jsf_hdr.msg_type==SONAR_DATA_TYPE){
            jsf_sss_ping ping= process_side_scan_ping(input, jsf_hdr, jsf_sonar_data_hdr);

            if (ping.is_bad){
                cout << pings.size()+1 <<"th ping is a bad ping, ignore!"<<endl;
            }
            else{
                pings.push_back(ping);
            }
            
        }
        else{
            skip_data(input,jsf_hdr);
        }
    }
    cout<<"Read Done!"<<endl;

    if (!pings.empty()) {
        pings[0].first_in_file_ = true;
    }
    return pings;

}


jsf_sss_ping::PingsT parse_file_impl(const boost::filesystem::path& path){
    jsf_sss_ping::PingsT pings;
    if (boost::filesystem::extension(path) != ".JSF") {
        cout << "Not an .JSF file, skipping..." << endl;
        cout << "Extension: " << boost::filesystem::extension(path) << endl;
        return pings;
    }

    ifstream input;
    input.open(path.string(), ios::binary);
    if (input.fail()) {
        cout << "ERROR: Cannot open the file..." << endl;
        exit(0);
        return pings;
    }

    jsf_msg_header jsf_hdr;

    pings = read_jsf_file(input, jsf_hdr);
    cout<<"pings size is: "<<pings.size()<<endl;
    return pings;
    

}

} // namespace jsf_data



namespace std_data{
using namespace jsf_data;

jsf_sss_ping::PingsT parse_file(const boost::filesystem::path& file)
{
    return parse_file_impl(file);
}



} // namespace std_data