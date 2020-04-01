/* Copyright 2019 Yiping Xie (yipingx@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <fstream>
#include <iostream>
#include <cereal/archives/json.hpp>
#include <data_tools/jsf_data.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace jsf_data;
using namespace std_data;

jsf_sss_ping::PingsT test_parse_sss(const boost::filesystem::path& path)
{
    jsf_sss_ping::PingsT pings = parse_file<jsf_sss_ping>(path);
    int rows = pings.size();
    int cols = pings[0].port.pings.size() + pings[0].stbd.pings.size();

    for (const jsf_data::jsf_sss_ping& ping : pings) {
        // cout << "Ping duration:" << ping.port.time_duration << endl;
        // cout << "Ping pos: " << ping.pos_.transpose() << endl;
        // cout << "Ping rpy: " << ping.rpy.transpose() << endl;
    }

    // jsf_sss_ping::PingsT filtered_pings = filter_frequency(pings, 21269);
 
    printf("rows num: %d\n", rows);
    printf("cols num: %d\n", cols);
    // show_waterfall_image(filtered_pings);
    return pings;
}

jsf_dvl_ping::PingsT test_parse_dvl(const boost::filesystem::path& path)
{
    jsf_dvl_ping::PingsT pings = parse_file<jsf_dvl_ping>(path);
    cout << "Number of dvl pings: " << pings.size() << endl;
    for (auto i : pings) {
        if(i.error_)  cout << "Error occured, time string is: " << i.time_string_ << endl;
        
        bool all_false_flag=true;

        for (auto & k: i.flag_){
            if (k.second) all_false_flag = false;
        }
        if (all_false_flag) cout << "All flags are false, time string is: " << i.time_string_ << endl;
    
    }
    if (!pings.empty()) {
        cout << "Sound velocity from the first dvl data: " << pings[0].sound_vel_ << " m/s" << endl;
    }
    return pings;
}


void test_match_sound_vel(const boost::filesystem::path& path){
    jsf_sss_ping::PingsT sss_pings = parse_file<jsf_sss_ping>(path);
    jsf_dvl_ping::PingsT dvl_pings = parse_file<jsf_dvl_ping>(path);


    sss_pings = match_sound_vel(sss_pings, dvl_pings);

    for (auto i : sss_pings) {
        // if (i.sound_vel==1477)
        cout << "Sound velocity from the sss data at time_stamp: " << i.time_string_ << ", "<< i.sound_vel << " m/s" << endl;
    }
    for (auto i : dvl_pings) {
        // if (i.sound_vel_==1477)
    cout << "Sound velocity from the dvl data at time_stamp: " << i.time_string_ << ", "<<i.sound_vel_ << " m/s" << endl;
    }
    cout << "slant range: " << sss_pings[0].slant_ << endl;

}

int main(int argc, char** argv){
    int rtn=0;
    boost::filesystem::path path(argv[1]);

    jsf_dvl_ping::PingsT dvl_pings = test_parse_dvl(path);

    jsf_sss_ping::PingsT sss_pings = test_parse_sss(path);

    double time = sss_pings[0].sample_interval * sss_pings[0].port.pings.size();
    double sound_vel = dvl_pings[0].sound_vel_;
    double slant_range = sound_vel * time/2;
    cout << "time: " << time << endl;
    cout << "slant range: " << slant_range << endl;
    cout << "port time duration: " << sss_pings[0].port.time_duration << endl;
    cout << "stbd time duration: " << sss_pings[0].stbd.time_duration << endl;
    cout << "port.pings.size(): "<< sss_pings[0].port.pings.size()<<endl;
    cout << "dvl depth: " << dvl_pings[0].depth_ << endl;
    
    test_match_sound_vel(path);
    return rtn;
}
