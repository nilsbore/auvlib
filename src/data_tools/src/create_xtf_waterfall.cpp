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
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cereal/archives/json.hpp>
#include <data_tools/xtf_data.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace std_data;
using namespace xtf_data;

/** call as:
./create_xtf_waterfall ./pathtoXTFfolder maxIntensity rowdownsample 
**/

int main(int argc, char** argv)
{

  
  //boost::filesystem::path folder("/home/nbore/Data/KTH_GBG_PING/Ping_Unprocessed/2-Raw_Data/SSS/xtf");
  boost::filesystem::path folder(argv[1]);
    
    xtf_sss_ping::PingsT pings = parse_folder<xtf_sss_ping>(folder);
    long maxIntensity=65535;
    long rowdownsample=1;
 
    if (argc>2) maxIntensity=atol(argv[2]);
    if (argc>3) rowdownsample=atol(argv[3]);
		  
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const xtf_sss_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        xtf_sss_ping::PingsT track_pings(pos, next);
	int r = track_pings.size()/rowdownsample;
        cv::Mat waterfall_img = make_waterfall_image(track_pings, 512, r, maxIntensity);
	long params[4];
	params[0]=512;
	params[1]=r;
	params[2]=0;
	params[3]=0;
	cv::Mat norm_img= normalize_waterfall(track_pings,params);
	int rows=waterfall_img.rows;
	int cols=waterfall_img.cols;
	cv::imwrite("a.png",waterfall_img);
	cv::imshow("My image", waterfall_img);
	cv::imwrite("norm.png",norm_img);
	cv::imshow("mynormed", norm_img);
        cv::waitKey();
        pos = next;
    }
    return 0;
}

