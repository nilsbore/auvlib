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
#include <cstdlib>
using namespace std;
using namespace std_data;
using namespace xtf_data;

/** call as:
./create_xtf_waterfall ./pathtoXTFfolder maxIntensity rowdownsample 
**/

extern int verbose_level;

int main(int argc, char** argv)
{

  
  //boost::filesystem::path folder("/home/nbore/Data/KTH_GBG_PING/Ping_Unprocessed/2-Raw_Data/SSS/xtf");
  boost::filesystem::path folder(argv[1]);
  //verbose_level=0;
    xtf_sss_ping::PingsT pings = parse_folder_ordered<xtf_sss_ping>(folder);
     long maxIntensity=65535;
    long rowdownsample=1;
    int chopup=0;
    if (argc>2) maxIntensity=atol(argv[2]);
    if (argc>3) rowdownsample=atol(argv[3]);

    long p_nadir[pings.size()];
    long s_nadir[pings.size()];
   long artif[pings.size()];
   std::cout<<folder<<" is parsed\n";
   if (0) 
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const xtf_sss_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        xtf_sss_ping::PingsT track_pings(pos, next);
	int r = track_pings.size()/rowdownsample;
     	long params[4];
	params[0]=1024;
	params[1]=r;
	params[2]=0;
	params[3]=0;
	cv::Mat norm_img= normalize_waterfall(track_pings,params);
	cv::imwrite("unnorm.png",norm_img);
	cv::imshow("myuncrrected", norm_img);
        cv::waitKey();
        pos = next;

    }
    long w=pings[0].port.pings.size()-100;
    // generally the minimum intensity param must be adjusted for differe bottoms and SSL is much brighter so also max intensity has to be like 500000 
    removeLineArtifact_port(pings, artif,10,120,0, 1000, .088, 3, .032);
    xtf_data::findNadirPort(pings,p_nadir,10,1000, 80);
    xtf_data::findNadirStbd(pings,s_nadir,10,1000, 80);
    if (0) //plot the artifact line bright or 0
    for (int i=0; i<pings.size(); i++){
      for (int k=0; k<100; k++){
	if (artif[i]<w)
	  pings[i].port.pings[artif[i]+k]=0;//100000;
      }
    }


    //    


    if(0)
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const xtf_sss_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        xtf_sss_ping::PingsT track_pings(pos, next);
	int r = track_pings.size()/rowdownsample;
	long params[4];
	params[0]=1024;
	params[1]=r;
	params[2]=0;
	params[3]=0;
	cv::Mat norm_img= normalize_waterfall(track_pings,params);
	cv::imwrite("norm.png",norm_img);
	cv::imshow("mynormed", norm_img);
        cv::waitKey();
        pos = next;
    }



    //plot the nadir in a bright wide line

    for (int i=0; i<pings.size(); i++){
      for (int k=0; k<40; k++){
	if (p_nadir[i]-k>0)
	  pings[i].port.pings[p_nadir[i]-k]=maxIntensity;
	if (s_nadir[i]-k>0)
	   pings[i].stbd.pings[s_nadir[i]-k]=maxIntensity;
      }
    }
    int n=1;
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const xtf_sss_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        xtf_sss_ping::PingsT track_pings(pos, next);
	std::cerr<<n<<"th file\n";
	n++;
	int r = track_pings.size()/rowdownsample;
	        cv::Mat waterfall_img = make_waterfall_image(track_pings, 1024, r, maxIntensity);
	cv::imwrite("notregularized.png",waterfall_img);
	cv::imshow("Before regularization image", waterfall_img);
        cv::waitKey();
        pos = next;
    }

    std::cout<<" Regularizing Now\n";
    n=1;
     xtf_data::regularize_pings(pings,p_nadir,s_nadir, (22/180.0*M_PI));

    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const xtf_sss_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        xtf_sss_ping::PingsT track_pings(pos, next);
	int r = track_pings.size()/rowdownsample;
	        cv::Mat waterfall_img = make_waterfall_image(track_pings, 1024, r, maxIntensity);
	std::cerr<<n<<"th file\n";
	n++;
	//cv::imwrite("regularized.png",waterfall_img);
	cv::imshow("My regularzed image", waterfall_img);
        cv::waitKey();
        pos = next;
    }
    return 0;
}

