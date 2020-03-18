/* Copyright 2019 john folkesson (johnf@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cereal/archives/json.hpp>
#include <data_tools/xtf_data.h>
#include <preprocess/sidescan.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
  boost::filesystem::path folder(argv[1]);
  preprocess::Sidescan sss(folder);
  std::cout<<folder<<" is parsed\n";
  preprocess::Sidescan temp;
  //pointless illustrations:
  //You can find an index for a  time value:
  Cure::Timestamp t(1787364382.2);
  long index=sss.find(t);
  //index is actually -1 now if that time in seconds since 1970 is not in sss
  //You can grap part of sss between two index values:
  sss.set(temp, 100,250);
  // (You can also do that with Cure::Timestamps.)
  //You can translate between the types with this:
  xtf_data::xtf_sss_ping::PingsT pings;
  sss.add(pings,200,400);
  std::cout<<"translated partial\n";
  temp=pings;
  std::cout<<"translated back\n";
  // pointless illustration section done.
  long len=0;
  std::cout<<&sss.side[1]<<" "<<len<<"\n";
  preprocess::SidescanSide *cs=sss.side[1].change(len);
  std::cout<<cs<<" "<<len<<"\n";
  while (cs){
    cs=cs->change(len);
    std::cout<<cs<<" "<<len<<"\n";
  }
  len=0;
  cs=sss.side[2].change(len);
  while (cs)cs=cs->change(len);
  std::cout<<cs<<" "<<len<<"\n";
  std::cout<<sss.side[1].h<<" "<<sss.side[2].h<<" "<<sss.h<<"\n ";
  long maxIntensity=65535;
  long rowdownsample=1;
  if (argc>2) maxIntensity=atol(argv[2]);
  if (argc>3) rowdownsample=atol(argv[3]);
  long p_nadir[sss[1].h];
  long s_nadir[sss[2].h];
  long artif[sss[1].h];
  // generally the minimum intensity param must be adjusted for differe bottoms and SSL or other factors
  double minintensity=10000;
  //remove the annoying line in the port and set to 0
  sss[1].removeLineArtifact(artif,minintensity,8,100,1);
  // set the nadir arrays 
  sss.findNadir(p_nadir,s_nadir, minintensity, 8, 80);
  //draw the nadir line found in the pings:  
  for (int i=0; i<sss.h; i++){
      for (int k=0; k<40; k++){
	if (p_nadir[i]-k>0)
	  sss[1][i][p_nadir[i]-k]=maxIntensity;
	if (s_nadir[i]-k>0)
	  sss[2][i][s_nadir[i]-k]=maxIntensity;
      }
  }


  sss[1].show_waterfall(1280,512,maxIntensity, 0, -1,
  		      "Port Before Normalization");

  sss[2].show_waterfall(1280, 512,maxIntensity, 0, -1,
  		      "Starboard Before Normalization");
 
  std::cout<<" Normalizing Now\n";
  // double roll[port.h];
   double roll[sss[1].h];
  double nadir_angle=(22.0*M_PI/180.0);
  memset(roll, 0, sizeof(roll));
  // use a tan of incidence angle to normalize images
  sss.normalize(p_nadir,s_nadir, 0, nadir_angle);
  
  sss[1].show_waterfall(1280, 512,maxIntensity, 0, -1,
			"Port After Normalization");
  sss[2].show_waterfall(1280, 512,maxIntensity, 0, -1,
			"Starboard After Normalization");
  
  std::cout<<" Regularizing Now\n";
  // set to a canonical representation with equal horizontal bin size of 5 cm
  sss.regularize(roll,p_nadir,s_nadir,0.05, .85, nadir_angle);
  
sss[1].show_waterfall(1280, 512,maxIntensity, 0, -1,
			"Port After Regularization");
  sss[2].show_waterfall(1280, 512,maxIntensity, 0, -1,
			"Starboard After Regularization");
    return 0;
}

