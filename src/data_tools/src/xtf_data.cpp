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

#include <data_tools/xtf_data.h>
extern "C" {
#include <libxtf/xtf_reader.h>
}
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

//#include <stdio.h>
#include <fcntl.h>
#ifdef _MSC_VER
  #include <io.h>
#else
  #include <unistd.h>
#endif
//#include <stdlib.h>
//#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <data_tools/lat_long_utm.h>

using namespace std;

namespace xtf_data {

  void findNadirPort(xtf_sss_ping::PingsT& pings, long * nadir, double minalt, long minintensityatnadir){
    double slantr=pings[0].port.slant_range;
    double res=slantr/(double)pings[0].port.pings.size();//m per bin
    double f=.995*.008/res; 
    int w=pings[0].port.pings.size();
    int maw=(int)((double)(.16/res+.5));
    int minj=(int)((double)(minalt/res+.5));
    if (maw<2)maw=2;
    if (minj<10)minj=10;
    //remove any funny stuff below min alt and check if scan is no good
    for (int i = 0; i < pings.size(); i++) {
      for (int j = 0; j <w; j++){
	if ((pings[i].port.pings[j]<0)||(pings[i].port.pings[j]>(1<<29)))
	  if (i>0)pings[i].port.pings[j]=pings[i-1].port.pings[j];
	  else pings[i].port.pings[j]=0;
      }
      for (int j = 0; j <minj; j++) {
	  pings[i].port.pings[j]=0;
      }
    }
    
    for (int i = 0; i < pings.size(); i++) {
      nadir[i]=0;
      long max=0;
      long temp=0;
      for (int j = 0; j < w-maw-1; j++) {
	for (int k=j;k<j+maw; k++)
	  temp+=pings[i].port.pings[k];
	temp/=(maw);
	if (max<temp)
	  if (max< minintensityatnadir) max=temp;
	  else {
	    if (max*2<temp){
	      max=temp;
	      nadir[i]=j;
	      j=w;
	    }
	  }
	max*=f;
	max+=(1-f)*temp;
      }
    }

    //nadir is max of 3 values 
    long lastn=nadir[0];
    long long avg=lastn;
    long long min=lastn;
    long long max=lastn;
    for(int i=2; i< pings.size(); i++){
      long temp=nadir[i-1];
      if (nadir[i-1]<lastn)
	nadir[i-1]=lastn;
      if (nadir[i-1]<nadir[i])
      	nadir[i-1]=nadir[i];
      lastn=temp;
      avg+=lastn;
      if (min>lastn)min=lastn;
      if (max<lastn)max=lastn;
    }
    if (nadir[pings.size()-1]<lastn)
      nadir[pings.size()-1]=lastn;
    lastn=nadir[pings.size()-1];
    avg+=lastn;
    if (min>lastn)min=lastn;
    if (max<lastn)max=lastn;
    avg/=pings.size();
    double nad=avg*res;
    std::cout<<" Average Port Nadir is "<<nad<<" m and range is ("<<(min*res)<<", "<<(max*res)<<")\n";
  }
void findNadirStbd(xtf_sss_ping::PingsT& pings, long * nadir, double minalt, long minintensityatnadir){
    double slantr=pings[0].stbd.slant_range;
    double res=slantr/(double)pings[0].stbd.pings.size();//m per bin
    double f=.995*.008/res; 
    int w=pings[0].stbd.pings.size();
    int maw=(int)((double)(.16/res+.5));
    int minj=(int)((double)(minalt/res+.5));
    if (maw<2)maw=2;
  	
    //remove any funny stuff below min alt and check if scan is no good
    for (int i = 0; i < pings.size(); i++) {
      for (int j = 0; j <w; j++){
	if ((pings[i].stbd.pings[j]<0)||(pings[i].stbd.pings[j]>(1<<29)))
	  if (i>0)pings[i].stbd.pings[j]=pings[i-1].stbd.pings[j];
	  else pings[i].stbd.pings[j]=0;
      }
      for (int j = 0; j <minj; j++) {
	  pings[i].stbd.pings[j]=0;
      }
    }
    
    for (int i = 0; i < pings.size(); i++) {
      nadir[i]=0;
      long max=0;
      long temp=0;
      for (int j = 0; j < w-maw-1; j++) {
	for (int k=j;k<j+maw; k++)
	  temp+=pings[i].stbd.pings[k];
	temp/=(maw);
	if (max<temp)
	  if (max< minintensityatnadir) max=temp;
	  else {
	    if (max*2<temp){
	      max=temp;
	      nadir[i]=j;
	      j=w;
	    }
	  }
	max*=f;
	max+=(1-f)*temp;
      }
    }

    //nadir is max of 3 values 
    long lastn=nadir[0];
    long long avg=lastn;
    long long min=lastn;
    long long max=lastn;
    for(int i=2; i< pings.size(); i++){
      long temp=nadir[i-1];
      if (nadir[i-1]<lastn)
	nadir[i-1]=lastn;
      if (nadir[i-1]<nadir[i])
      	nadir[i-1]=nadir[i];
      lastn=temp;
      avg+=lastn;
      if (min>lastn)min=lastn;
      if (max<lastn)max=lastn;
    }
    if (nadir[pings.size()-1]<lastn)
      nadir[pings.size()-1]=lastn;
    lastn=nadir[pings.size()-1];
       avg+=lastn;
    if (min>lastn)min=lastn;
    if (max<lastn)max=lastn;
    avg/=pings.size();
    double nad=avg*res;
    std::cout<<" Average Starboard Nadir is "<<nad<<" m and range is ("<<(res*min)<<", "<<(res*max)<<")\n";


  }

cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings)
{
    int rows = pings.size();
    int cols = pings[0].port.pings.size() + pings[0].stbd.pings.size();
    cv::Mat swath_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < pings.size(); ++i) {
        for (int j = 0; j < pings[i].port.pings.size(); ++j) {
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, pings[0].stbd.pings.size()+j);
            p->z = uchar(255.*(float(pings[i].port.pings[j]) + 32767.)/(2.*32767.));
            p->y = uchar(255.*(float(pings[i].port.pings[j]) + 32767.)/(2.*32767.));
            p->x = uchar(255.*(float(pings[i].port.pings[j]) + 32767.)/(2.*32767.));
        }
        for (int j = 0; j < pings[i].stbd.pings.size(); ++j) {
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, pings[0].stbd.pings.size()-j-1);
            //tie(p->z, p->y, p->x) = uchar(255.*(float(pings[i].port_pings[j]) + 32767.)/(2.*65536));
            p->z = uchar(255.*(float(pings[i].stbd.pings[j]) + 32767.)/(2.*32767.));
            p->y = uchar(255.*(float(pings[i].stbd.pings[j]) + 32767.)/(2.*32767.));
            p->x = uchar(255.*(float(pings[i].stbd.pings[j]) + 32767.)/(2.*32767.));
        }
    }
    cv::Mat resized_swath_img;//dst image
    cv::resize(swath_img, resized_swath_img, cv::Size(cols/8, rows/8));//resize image
    
    return resized_swath_img;
}

void  regularize_pings(xtf_sss_ping::PingsT& pings, const long * port_nadir, const long * stbd_nadir, double nadir_angle)
{
    //res is m per bin
  double res=(double)pings[0].port.slant_range/(double)pings[0].port.pings.size();
  double b=cos(nadir_angle)/2.0;
  
  int w=pings[0].port.pings.size();    
  for (int i = 0; i < pings.size(); i++) {
    double c=port_nadir[i];
    if (c==0){
      c=2.0*stbd_nadir[i];
    }  else if (stbd_nadir[i]==0)c*=2.0;
    else  c+=stbd_nadir[i];
    if (c>1E-6){
      double a=b*c;
      for (int j = port_nadir[i]; j < w; j++) {
	c=a/(double)j;
	if (c>1)c=1;
	if (c<.01) c=.01;
	double phi=acos(c);
	double intensity=pings[i].port.pings[j];
	intensity*=sin(phi);
	intensity/=c;
	pings[i].port.pings[j]=(long)(intensity+.5);
      }

      for (int j = stbd_nadir[i]; j < w; j++) {
	c=a/(double)j;
	if (c>1)c=1;
	if (c<.01) c=.01;
	double phi=acos(c);
	double intensity=pings[i].stbd.pings[j];
	intensity*=sin(phi);
	intensity/=c;
	pings[i].stbd.pings[j]=(long)(intensity+.5);
      }
    }
  }
}
cv::Mat  normalize_waterfall(const xtf_sss_ping::PingsT& pings, long* params)
{
  int rows = pings.size();
  long width=params[0];
  long height=params[1];
  long startping=params[2];
  long endping=params[3];
  if (startping<0) startping=0;
  if ((endping>rows)||(endping<=0))endping=rows;
  rows=endping-startping;
  
  int cols = pings[0].port.pings.size() + pings[0].stbd.pings.size();     
    int downsample=1;
    if (width>0) downsample=cols/width;
    else width=cols;
    if (downsample==0){
      width=cols;
      downsample=1;
    }

    int downrow=1;
    if (height>0)downrow=rows/height;
    else height=rows;
    if (downrow==0) {
      height=rows;
      downrow=1;
    }
    long maxPingIntensity[width];
    long minPingIntensity[width];
    std::memset(maxPingIntensity, 0, sizeof maxPingIntensity);
    std::memset(minPingIntensity, 256, sizeof minPingIntensity);
    int num=0;
    long long acc=0;
    int mid=width/2;
    
    if ((height*rows)<=0){
      return cv::Mat(0, 0, CV_8UC1, cv::Scalar(0));
    }
    cv::Mat swath_img = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    int toprow= pings.size()-downrow+1;
    long m[height*width];
    
    for(int i=0; i< height; i++){
      for (int j=mid; j<width; j++){

	acc=0;
	num=(downrow*downsample);
	for (int  w=0; w<downrow; w++){
	  int over=pings[startping+i*downrow+w].port.pings.size();
	  int index=(j-mid)*downsample;
	  for (int k=0; k<downsample; k++)
	    {
	      long temp=pings[startping+i*downrow+w].port.pings[index];
	      if ((temp>=0)&&(index<over)) acc+=temp;
	      else num--;
	      //	      if ((i==30)&&(j>344)&&(j<348))
	      //cout<<temp<<" i "<<i<<" j "<<j<<" w "<<w<<" k "<<k<<"\n";
	      index++;
	    }
	}
	
	if (num>0){
	  m[j*height+i]=(long)(((double)acc/(double)num)+0.5);
	  if (m[j*height+i]>maxPingIntensity[j]){
	    maxPingIntensity[j]=m[j*height+i];
	  }
	  if (m[j*height+i]<minPingIntensity[j])
	    minPingIntensity[j]=m[j*height+i];
	}
	else {
	  m[j*height+i]=0;
	}
      }
      for (int j=0; j<mid; j++){
	acc=0;
	num=(downrow*downsample);
	for ( int w=0; w<downrow; w++){
	  int over=pings[startping+i*downrow+w].stbd.pings.size();
	  int index=(mid-j)*downsample-1;
	  for (int k=0; k<downsample; k++)
	    {
	      long temp=pings[startping+i*downrow+w].stbd.pings[index];
	      
		//weird values in some pings of =-2147483648=-2^31=FFFFFFFF
	      if ((temp>=0)&&(index<over)) acc+=temp;
	      else num--;
	      index--;
	    }
	  
	}
	if (num>0){
	  m[j*height+i]=(long)((double)acc/(double)num+0.5);
	  if (m[j*height+i]>maxPingIntensity[j])
	    maxPingIntensity[j]=m[j*height+i];
	  if (m[j*height+i]<minPingIntensity[j])
	    minPingIntensity[j]=m[j*height+i];
	}
	else m[j*height+i]=0;	
      }
    }
    long maxmax=0;
    for (int j=0; j<width;j++){
      maxPingIntensity[j]*=1.05;//to avoid saturation
      if (maxPingIntensity[j]>maxmax) maxmax=maxPingIntensity[j];
      if (minPingIntensity[j]>10)minPingIntensity[j]=10;
    }
    for (int j=0; j<width;j++)
      if (maxPingIntensity[j]<maxmax/10)maxPingIntensity[j]=maxmax/10;
    
    if (width>2){//smooth the normalization values

      long long temp=maxPingIntensity[0]+maxPingIntensity[1];
      long long temp2=minPingIntensity[0]+minPingIntensity[1];
      maxPingIntensity[0]=temp/2;
      temp+=maxPingIntensity[2];
      maxPingIntensity[1]=temp/3;
      maxPingIntensity[0]=temp2/2;
      temp2+=minPingIntensity[2];
      minPingIntensity[1]=temp2/3;
      for (int j=2; j<width;j++){
	temp-=maxPingIntensity[j-2];
	temp2-=minPingIntensity[j-2];
	temp+=maxPingIntensity[j];
	temp2+=minPingIntensity[j];
	minPingIntensity[j]=temp2/3;
	maxPingIntensity[j]=temp/3;
      }
      temp=maxPingIntensity[0]+maxPingIntensity[1];
      temp2=minPingIntensity[0]+minPingIntensity[1];
      maxPingIntensity[0]=temp/2;
      temp+=maxPingIntensity[2];
      maxPingIntensity[1]=temp/3;
      maxPingIntensity[0]=temp2/2;
      temp2+=minPingIntensity[2];
      minPingIntensity[1]=temp2/3;
      for (int j=2; j<width;j++){
	temp-=maxPingIntensity[j-2];
	temp2-=minPingIntensity[j-2];
	temp+=maxPingIntensity[j];
	temp2+=minPingIntensity[j];
	minPingIntensity[j]=temp2/3;
	maxPingIntensity[j]=temp/3;
      }
    }
    
    for (int j=0; j<width;j++){
      for (int i=0; i<height; i++){
	unsigned short temp=(unsigned short)(255*((double)(m[i+j*height]-minPingIntensity[j])/(double)(maxPingIntensity[j]-minPingIntensity[j]))+0.5);
	cv::Scalar_<uchar>* p = swath_img.ptr<cv::Scalar_<uchar> >(i,j);
	if (temp>255){
	  temp=255;
	}	
	p[0]=(uchar)temp;	
      }
    }
    return swath_img;
  }

  cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings, long width, long height, long maxPingIntensity, long minPingIntensity)
  {
    if (maxPingIntensity<1)maxPingIntensity=65535;
    if (maxPingIntensity<minPingIntensity)minPingIntensity=0;
    int rows = pings.size();
    int cols = pings[0].port.pings.size() + pings[0].stbd.pings.size();     
    int downsample=1;
    if (width>0) downsample=cols/width;
    else width=cols;
    if (downsample==0){
      width=cols;
      downsample=1;
    }
    uchar val=0;
    int downrow=1;
    if (height>0)downrow=rows/height;
    else height=rows;
    if (downrow==0) {
      height=rows;
      downrow=1;
    }
    int count=0;
    int num=0;
    long long acc=0;
    int mid=width/2;
    cv::Mat swath_img = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    int toprow= pings.size()-downrow+1;
    for(int i=0; i< height; i++){
      
      for (int j=mid; j<width; j++){

	acc=0;
	num=(downrow*downsample);
	for (int  w=0; w<downrow; w++){
	  int over=pings[i*downrow+w].port.pings.size();
	  int index=(j-mid)*downsample;
	  for (int k=0; k<downsample; k++)
	    {
	      long temp=pings[i*downrow+w].port.pings[index];
	      if (temp>maxPingIntensity){
		count++;
		temp=maxPingIntensity;
	      }
	      if (temp>=minPingIntensity)temp=temp-minPingIntensity;
	      else temp=0;
	      //weird values in some pings of =-2147483648=-2^31=FFFFFFFF
	      if ((temp>=0)&&(index<over)) acc+=temp;
	      else num--;
	      index++;
	    }
	}
	if (num>0)val=(255*((double)acc/(double)num)/(double(maxPingIntensity-minPingIntensity)));
	else val=0;
	cv::Scalar_<uchar>* p = swath_img.ptr<cv::Scalar_<uchar> >(i,j);
	p[0]=val;
      }
      for (int j=0; j<mid; j++){
	acc=0;
	num=(downrow*downsample);
	for ( int w=0; w<downrow; w++){
	  int over=pings[i*downrow+w].stbd.pings.size();
	  int index=(mid-j)*downsample-1;
	  for (int k=0; k<downsample; k++)
	    {
	      long temp=pings[i*downrow+w].stbd.pings[index];
	      
	      if (temp>maxPingIntensity){
		count++;
		temp=maxPingIntensity;
	      }
	      if (temp>=minPingIntensity)temp=temp-minPingIntensity;
	      else temp=0;
		//weird values in some pings of =-2147483648=-2^31=FFFFFFFF
	      if ((temp>=0)&&(index<over)) acc+=temp;
	      else num--;
	      index--;
	    }
	  
	}
	
	if (num>0)val=(255*((double)acc/(double)num)/(double(maxPingIntensity-minPingIntensity)));
	else val=0;
	cv::Scalar_<uchar>* p = swath_img.ptr<cv::Scalar_<uchar> >(i,j);
	p[0]=val;
     
      }
    }
    double p=count*100.0/(rows*cols);
    std::cout<<"There were "<<count<<" or "<<p<<"% above the maxPingIndensity of "<<maxPingIntensity<<"\n";
    return swath_img;
  }
/**
 RemoveLineArtifact_port(xtf_sss_ping::PingsT& pings, long * nadir, double minArtifactRange minr=30,  double minArtifactRange maxr=90, bool setzero=false)

Our Port side Sidescan has an artifact that appears as a bright spot in about 25 cm of bins.  Which bins varies continously and smoothly in time accross bins.
You can give some hints as to the range that the artifact wanders over and choose to set the values to 0 instead of trying to fill them with 'average' values nearby plus noise.

nadir -  the array returned from calling findNadirPort. This array will be changed to contain the detected bin of the artifact.  

 **/


void removeLineArtifact_port(xtf_sss_ping::PingsT& pings, long * nadir, const double minArtifactRange,  const double maxArtifactRange, const bool setzero){
    //res is m per bin
    double res=(double)pings[0].port.slant_range/(double)pings[0].port.pings.size();
    //This is the low pass filter applied as we move along the elements of a ping.  
    double f=.995*.008/res; 
    int jstart=(minArtifactRange/res);
    int jend=(maxArtifactRange/res+.5);
    int w=pings[0].port.pings.size();
    if (jend>w)jend=w;
    int jw=jend-jstart;
    //this is the moveing avgerage widow size as we move along the ping elements
    int maw=(int)((double)(.04/res+.5));
    if (maw<2)maw=2;
    int howmany=0;
    long artifact=0;   
    double running=jstart;
    int countr=0;
    int countfails=0;
    //This sets how far the artifact can wander between pings.
    double binmotion=10.0/res;
    // this si the start value for the change in the moving average that is called a detection of the artifact.  It will be changed adaptively and stubornly to find all artifacts.
    double testthreshold=3.5;
    for (int i = 0; i < pings.size(); i++) { 
      int jn=nadir[i];
      nadir[i]=0;
      double max=0;
      long temp=0;
      double mv=0;
      
      double tt=testthreshold;
      while (nadir[i]==0){
	for (int j = jn; j < jend-maw-1; j++) {
	  for (int k=j;k<j+maw; k++)
	    temp+=pings[i].port.pings[k];
	  temp/=(maw);
	  if (j==jn) max=temp;	      
	  if (max<temp)
	    if ((j>jstart)&&(max*tt<(double)temp)){
	      double bn=binmotion;
	       bn*=exp(2.0*countfails);
	      if (countr<10) bn*=(10-countr);
	      mv=j-running;	     
	      if  ((countr==0)||((mv<bn)&&(mv>-bn))){
		nadir[i]=j;
		j=w;
		artifact=temp;
	      }
	      else max=temp;
	    }
	  max*=f;
	  max+=((double)temp*(1.0-f));
	}
	if (nadir[i]==0){
	  tt*=.9;
	}
	if (tt<2) break;
      }
      
      if (nadir[i]>0){
	countr++;
	countfails=0;
	if (countr==1) running =nadir[i];
	else    running=.9*running+.1*nadir[i];
	
	
       
	int top =nadir[i]+50+maw;
	if (top>w) top=w;
	
	double avg=0;
	if (!setzero){
	  int sta=top+1/res;
	  if (sta>w) sta=w-50;
	  for (int k=sta; k<sta+50; k++)
	    avg+=pings[i].port.pings[k];
	  sta=nadir[i]-100-1/res;
	  if (sta<0) sta=0;
	  for (int k=sta; k<sta+50; k++)
	    avg+=pings[i].port.pings[k];
	  avg/=100;
	}
	double r1=(2.0*(double)rand())/((double)RAND_MAX);
	for (int k=nadir[i]-50;k<top; k++){
	  if(pings[i].port.pings[k]>artifact/(testthreshold-0.5)){
	    howmany++;
	    double r=((double)rand())/(1.0*(double)RAND_MAX) +.5;
	    pings[i].port.pings[k]=avg*r*r1;
	  }
	}
	
      } else countfails++;
    }
    howmany/=countr;
    std::cout<<"Corrected "<<countr<<" of "<<pings.size()<< " pings with an average of "<<howmany<<" bins (= "<<(howmany*res)<<"m) in each ping"<<"\n";   
}

/**
 RemoveLineArtifact_stbd(xtf_sss_ping::PingsT& pings, long * nadir, double minArtifactRange minr=30,  double minArtifactRange maxr=90, bool setzero=false)

Our Stbd side Sidescan has an artifact that appears as a bright spot in about 25 cm of bins.  Which bins varies continously and smoothly in time accross bins.
You can give some hints as to the range that the artifact wanders over and choose to set the values to 0 instead of trying to fill them with 'average' values nearby plus noise.

nadir -  the array returned from calling findNadirStbd. This array will be changed to contain the detected bin of the artifact.  

 **/
void removeLineArtifact_stbd(xtf_sss_ping::PingsT& pings, long * nadir, const double minArtifactRange,  const double maxArtifactRange, const bool setzero){
    //res is m per bin
    double res=(double)pings[0].stbd.slant_range/(double)pings[0].stbd.pings.size();
    //This is the low pass filter applied as we move along the elements of a ping.  
    double f=.995*.008/res; 
    int jstart=(minArtifactRange/res);
    int jend=(maxArtifactRange/res+.5);
    int w=pings[0].stbd.pings.size();
    if (jend>w)jend=w;
    int jw=jend-jstart;
    //this is the moveing avgerage widow size as we move along the ping elements
    int maw=(int)((double)(.04/res+.5));
    if (maw<2)maw=2;
    int howmany=0;
    long artifact=0;   
    double running=jstart;
    int countr=0;
    int countfails=0;
    //This sets how far the artifact can wander between pings.
    double binmotion=10.0/res;
    // this si the start value for the change in the moving average that is called a detection of the artifact.  It will be changed adaptively and stubornly to find all artifacts.
    double testthreshold=3.5;
    for (int i = 0; i < pings.size(); i++) { 
      int jn=nadir[i];
      nadir[i]=0;
      double max=0;
      long temp=0;
      double mv=0;
      
      double tt=testthreshold;
      while (nadir[i]==0){
	for (int j = jn; j < jend-maw-1; j++) {
	  for (int k=j;k<j+maw; k++)
	    temp+=pings[i].stbd.pings[k];
	  temp/=(maw);
	  if (j==jn) max=temp;	      
	  if (max<temp)
	    if ((j>jstart)&&(max*tt<(double)temp)){
	      double bn=binmotion;
	       bn*=exp(2.0*countfails);
	      if (countr<10) bn*=(10-countr);
	      mv=j-running;	     
	      if  ((countr==0)||((mv<bn)&&(mv>-bn))){
		nadir[i]=j;
		j=w;
		artifact=temp;
	      }
	      else max=temp;
	    }
	  max*=f;
	  max+=((double)temp*(1.0-f));
	}
	if (nadir[i]==0){
	  tt*=.9;
	}
	if (tt<2) break;
      }
      
      if (nadir[i]>0){
	countr++;
	countfails=0;
	if (countr==1) running =nadir[i];
	else    running=.9*running+.1*nadir[i];
	
	
       
	int top =nadir[i]+50+maw;
	if (top>w) top=w;
	
	double avg=0;
	if (!setzero){
	  int sta=top+1/res;
	  if (sta>w) sta=w-50;
	  for (int k=sta; k<sta+50; k++)
	    avg+=pings[i].stbd.pings[k];
	  sta=nadir[i]-100-1/res;
	  if (sta<0) sta=0;
	  for (int k=sta; k<sta+50; k++)
	    avg+=pings[i].stbd.pings[k];
	  avg/=100;
	}
	double r1=(2.0*(double)rand())/((double)RAND_MAX);
	for (int k=nadir[i]-50;k<top; k++){
	  if(pings[i].stbd.pings[k]>artifact/(testthreshold-0.5)){
	    howmany++;
	    double r=((double)rand())/(1.0*(double)RAND_MAX) +.5;
	    pings[i].stbd.pings[k]=avg*r*r1;
	  }
	}
	
      } else countfails++;
    }
    howmany/=countr;
    std::cout<<"Corrected "<<countr<<" of "<<pings.size()<< " pings with an average of "<<howmany<<" bins (= "<<(howmany*res)<<"m) in each ping"<<"\n";   
}



Eigen::MatrixXd make_eigen_waterfall_image(const xtf_sss_ping::PingsT& pings)
{
    int rows = pings.size();
    int cols = pings[0].port.pings.size() + pings[0].stbd.pings.size();
    //cv::Mat swath_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    Eigen::MatrixXd swath_img = Eigen::MatrixXd::Zero(rows, cols);
    for (int i = 0; i < pings.size(); ++i) {
        for (int j = 0; j < pings[i].port.pings.size(); ++j) {
            //swath_img(i, pings[0].stbd.pings.size()+j) = (float(pings[i].port.pings[j]) + 32767.)/(2.*32767.);
            swath_img(i, pings[0].stbd.pings.size()+j) = double(pings[i].port.pings[j] + 4000.)/20000.;
        }
        for (int j = 0; j < pings[i].stbd.pings.size(); ++j) {
            //swath_img(i, pings[0].stbd.pings.size()-j-1) = (float(pings[i].stbd.pings[j]) + 32767.)/(2.*32767.);
            swath_img(i, pings[0].stbd.pings.size()-j-1) = double(pings[i].stbd.pings[j] + 4000.)/20000.;
        }
    }
    return swath_img;
}

void show_waterfall_image(const xtf_sss_ping::PingsT& pings)
{
    cv::Mat waterfall_image = make_waterfall_image(pings);
    cv::imshow("Waterfall image", waterfall_image);
    cv::waitKey();
}

xtf_sss_ping process_side_scan_ping(XTFPINGHEADER *PingHeader, XTFFILEHEADER *XTFFileHeader) {
/****************************************************************************/
// Put whatever processing here to be performed on SIDESCAN data.
// PingHeader points to the 256-byte ping header structure.  That structure
// identifies how many channels of sidescan follow.  The structure is followed
// by the sidescan data itself.
//
// For example: assume there are two channels of sidescan, stored 1024
// 8-bit samples per channel.  The data pointed to by PingHeader looks like:
// 
// 256 bytes   - XTFPINGHEADER structure holds data about this ping
// 64 bytes    - XTFPINGCHANHEADER structure holds data about channel 1 (port)
// 1024 bytes  - channel 1 imagery
// 64 bytes    - XTFPINGCHANHEADER structure holds data about channel 2 (stbd)
// 1024 bytes  - channel 2 imagery
//
   const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
   WORD chan;
   int tmp;
   unsigned char *Ptr = (unsigned char *)PingHeader;

   // For backwards-compatibility.  The samples per channel used to
   // be stored in the file header.  


   // skip past the ping header
   Ptr += sizeof(XTFPINGHEADER);

   xtf_sss_ping ping;
   ping.lat_ = PingHeader->SensorYcoordinate;
   ping.long_ = PingHeader->SensorXcoordinate;

   double easting, northing;
   string utm_zone;
   tie(northing, easting, utm_zone) = lat_long_utm::lat_long_to_UTM(ping.lat_, ping.long_);
   ping.pos_ = Eigen::Vector3d(easting, northing, -PingHeader->SensorDepth);
   ping.heading_ = M_PI/180.*PingHeader->SensorHeading;
   ping.heading_ = 0.5*M_PI-ping.heading_; // TODO: need to keep this for old data
   ping.sound_vel_ = PingHeader->SoundVelocity;

   boost::posix_time::ptime data_time(boost::gregorian::date(PingHeader->Year, PingHeader->Month, PingHeader->Day), boost::posix_time::hours(PingHeader->Hour)+boost::posix_time::minutes(PingHeader->Minute)+boost::posix_time::seconds(PingHeader->Second)+boost::posix_time::milliseconds(10*int(PingHeader->HSeconds))); 
   stringstream time_ss;
   time_ss << data_time;
   ping.time_string_ = time_ss.str();
   boost::posix_time::time_duration const diff = data_time - epoch;
   ping.time_stamp_ = diff.total_milliseconds();

   ping.port.tilt_angle = M_PI/180.*XTFFileHeader->ChanInfo[0].TiltAngle;
   ping.port.beam_width = M_PI/180.*XTFFileHeader->ChanInfo[0].BeamWidth;
   ping.stbd.tilt_angle = M_PI/180.*XTFFileHeader->ChanInfo[1].TiltAngle;
   ping.stbd.beam_width = M_PI/180.*XTFFileHeader->ChanInfo[1].BeamWidth;

   for (chan=0; chan<PingHeader->NumChansToFollow; chan++) {

      XTFPINGCHANHEADER *ChanHeader;
      short *Imagery; // since BytesPerSample is 2, should be signed 16 bit
      WORD ChannelNumber;
      WORD BytesPerSample;
      DWORD SamplesPerChan;
      DWORD BytesThisChannel;
      
      // point to the channel header
      //
      ChanHeader = (XTFPINGCHANHEADER *) Ptr;

      // See which channel this is (0=port, 1=stbd, etc...)
      //
      ChannelNumber = ChanHeader->ChannelNumber;

      // Compute the number of bytes of imagery for this channel
      //
      BytesPerSample   = XTFFileHeader->ChanInfo[ChannelNumber].BytesPerSample;

      // If the NumSamples value in the channel header is zero, then this
      // must be an old-style XTF File where the samples per channel was
      // stored in this file header.
      SamplesPerChan = XTFFileHeader->ChanInfo[ChannelNumber].Reserved; // for backwards compatibility only!
      tmp = ChanHeader->NumSamples;
      if (tmp != 0) SamplesPerChan = tmp;

      BytesThisChannel = BytesPerSample * SamplesPerChan;

      // skip past the channel header
      //
      Ptr += sizeof(XTFPINGCHANHEADER);

      // Point to the imagery.  If BytesPerSample is 2, then
      // Imagery should be a pointer to a signed 16-bit value.
      // If BytesPerSample is 1, then Imagery should point to 
      // a unsigned 8-bit value.
      Imagery = (short*)Ptr;
      xtf_sss_ping_side* ping_channel;
      if (ChannelNumber == 0) {
          ping_channel = &ping.port;
      }
      else if (ChannelNumber == 1) {
          ping_channel = &ping.stbd;
      }
      else {
          // skip past the imagery;
          Ptr += BytesThisChannel;
          continue;
      }
      ping_channel->pings.reserve(SamplesPerChan);
      for (int i = 0; i < SamplesPerChan; ++i) {
          // we should get port and starboard channel from header definition
          if (ChanHeader->Weight == 0) {
              ping_channel->pings.push_back(Imagery[i]);
          }
          else {
              ping_channel->pings.push_back((int)(Imagery[i]) << (12 - ChanHeader->Weight));
          }
      }
      ping_channel->time_duration = ChanHeader->TimeDuration;
      ping_channel->slant_range = ChanHeader->SlantRange;

      // Do whatever processing on the sidescan imagery here.
      //cout << "Processing a side scan ping!!" << endl;
      cout << "Size of short: " << sizeof(short) << endl;
      cout << "Channel number: " << int(ChannelNumber) << endl;
      //cout << "Channel name: " << ChannelName << endl;
      cout << "Bytes per sample: " << int(BytesPerSample) << endl;
      cout << "Samples per chan: " << int(SamplesPerChan) << endl;
      cout << "Ground range: " << int(ChanHeader->GroundRange) << endl; // seems to always be 0
      cout << "Slant range: " << int(ChanHeader->SlantRange) << endl;
      cout << "Time duration: " << ChanHeader->TimeDuration << endl;
      cout << "SecondsPerPing: " << ChanHeader->SecondsPerPing << endl; // seems to always be 0
      cout << "GAIN Code: " << ChanHeader->GainCode << endl;
      cout << "Initial GAIN Code: " << ChanHeader->InitialGainCode << endl;
      cout << "Weight: " << ChanHeader->Weight << endl;

      // skip past the imagery;
      Ptr += BytesThisChannel;
   }

   std::reverse(ping.port.pings.begin(), ping.port.pings.end());

   return ping;
}

xtf_sss_ping::PingsT read_xtf_file(int infl, XTFFILEHEADER* XTFFileHeader, unsigned char* buffer)
{
    /***************************************************************************************/
    // Given a handle to on open .XTF file, read through the file and
    // print out some data about it.

    //
    // Read the XTF file header
    //

    xtf_sss_ping::PingsT pings;
    if (ReadXTFHeader(infl, XTFFileHeader, buffer) == FALSE) {
        return pings;
    }

    ProcessXTFHeader(infl, XTFFileHeader, buffer);

    //
    // Read the XTF file one packet at a time
    //
    int first=1;
    unsigned int amt;
    char* ptr;
    //HEY This is where we call the c code that produces the wrong type and size
    while ((amt = ReadXTFFormatFileData(infl, buffer)) != 0xFFFF) {
        //
        // Buffer now holds a single record which can be processed
        // here.  The return value from ReadXTFFormatFileData()
        // holds byte length of the data record (i.e., sidescan ping)
        //
      
        XTFPINGHEADER* PingHeader = (XTFPINGHEADER*)buffer;
        if (PingHeader->HeaderType != XTF_HEADER_SONAR) {
            continue;
        }

        xtf_sss_ping ping = process_side_scan_ping((XTFPINGHEADER*)PingHeader, XTFFileHeader);
        ping.first_in_file_ = false;
        pings.push_back(ping);
	if (first){
	  cout << "SONAR "
	       << int(PingHeader->Year) << " "
	       << int(PingHeader->Month) << " "
	       << int(PingHeader->Day) << " "
	       << int(PingHeader->Hour) << " "
	       << int(PingHeader->Minute) << " "
	       << int(PingHeader->Second) << " "
	       << int(PingHeader->HSeconds) << " "
	       << "Sound vel=" << PingHeader->SoundVelocity << " "
	       << "Computed sound vel=" << PingHeader->ComputedSoundVelocity << " "
	       << "Y=" << PingHeader->SensorYcoordinate << " "
	       << "X=" << PingHeader->SensorXcoordinate << " "
	       << "altitude=" << PingHeader->SensorPrimaryAltitude << " "
	       << "depth=" << PingHeader->SensorDepth << " "
	       << "pitch=" << PingHeader->SensorPitch << " "
	       << "roll=" << PingHeader->SensorRoll << " "
	       << "heading=" << PingHeader->SensorHeading << " "  // [h] Fish heading in degrees
	       << "heave=" << PingHeader->Heave << " "            // Sensor heave at start of ping. 
	    // Positive value means sensor moved up.
	       << "yaw=" << PingHeader->Yaw << endl;              // Sensor yaw.  Positive means turn to right.
	  cout << "Tilt angle 0: " << XTFFileHeader->ChanInfo[0].TiltAngle << endl;        // Typically 30 degrees
	  cout << "Beam width 0: " << XTFFileHeader->ChanInfo[0].BeamWidth << endl;        // 3dB beam width, Typically 50 degrees
	  cout << "Tilt angle 1: " << XTFFileHeader->ChanInfo[1].TiltAngle << endl;        // Typically 30 degrees
	  cout << "Beam width 1: " << XTFFileHeader->ChanInfo[1].BeamWidth << endl;        // 3dB beam width, Typically 50 degrees
	  cout << ping.time_string_ << endl;
	  first=0;
	}
    }
    
    if (!pings.empty()) {
        pings[0].first_in_file_ = true;
    }

    if (amt == 0xFFFF) {
        cout << "Stopped - read -1 bytes" << endl;
    }

    cout << "Done!" << endl;

    return pings;
}

xtf_sss_ping::PingsT correct_sensor_offset(const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& sensor_offset)
{
    xtf_sss_ping::PingsT new_pings = pings;
    for (xtf_sss_ping& ping : new_pings) {
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
        // these are my estimated values for the
        // sidescan offset from the center of motion
        //ping.pos_.array() += (2.*Rz.col(0) + -1.5*Rz.col(1)).array();
        ping.pos_.array() += (sensor_offset(0)*Rz.col(0) + sensor_offset(1)*Rz.col(1) + sensor_offset(2)*Rz.col(2)).array();
    }

    return new_pings;
}

xtf_sss_ping::PingsT match_attitudes(const xtf_sss_ping::PingsT& pings, const std_data::attitude_entry::EntriesT& entries)
{
    xtf_sss_ping::PingsT new_pings = pings;

    auto pos = entries.begin();
    for (xtf_sss_ping& ping : new_pings) {
        pos = std::find_if(pos, entries.end(), [&](const std_data::attitude_entry& entry) {
            return entry.time_stamp_ > ping.time_stamp_;
        });

        ping.pitch_ = 0.;
        ping.roll_ = 0.;
        double heave;
        if (pos == entries.end()) {
            ping.pitch_ = entries.back().pitch;
            ping.roll_ = entries.back().roll;
        }
        else if (pos == entries.begin()) {
                ping.pitch_ = pos->pitch;
                ping.roll_ = pos->roll;
        }
        else {
            const std_data::attitude_entry& previous = *(pos - 1);
            double ratio = double(ping.time_stamp_ - previous.time_stamp_)/double(pos->time_stamp_ - previous.time_stamp_);
            ping.pitch_ = previous.pitch + ratio*(pos->pitch - previous.pitch);
            ping.roll_ = previous.roll + ratio*(pos->roll - previous.roll);
        }
    }

    return new_pings;
}

} // namespace xtf_data

namespace std_data {

using namespace xtf_data;

template <>
xtf_sss_ping::PingsT parse_file(const boost::filesystem::path& file)
{
   xtf_sss_ping::PingsT pings;

   // quick sanity check to make sure the compiler didn't
   // screw up the data structure sizes by changing alignment.
   if (
      sizeof(XTFFILEHEADER)      != 1024 || 
      sizeof(CHANINFO)           != 128  ||
      sizeof(XTFPINGHEADER)      != 256  ||
      sizeof(XTFNOTESHEADER)     != 256  ||
      sizeof(XTFBATHHEADER)      != 256  ||
      sizeof(XTFRAWSERIALHEADER) != 64   ||
      sizeof(XTFPINGCHANHEADER)  != 64
      ) {
         cout << "Error: Bad structure size! " <<
            sizeof(XTFFILEHEADER)       << " " <<
            sizeof(CHANINFO)            << " " <<
            sizeof(XTFPINGHEADER)       << " " <<
            sizeof(XTFNOTESHEADER)      << " " <<
            sizeof(XTFBATHHEADER)       << " " <<
            sizeof(XTFRAWSERIALHEADER)  << " " <<
            sizeof(XTFPINGCHANHEADER)   << endl;
         return pings;
   }

   int infl = open(file.string().c_str(), O_RDONLY, 0000200);
   if (infl <= 0) {
       cout << "Error: Can't open " << file.string() << " for reading!" << endl;
       return pings;
   }

   //
   // Allocate memory for reading file data
   //
   // NOTE: max file size is 268MB!
   unsigned char* buffer = (unsigned char*)malloc(268435456);
   if (buffer == NULL) {
       cout << "Can't allocate memory!" << endl;
       exit(-1);
       return pings;
   }

   //
   // Allocate memory for storing XTF header
   //
   XTFFILEHEADER XTFFileHeader; // = (XTFFILEHEADER*)malloc((WORD)sizeof(XTFFILEHEADER));
   pings = read_xtf_file(infl, &XTFFileHeader, buffer);

   if (infl > 0) {
       close(infl);
       infl = 0;
   }
   if (buffer != NULL) {
       free(buffer);
       buffer = NULL;
   }

   return pings;
}

} // namespace std_data
