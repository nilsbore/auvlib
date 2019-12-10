/*
* Copyright 2019 John Folkesson (johnfe@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <preprocess/sidescan.hpp>
#include <data_tools/std_data.h>
#include <data_tools/xtf_data.h>
extern "C" {
#include <libxtf/xtf_reader.h>
}
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#define M_MAXALLOC 100000000

#include <fcntl.h>
#ifdef _MSC_VER
#include <io.h>
#else
#include <unistd.h>
#endif
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <data_tools/lat_long_utm.h>

using namespace std;
using namespace xtf_data;
namespace preprocess {
  /*
  long trackLines(const long i,  const long h, long * detect, long * labels, long * last5,  int &latest, int  &nextlabel, const double binmotion )

    checks the up to 5 tracks
    
    i -index of the current detection
    
    h - the length of detect.
    
    detect - the detected points length h, detect[i] is the range bin of the current detection 
    
    labels is legth 2h and the first h elements are track labels for the corresponding detect element.  After that come the h counts of the tracks.  So that labels[lables[k]+h-1] are the number of detections assigned label value of labels[k]. Label values run from 1 to h.

    last5 -  the array of length 5 that holds the labels of the last 5 tracks

    latest - the index of the most recent track label.  if latest<0 there are less than 5 tracks and one must add 4 to latest to get the index.
    
    nextlabel - keeps track of the next free label number ready to be assigned and updated  if the current detection is not matched to an existing track.
    
    binmotion is the number of bins between adjacent detections for them to stll be part of same track.
    
    returns counttrack
   */
  
  long trackLines(const long i,  const long h, long * detect, long * labels, long * last5,  int &latest, int  &nextlabel, const double binmotion )
  {
    long track=-1;
    long counttrack=-1;
    long  *counts=&labels[h-1];
    int tp=5;
    int late=latest;
    if(latest<0){
      late=4+latest;
      tp=late+1;
    }
    int label=-1;
    int bestl=-1;

    //last5 has 'i' ping number values for the last 5 detections
    //labels[<h] has the label of the track for that ping which is also the index into counts
    //counts[label[i]] has the number of pings with that label
    if (tp>0)
      for (int k=0; k<tp; k++){
	int kk=(5+late-k)%5;
	int cnt=counts[labels[last5[kk]]];
	if (cnt>counttrack){
	  counttrack=cnt;
	  track=last5[kk];
	}
	float mv=detect[i]-detect[last5[kk]];
	if (mv<0)mv=-mv;
	if (cnt>100)mv/=4.0;
	else if (cnt>50)mv/=2.0;
	if (mv<binmotion) {
	  if ((label==-1)||(cnt>bestl)){
	    label=labels[last5[kk]];
	    bestl=cnt;
	  }
	}
      }
    if (label==-1){
      label=nextlabel;
      nextlabel++;
    }
    late=(late+1)%5;
    last5[late]=i;
    if (latest<0) latest=late-4;
    else latest=late;	
    if (labels[i]>0){
      if (counts[label]>counts[labels[i]]){
	counts[labels[i]]--;
	labels[i]=label;
	counts[labels[i]]++;
      }
    }else{
      labels[i]=label;
      counts[labels[i]]++;
    }
    if (counts[labels[i]]>counttrack)  track=i;    
    return track;
  }
  /*
  SidescanSide::SidescanSide(){
    rows=0;
    sonar=0;
    w=0;
    h=0;
    intensity = NULL;
    slantRange =NULL;
    next=NULL;
    }*/
  void SidescanSide::setEqual(xtf_data::xtf_sss_ping::PingsT& pings,long skip){
    clear();
    h=pings.size()-skip;
    if (h<=0){
      h=0;
      return;
    }
    if (sonar==1){
      w=pings[skip].port.pings.size();
      unsigned short sr=(pings[skip].port.slant_range*32.0+.5);
      long tw=w;
      if (h>0){
	if (w<1){
	  w=1;
	  tw=0;
	}
	if (h*w<M_MAXALLOC) rows=h;
	else rows=M_MAXALLOC/w;
	if (rows>1)
	  for (long i=1; i<rows;i++){
	    if ((pings[skip+i].port.pings.size()!=tw)||
		(different((unsigned short)
			   (pings[skip+i].port.slant_range*32.0+.5),sr))){
	      rows=i;
	    }
	  }
	intensity = (long*)malloc((rows*w)*sizeof(long));
	slantRange = (unsigned short*)malloc((rows)*sizeof(unsigned short));
	for (long i=0; i<rows;i++){
	  slantRange[i]=((double)(pings[skip+i].port.slant_range*32.0+.5));
	  long *p=(*this)[i];
	  for (long j=0; j<w; j++)
	    if (tw) p[j]=pings[skip+i].port.pings[j];
	    else p[j]=0;
	}
      }
    }else if(sonar==2){
      w=pings[skip].stbd.pings.size();
      unsigned short sr=(pings[skip].stbd.slant_range*32.0+.5);
       long tw=w;
      if (h>0){
	if (w<1){
	  w=1;
	  tw=0;
	}
	if (h*w<M_MAXALLOC) rows=h;
	else rows=M_MAXALLOC/w;
	if (rows>1)
	  for (long i=1; i<rows;i++){
	    if ((pings[skip+i].stbd.pings.size()!=tw)||
		(different((unsigned short)
			   (pings[skip+i].stbd.slant_range*32.0),sr))){
	      rows=i;
	    }
	  }
	slantRange = (unsigned short*)malloc((rows)*sizeof(unsigned short));
	intensity = (long*)malloc((rows*w)*sizeof(long));
	for (long i=0; i<rows;i++){
	  slantRange[i]=((double)(pings[skip+i].stbd.slant_range*32.0+.5));
	  long *p=(*this)[i];
	  for (long j=0; j<w; j++)
	    if (tw) p[j]=pings[skip+i].stbd.pings[j];
	    else p[j]=0;
	}
      }
    }
    if (h>rows){
      next=new SidescanSide(pings, sonar, skip+rows);
    }
  }
    
  void SidescanSide::operator += (xtf_data::xtf_sss_ping::PingsT& pings){   
    SidescanSide s(pings,sonar,0);
    (*this)+=s;
  }
  void SidescanSide::operator = (SidescanSide& pings){
    clear();
    sonar=pings.sonar;
    h=pings.h;
    rows=pings.rows;
    w=pings.w;
    if (rows>0){
      slantRange=(unsigned short*)malloc(rows*sizeof(unsigned short));
      memcpy(slantRange,pings.slantRange,rows*sizeof(unsigned short));
      if (w<1){
	w=1; 
     	intensity = (long*)malloc(rows*sizeof(long));
	memset(intensity, 0, rows*sizeof(long));
      } else {
	intensity = (long*)malloc(rows*w*sizeof(long));
	memcpy(intensity,pings.intensity,rows*w*sizeof(long));
      }
    }
    if (pings.next) next=new SidescanSide(*pings.next);
  }

  
  void SidescanSide::operator += (SidescanSide& pings){
    if (sonar!=pings.sonar)return;
    if (pings.h==0)return;
    if (pings.rows==0){
      if (pings.next) return (*this)+=(*pings.next);
      else return;
    }
    if (h==0){
      (*this)=pings;
      return;
    }
    if (next){
      if (rows!=h-next->h){
	std::cout<<" OUCH "<<rows<<" "<<h<<" "<<next->h<<" "<<next->rows<<"\n";
      }
    }
    if (next){
      h+=pings.h;
      (*next)+=pings;
      if (next){
	if ((next->w==0)||(rows!=h-next->h)){
	        std::cout<<pings.h<<" ";
		std::cout<<" OUCH2 "<<rows<<" "<<h<<" "<<next->h<<" "<<next->rows<<" "<<w<<" "<<next->w<<"\n";
	}
    }
      return;
    }
    if (rows==0) return(*this)=pings;
    if ((w!=pings.w)||(different(pings.slantRange[0]))){
      h+=pings.h;
      next = new SidescanSide(pings);
      if (next){
	if ((next->w==0)||(rows!=h-next->h)){
	  std::cout<<next->w<<" OUCH3 "<<rows<<" "<<h<<" "<<next->h<<" "<<next->rows<<"\n";
	}
      }
      return;
    }
    long sz=(rows+pings.rows);
    if (sz*w>M_MAXALLOC){
      sz=M_MAXALLOC/w;
      if (sz<rows){
	h=h+pings.h;
	next= new SidescanSide(pings);
	if (next){
	  if ((0==next->w)||(rows!=h-next->h)){
	    std::cout<<next->w<<" OUCHa "<<rows<<" "<<h<<" "<<next->h<<" "<<next->rows<<"\n";
	  }
	}
	return;	
      }
      
    }//now sz is the amount to allocate
    if (sz>0){
      long tw=w;
      if (w==0)w=1;
      long* newinten= (long*)malloc(sz*w*sizeof(long));
      if (intensity){
	if (tw)
	  memcpy(newinten,intensity,rows*w*sizeof(long));
	else memset(newinten,0,rows*w*sizeof(long));
	free(intensity);
      }
      unsigned short* newsl= (unsigned short*)malloc(sz*sizeof(unsigned short));
      if (slantRange){
	memcpy(newsl,slantRange,rows*sizeof(unsigned short));
	free(slantRange);
      }
      if (pings.intensity)
	if (tw)
	  memcpy(&newinten[rows*w],pings.intensity,(sz-rows)*w*sizeof(long));
	else
	  memset(&newinten[rows*w],0,(sz-rows)*w*sizeof(long));
      intensity=newinten;
      if (pings.slantRange)
	memcpy(&newsl[rows],pings.slantRange,(sz-rows)*sizeof(unsigned short));
      slantRange=newsl;
    
      long skip=sz-rows;
      h=sz;
      rows=sz;    
      sz=pings.rows-skip;   // amount remaining to put on our next
      if (sz>0){
	if (skip>pings.rows)return; //should not happen
	next= new SidescanSide();
	next->sonar=sonar;
	next->intensity= (long*)malloc(sz*w*sizeof(long));
	next->slantRange= (unsigned short*)malloc(sz*sizeof(unsigned short));
	if (tw)
	  memcpy(next->intensity,&pings.intensity[skip*w],sz*w*sizeof(long));
	else memset(next->intensity,0,sz*w*sizeof(long));
	memcpy(next->slantRange,&pings.slantRange[skip],
	       sz*sizeof(unsigned short));
	next->h=sz;
	next->rows=sz;
	next->w=w;
	h+=sz;
	if (pings.next){
	  h+=pings.next->h;
	  (*next)+=(*pings.next);
	  if (next){
	    if (rows!=h-next->h){
	      std::cout<<" OUCH4 "<<rows<<" "<<h<<" "<<next->h<<" "<<next->rows<<"\n";
	    }
	  }
	  
	}
      
      }
    }
    if (next){
      if (rows!=h-next->h){
	std::cout<<" OUCH5 "<<rows<<" "<<h<<" "<<next->h<<"\n";
      }
    }
  }
  
  
  /*  
We average in .8 m windows and then compare 3 of these space at 1.2 m intervals.
The middle window must have an average intensity above minintensityatnadir and be larger than the first window.   Then the second derivative foune by these three is compared and the maximum taken as the detection.  There is also a factor to favor points near the recently found nadir points.  The nadir is taken as the start of the middle window (found empirically).

  */
  int SidescanSide::findNadir(long * nadir, long minintensityatnadir,
			      double minalt, double maxrange){
    if (h==0) return 0;
    long len=0;
    SidescanSide *cs=change(len);
    if (w==0){
      if (cs)
	return
	  cs->findNadir(&nadir[len], minintensityatnadir, minalt, maxrange);
      return 0;
    }
    //res is m per bin
    //moving average window size
    double resolution=slant_range(0);
    resolution/=(double)(w);
    int maw=(int)((double)(1.2/resolution+.5));
    if (maw<2)maw=2;
    int pw=(int)((double)(.8/resolution+.5));
    if (pw<2)pw=2;
    int minj=(int)((double)(minalt/resolution+.5));
    minj-=maw;  //to allow derivatives at minalt
    if (minj<2)minj=2;  
    minj+=maw;  //to keep range=(j+minj)*res
    int jw=(int)((double)(maxrange/resolution+.5));
    jw+=maw+pw;  //to allow second derivatives at max range
    if (jw>w)jw=w;
    jw-=(maw+pw);  //j range is min alt to max range but m mw must extend beyond that
    jw=(jw-minj);
    jw+=maw;
    if (jw<0)return 0;
    int mw=jw+2*maw;
    unsigned long mvavg[mw];  //avg (j,j+maw)
  //This sets how far the nadir can wander between pings.
    double binmotion=1.5/resolution;
    float mv=0;
    float bestmv=0;
    int countr=0;
    int counttrack=0;
    float running=minj+maw;
    for (int direction=0; direction<2; direction++)
      for (long ii=0; ii<len; ii++){
	long i=ii;
	if (ii==0)counttrack=0;
	if (direction%2){
	  i=len-1-ii;
	}else {
	  if (direction==0)nadir[i]=0;
	}
	if (nadir[i]>0){
	  if (counttrack>5){
	    mv=nadir[i]-minj-running;
	    if (mv<0) mv=-mv;
	    if (mv>binmotion)nadir[i]=0;
	  }
	}
	if (nadir[i]==0){
	  long * pings=&(*this)[i][minj-maw];
	  mvavg[0]=pings[0]+pw/2;;
	  for (int k=1;k<pw; k++)mvavg[0]+=pings[k];
	  for (int j=1; j<mw; j++){
	    mvavg[j]=mvavg[j-1];
	    mvavg[j]-=pings[j-1];
	    mvavg[j]+=pings[j-1+pw];
	    mvavg[j-1]/=pw;
	  }
	  mvavg[mw-1]/=pw;
	  float maxscore=0;
	  int jmax=0;
	  bestmv=-1;
	  int jstart=maw;
	  int jend=jw-maw;
	  int mi=minintensityatnadir;
	  int trys=3;
	  while (trys){
	    for( int j=0; j<jw; j++){
	      unsigned long temp=mvavg[j+maw];// first derivative
	      if ((temp>mi)&&(mvavg[j]<temp)) {
		double score=temp;
		temp=mvavg[j+2*maw]+mvavg[j];
		temp=(temp>>1);
		score-=temp;
		score/=(double)(mvavg[j]+10);
		if (score>0){
		  mv=j-running;
		  if (mv<0) mv=-mv;
		  mv/=binmotion;
		  if (counttrack>5){
		    double x=(6-counttrack*.02);
		    if (x<1)x=1.0;
		    if ((counttrack<100)||(mv>1.0))
		      if (mv<x)score-=mv*mv*(1.5/(x*x));
		      else score-=1.5;
		  }
		  if(score>maxscore){
		    if ((bestmv>-1)&&(mv>bestmv)&&(counttrack>3))j=mw;
		    else{
		      maxscore=score;
		      jmax=j;
		      bestmv=mv;
		      if (counttrack<5)
			if (score>0.5)
			  j=mw;
		    }
		  }
		}
	      }
	    }
	    mi/=2;
	    if (jmax)trys=0;
	    else trys--;
	  }
	  if(jmax>0)nadir[i]=jmax+minj;
	  if(nadir[i]>0){
	    if (counttrack>0)
	      if (bestmv<1.0) {
		running=nadir[i]-minj;
		counttrack++;
	      }else counttrack--;
	    else {
	      running=nadir[i]-minj;
	      counttrack++;
	    }
	  } else {
	    if (counttrack>100)
	      counttrack=(counttrack>>1);
	    else counttrack=0;
	    //nadir[i]=running;
	  }
	}
      }
    long last5[5];
    int latest=-5;
    long labels[2*len];
    std::memset(labels, 0, sizeof labels);
    long  *counts=&labels[len-1];
    int nextlabel=1;
    running =minj;
    for(long i=0; i<len;i++){
      if (nadir[i]>0){
	double bm=binmotion/1.5;
	if (counttrack>100)bm/=4.0;
	else if (counttrack>50)bm/=2.0;
	long track=trackLines(i, len, nadir, labels, last5, latest,
			      nextlabel, bm);
	if (track>-1){
	  running=nadir[track];
	  counttrack=counts[labels[track]];
	}
      }	
    }
    
    for(int i=0; i<len; i++){
      if (counts[labels[i]]<25){
	counts[labels[i]]--;
	labels[i]=0;
	nadir[i]=0;
      }  
    }
    running=0;
    for (long i=0; i<len; i++){
      if (nadir[i]==0){
	long next=0;
	long ii=i+1;
	while (ii<len){
	  if (nadir[ii]>0){
	    next=ii;
	    ii=len;
	  }
	  ii++;
	}
	if (next>0){
	  if (running>0)
	    nadir[i]=(float)(0.5+(running*(float)(next-i)+(float)nadir[next])/(float)(next-i+1));
	  else
	    nadir[i]=nadir[next];
	} else {
	if (running>0)
	  nadir[i]=running;
	else
	  nadir[i]=minj;
	}
      }
      running =nadir[i]; 
    }
    countr=0;
    long long avg=0;
    int max=0;
    int min=w;
    for (long i=2; i<len-2;i++)
      nadir[i]=((nadir[i-2]+nadir[i-1]+nadir[i+1]+nadir[i+2])>>2);
    for (long i=0; i<len;i++){
      if (nadir[i]>max)max=nadir[i];
      if (nadir[i]<min)min=nadir[i];
      avg+=nadir[i];
      countr++;
    }
    double nad=((double)avg/countr);
#ifdef M_VERBOSE_DATA_1
    std::cout<<(len-countr)<<" not tracked, "<<countr<<" tracked "<<" of "<<len<<" pings. \n";
    std::cout<<"side "<<sonar<<" Avg  Nadir is "<<(resolution*nad)<<" m and range is ("<<(min*resolution)<<", "<<(max*resolution)<<")\n";
#endif
    if (cs) return countr+
	      cs->findNadir(&nadir[len], minintensityatnadir, minalt, maxrange);
    return countr;
  }
  /*
    RemoveLineArtifact_port(xtf_sss_ping::PingsT& pings, long * nadir, double minArtifactRange minr=30,  double minArtifactRange maxr=90, bool setzero=false)

Our Port side Sidescan has an artifact that appears as a bright spot in about 25 cm of bins.  Which bins varies continously and smoothly in time accross bins.
You can give some hints as to the range that the artifact wanders over and choose to set the values to 0 instead of trying to fill them with 'average' values nearby plus noise.

 **/

 int SidescanSide::removeLineArtifact(long * artif, float  minintensity,
				      const double minArtifactRange,
				      const double maxArtifactRange,
				      const bool setzero, float period,
				      int numpeaks, float peakwidth ){
   if (h==0) return 0;
   long len=0;
   SidescanSide *cs=change(len);
   if (w==0){
     if (cs)
       return
	 cs->removeLineArtifact(&artif[len], minintensity, minArtifactRange,
				maxArtifactRange, setzero, period, numpeaks,
				peakwidth );
     return 0;
   }
   double resolution = slant_range(0);
   resolution/=(double)(w);
   //moving average window size
   int per=(int)((double)(period/resolution+.5));
  if (per<2)per=2;
  int pw=(int)((double)(peakwidth/resolution+.5));
  if (pw<2)pw=2;
  int maw=numpeaks*per;
  int minj=(int)((double)(minArtifactRange/resolution+.5));
  if (minj<5)minj=5;
  minj+=maw;
  int jw=(int)((double)(maxArtifactRange/resolution+.5));
  if (jw>w)jw=w;
  jw=(jw-minj-maw);
  if (jw+2*maw+(numpeaks-1)*per+pw>w)
    jw=w-2*maw+(numpeaks-1)*per+pw-1;
  if (jw<1)return 0;
  //This sets how far the artif can wander between pings.
  double binmotion=0.5/resolution;
  double mv=0;
  int countr=0;
  int howmany=0;
  int counttrack=0;  
  int mw=jw+2*maw+(numpeaks-1)*per;
  unsigned long mvavg[mw];  //avg (j,j+maw)
  float running=jw/2+minj;
  long last5[5];
  int latest=-5;
  long labels[2*len];
  std::memset(labels, 0, sizeof labels);
  long  *counts=&labels[len-1];
  int nextlabel=1;
  float scale=1;
  for (int direction=0; direction<6; direction++){
    if (direction>1)   scale=0.1*(float)direction;
    for (long ii=0; ii<len; ii++){
      if (ii==0)latest=-5;
      int i=ii;
      int nindex=i+1;
      int inc=1;
      if (direction%2==1) {
	i=len-1-i;
	nindex=len-1-nindex;
	inc=-1;
      } 
      if (direction>0){
	if (counttrack>20)
	  while ((nindex<len-1)&&(nindex>1)){
	    if (labels[nindex]>0){
	      if (counts[labels[nindex]]>20){
		float diff=artif[nindex]-running;
		if (nindex-i-inc==0)
		  running+=diff/2.0;
		else{
		  running+=inc*diff/(nindex-i-inc);
		}
	      }
	      break;
	    }
	    nindex+=inc;
	    
	  }
      } else artif[i]=0;
      if (artif[i]==0){
	long *pings=&(*this)[i][minj-maw];
	mvavg[0]=pings[0]+pw/2;
	for (int k=1;k<pw; k++)mvavg[0]+=pings[k];
	for (int j=1; j<mw; j++){
	  mvavg[j]=mvavg[j-1];
	  mvavg[j]-=pings[j-1];
	  mvavg[j]+=pings[j-1+pw];
	  mvavg[j-1]/=pw;
	}
	mvavg[mw-1]/=pw;
	float maxscore=0;
	int jmax=0;
	int jstart=0;
	int jend=jw;
	if ((direction>1)&&(counttrack>200)){
	  int d=(float)(40.0/(direction*resolution)+.5);
	  jstart=running-d;
	  jend=running+d;
	  if  (jstart<0)jstart=0;
	  if (jend>jw)jend=jw;
	}
	if (jstart<jend)
	  for( int j=jstart; j<jend; j++){
	    //mvavg index is score index + maw
	    unsigned long long temp=0;
	    unsigned long long temp2=0;
	    for (int k=0; k<numpeaks; k++){
	      temp+=((mvavg[j+k*per]+mvavg[j+2*maw+k*per]+1)/2.0);
	      temp2+=mvavg[j+maw+k*per];
	    }
	    double score=temp;
	    score=-score;
	    score+=temp2;
	    if (score>minintensity*numpeaks) {
	      score/=(double)temp;
	      if (score>1.0){
		mv=j+minj-running;
		if (mv<0) mv=-mv;
		mv/=binmotion;
		double x=(6-counttrack*.02)*scale;
		if (x<0.1)x=0.1;
		if ((counttrack>100)&&(mv<2.0*scale))score+2.0;
		else if ((counttrack>2)&&(mv<x))score-=mv*mv*(1.5/(x*x));
		else score-=1.5;
		if(score>maxscore){
		  maxscore=score;
		  jmax=j;
		}
	      }
	    }
	  }
	if(maxscore>0)artif[i]=jmax+minj;
      }
      if (artif[i]>0){
      	long track=trackLines(i, len, artif, labels, last5, latest, nextlabel,
			      (binmotion/4.0));
	if (track>-1){
	  running=artif[track];
	  counttrack=counts[labels[track]];
	}
      }
    }
    
    countr=0;
    for(int i=0; i<len; i++){
      if (labels[i]>0){
	if (counts[labels[i]]<20){
	  counts[labels[i]]--;
	  labels[i]=0;
	  artif[i]=0;
	} else countr++;
      }
    }
    if (nextlabel>1)
      for (int i=1; i<nextlabel;i++){
	if (counts[i]==0){
	  if (nextlabel>i+1)
	    for (int ii=i+1; ii<nextlabel; ii++)
	      counts[ii-1]=counts[ii];
	  nextlabel--;
	  counts[nextlabel]=0;
	  for (int ii=0; ii<len; ii++)
	    if (labels[ii]>i)labels[ii]--;
	  i--;
	}
	
      }
  }
  double mean=0;
  for (int i=1; i<len-1; i++){
    if (artif[i]<1){
      if ((artif[i-1]>0)&&(artif[i+1]>0))
	if (labels[i-1]==labels[i+1]){
	  labels[i]=labels[i-1];
	  counts[labels[i]]++;
	  artif[i]=((artif[i-1]+artif[i+1])/2);
	  countr++;
	}
    }else mean+=(artif[i]);
  }
  double offset=(double)(pw+per*(numpeaks-1))/2.0;
  if (countr)mean/=(double)countr;
  else mean=-offset;
  mean+=(offset);
  mean*=resolution;
  for (int i=0; i<len; i++){
    if (artif[i]>0){
      artif[i]+=offset;
      if (artif[i]>w-1)artif[i]=w-1;
      int width=(float)(.35/resolution+.5);
      int top =artif[i]+width+maw;
      if (top>w) top=w;
      int bottom=artif[i]-width;
      if (bottom<0)bottom=0;
      double avg=0;
      int sta=top+1/resolution;
      if (sta>w-width-1) sta=w-width-1;
      long *pings=&(*this)[i][0];
      for (int k=sta; k<sta+width; k++)
	avg+=pings[k];
      sta=artif[i]-2*width-1/resolution;
      if (sta<0) sta=0;
      if (sta+width>w)width=w-sta;
      if (width>0){
	for (int k=sta; k<sta+width; k++)
	  avg+=pings[k];
	avg/=(2*width);
      }
      long artifact=avg*1.5;
      long artifact2=avg/2.0;
      if (setzero)avg=0;
      double r1=(1.0*(double)rand())/(2.0*(double)RAND_MAX)+.75;     
      r1*=avg;
      if (bottom<top)
	for (int k=bottom;k<top; k++){
	  if ((pings[k]>artifact)||(pings[k]<artifact2))
	    {
	      howmany++;
	      double r=((double)rand())/((double)RAND_MAX) +.5;
	      (*this)[i][k]=r1*r;
	    }
	  
	}
    }
  }
  howmany/=len;
#ifdef M_VERBOSE_DATA_1
  std::cout<<(len-countr)<<" not tracked, "<<countr<<" tracked "<<" of "<<len<<" pings. \n"<<(nextlabel-1)<<" tracks of lengths: ";
   for (int i=1; i<nextlabel; i++)
     std::cout<<counts[i]<<" ";
   std::cout<<"\n";  
   std::cout<<mean<<" range and  width of "<<howmany<<" bins (= "<<(howmany*resolution)<<"m) in each ping"<<"\n";
#endif
   if (cs) return countr+
	     cs->removeLineArtifact(&artif[len], minintensity, minArtifactRange,
				maxArtifactRange, setzero, period, numpeaks,
				peakwidth );
   return countr;
 }
   

  /**
     x is hieght of triangle fromed by the two nadir points and the 2*16 degree nadire angle:
   
     x cos a1= r1
     x cos(32-a1)= r2
     x[cos(32)cos(a1)-sin(32)sin(a1)=r2
     cos(32)r1-x sin(32)(1-(r1/x)^2)^.5 = r2
     ((cos(32)r1-r2)/sin(32))^2 = x^2-r1^2
     x^2=((cos(32)r1-r2)/sin(32))^2 + r1^2
     
     X is then the same for all points so that 
     x= r_i cos a_i
     where r_i is the range of the bin and a_i is now the incidence angle to the plane.
     cos(a_i)=x/r_i
     the intensities are to be adjusted by multiplying by the tan(a_i)
     This is an assumption that the reflection is difusse and that the line conecting the two nadir points extends to the extent of the scan on both sides.  i.e. the bttom is planar here.
  */

  void  SidescanSide::normalize(long * port_nadir, long * stbd_nadir,
				double offset, double nadir_angle)
{
  if (h==0) return;
  long len=0;
  SidescanSide *cs=change(len);
  if (w==0){
    if (cs)
      return
	cs->normalize( port_nadir, stbd_nadir, offset, nadir_angle);
    return;
  }
  double cn=cos(nadir_angle);
  double sn=sin(nadir_angle);
  long *nadir;
  long *onadir;
  if (sonar==1){
    nadir=port_nadir;
    onadir=stbd_nadir;
  } else {
    onadir=port_nadir;
    nadir=stbd_nadir;
  }
  //We set up a max horizontal bin of what we would have at 10 m altitude
  if ((len)&&(w))
    for (long i = 0; i < len; i++) {
      double ly=sn*((double)(nadir[i]+onadir[i])+1.0)+offset;  
      double lz=cn*(double)(nadir[i]-onadir[i]);
      double s=sqrt(ly*ly+lz*lz);
      ly/=s;
      lz/=s;
      double x= ((double)nadir[i]+0.5)*(cn*ly-sn*lz);
      double x2=x*x;
      long *pings=&(*this)[i][0];
      for (long j = 1; j < w; j++) {
	//cos(phi)=x/j  tan(phi)= sqrt(s/x2-1)
	s=((double)j+0.5)*((double)j+0.5);  
	if (s>x2){ //begun intersecting line
	  s= sqrt(s/x2-1);
	  if (s>1000.0)s=1000;
	  long long temp=pings[j];
	  temp=((double)(temp+0.5))*s;
	  if (temp<2147483648)pings[j]=temp;
	  else pings[j]=2147483647;
	} else {// point not intersecting the line yet
	  pings[0]=0;
	}
      }
    }
  if (cs) cs->normalize( port_nadir, stbd_nadir, offset, nadir_angle);
}
  void  SidescanSide::regularize(const double *roll,
				 long * port_nadir,
				 long * stbd_nadir,
				 double rangeResolution,
				 double offset, double nadir_angle, long width)
  {  
    if (h==0) return;
    if (rows*w){
      double cn=cos(nadir_angle);
      double sn=sin(nadir_angle);
      long *nadir;
      long *onadir;
      if (sonar==1){
	nadir=port_nadir;
	onadir=stbd_nadir;
      } else {
	onadir=port_nadir;
	nadir=stbd_nadir;
      }
      double resolution =slant_range(0);
      resolution/=(double)(w);
      double y=((double)w)*resolution;
      if (y<25)y=25;
      //We set up a max horizontal bin of what we would have at 10 m altitude
      if (rangeResolution<1E-8)rangeResolution=1E-8;
      long hw=sqrt(y*y-100.0)/rangeResolution;
      if (width>0) hw=width;  
      long *newinten=(long *)malloc(hw*rows*sizeof(long));
      memset(newinten,0,sizeof newinten);
      long maxbin=0;
      long minbin=hw;
      for (long i = 0; i < rows; i++) {
	double cr=cos(roll[i]);
	double sr=sin(roll[i]);
	if (sonar==2) sr=-sr;
	double ly=sn*((double)(nadir[i]+onadir[i])+1.0)+offset;  
	double lz=cn*(double)(nadir[i]-onadir[i]);
	double s=sqrt(ly*ly+lz*lz);
	ly/=s;
	lz/=s;
	double x= ((double)nadir[i]+0.5)*(cn*ly-sn*lz);
	double x2=x*x;
	long lastj=0;
	long bin=0;
	y=rangeResolution/resolution;
	double inc=y;
	long * pings=&(*this)[i][0];
	long long avg=pings[0];
	for (long j = 1; j < w; j++) {
	  s=((double)j+0.5)*((double)j+0.5);  
	  if (s>x2){ //begun intersecting line
	    s=sqrt(s-x2);
	    s= (s*ly-x*lz)*cr + (x*ly+ s*lz)*sr;//this is our horizontal range
	    if (s<y){  //have to still average into this bin
	      avg+=pings[j];
	    }else { //past the bin boundary
	      avg/=(j-lastj);
	      while ((y<=s)&&(bin<hw)){
		newinten[i*hw+bin]=avg;
		y+=inc;
		bin++;
	      }
	      avg=pings[j];
	      lastj=j;
	    }
	  }else {// point not intersecting the line yet
	    avg=pings[0];
	    lastj=j;
	  }
	}
	if(bin>maxbin)maxbin=bin;
	if(bin<minbin)minbin=bin;
	avg/=(w-lastj);
	while(bin<hw){
	  newinten[i*hw+bin]=0;
	  bin++;
	}
      }
      
      free(intensity);
      if ((hw>maxbin)&&(hw!=width)) {
	intensity=(long *)malloc(maxbin*rows*sizeof(long));
	memset(intensity,0,sizeof intensity);
	for (long i=0; i<rows; i++)
	  memcpy(&intensity[i*maxbin],&newinten[i*hw], maxbin*sizeof(long));
	w=maxbin;
	free(newinten);
      }else{
	intensity=newinten;
	w=hw;
      }
      resolution=rangeResolution;
    }
    if (next) next->regularize(roll+rows, port_nadir+rows, stbd_nadir+rows,
			       rangeResolution, offset,nadir_angle, w);
  }
  void SidescanSide::show_waterfall(int width, int height,
				    long maxPingIntensity, long startbin,
				    long endbin, std::string title){
    if (h==0) return;
    cv::Mat img;
    long i=0;
    long len=0;
    SidescanSide *cs=change(len);
    while (i<len){
      char b[title.length()+25];
      b[title.length()+24]=0;
      sprintf (b, "%s_%d_to_%d.png",title.c_str(),i, i+height);
      std::string s(b);
      cout<<"Showing "<<s<<"\n";
      img=make_waterfall_image(width, maxPingIntensity,1,
				       i,i+height,startbin, endbin);
      cv::imshow(title, img);
      int k=cv::waitKey();
      if (k==82)
	i-=height/2;
      else if (k==27)return;
      else if (k==115){
	std::cerr<<"saved "<<s<<"\n";
	cv::imwrite(s,img);
	int k=cv::waitKey();
      }
      else i+=height/2;
    }
    if (cs)cs->show_waterfall(width, height, maxPingIntensity, startbin,
			      endbin, title);
  }
  
  cv::Mat SidescanSide::make_waterfall_image(long width, long maxPingIntensity,
					     int rowdownsample, int start,
					     int end, long startbin,
					     long endbin)
  {
    //TODO improve bin boundary interpolation
    if (h*w==0) return  cv::Mat(0, 0, CV_8UC1, cv::Scalar(0));
    if (maxPingIntensity<1)maxPingIntensity=2147483647;
    if (startbin<0)startbin=0;
    else if(startbin>w-1)startbin=0;
    if (endbin<=startbin)endbin=w;
    if (endbin>w)endbin=w;
    long range=endbin-startbin;
    float downsample=1;
    if (width>0) downsample=(float)range/width;
    else width=range;
    if (downsample<1){
      width=range;
      downsample=1;
    }
    long len=0;
    SidescanSide *cs=change(len);
    if ((start<0)||(start>=len))start=0;
    if (end<start)end=len;
    if (end>len)end=len;
    long hh=end-start;
    uchar val=0;
    long height=hh;
    if (rowdownsample<1)rowdownsample=1;
    double d=(double)hh/rowdownsample;
    height=d;
    if (height*width==0){
      if (cs) return cs->make_waterfall_image(width, maxPingIntensity,
					      rowdownsample, start, end,
					      startbin, endbin);
      return  cv::Mat(0, 0, CV_8UC1, cv::Scalar(0));
    }
    d=(double)hh/(double)height;
    int count=0;
    int num=0;
    long long acc=0;
    cv::Mat swath_img = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    float topr=d+start;
    for(int i=0; i< height; i++, topr+=d){
      float topc=downsample+startbin;
      if ((int)topr>h)topr=len;
      for (int j=0; j<width; j++, topc+=downsample){
	acc=0;
	num=0;
	int r=i*d+start;
	int index=j*downsample+startbin;
	while (r<(int)topr){
	  while (index<(int)topc){
	    if ((index<endbin)&&(r<end)){
	      long temp=(*this)[r][index];
	      if (temp>maxPingIntensity){
		count++;
		temp=maxPingIntensity;
	      }
	      acc+=temp;
	      num++;
	    }
	    index++;
	  }
	  r++;
	}
	if (num>0)val=(255.0*((double)acc/(double)num))/
		    ((double)maxPingIntensity);
	else val=0;
	//	cv::Scalar_<uchar>* p = swath_img.ptr<cv::Scalar_<uchar> >(i,j);
	uchar* p = swath_img.ptr(i,j);
	*p=val;
      }
    }
    double p=count*100.0/(len*range);
#ifdef M_VERBOSE_DATA_1
    std::cout<<"There were "<<count<<" or "<<(p)<<"% above the maxPingIndensity of "<<maxPingIntensity<<"\n";
#endif
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
#ifdef M_VERBOSE_DATA_1
      std::cout<<"There were "<<count<<" or "<<p<<"% above the maxPingIndensity of "<<maxPingIntensity<<"\n";
#endif
    return swath_img;
  }


} // namespace dtools
