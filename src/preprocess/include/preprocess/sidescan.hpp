/* Copyright 2019 (johnf@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DATA_TOOLS_HPP
#define DATA_TOOLS_HPP
#include <data_tools/xtf_data.h>
#include <data_tools/std_data.h>
#include <data_tools/navi_data.h>
#include <Eigen/Dense>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <opencv2/core/core.hpp>
#include <preprocess/Timestamp.hh>



namespace preprocess {
  /* 
     A SidescanSide holds the intensity vector for a side of the sidescan.
     It is organized to be linked to other SidescanSides automatically.
     Each side can hold a number of pings up to a max that depends on the 
     length of the vectors.  All the pings on one SidescanSide will have the 
     nearly the same slant range (slant_range) and the same length of vectors 
     (w).  If one adds pings with different values for these a new SidescanSide
     object will automatically be made and linked in.
     This conserves memory and makes things run faster since there are few 
     links to follow, mostly the intensites are in contiguous memory and 
     the width  do not need to be repeated unless they change. More importantly
     it separates the parts that need to be processed differently.

     If navigation or attitude data is available it must be stored outside this.
   */
class SidescanSide{

public:
  long *intensity;
  unsigned short  *slantRange;
  //in units of  m/32, so that slantRange/32= slant_range in m
  long  w;   //slant_range/resolution
  long  h;   //numeber of pings;
  short sonar;  // 0 for subbottom 0, port 1, starboard 2 
  long rows; // amount of h in this object ( h-rows is in next and beyond)
  SidescanSide *next;
  
  ~SidescanSide(){
    clear();
    sonar=-1;
  }
  SidescanSide(){
    rows=0;
    sonar=0;
    w=0;
    h=0;
    intensity = NULL;
    slantRange =NULL;
    next=NULL;
  }
  SidescanSide(short which):SidescanSide(){
    sonar=which;
  }
  SidescanSide(std::string side):SidescanSide(){
    sonar=2;
    if ((side.compare("P")==0)||(side.compare("p")==0)||
	(side.compare("Port")==0)||(side.compare("PORT")==0)||
	(side.compare("port")==0)) sonar=1;
  }
  SidescanSide(xtf_data::xtf_sss_ping::PingsT& pings, short which, long skip=0)
    :SidescanSide(){
    sonar=which;
    setEqual(pings,skip);
  }
  SidescanSide(SidescanSide& pings)
    :SidescanSide(){
    (*this)=pings;
  }
  /*does not change sonar*/
  void clear(){
    if (intensity){
      free(intensity);
      intensity=NULL;
    }
    if (slantRange){
      free(slantRange);
      slantRange=NULL;
    }
    if (next){
      delete next;
      next=NULL;
    }
    w=0;
    rows=0;
    h=0;
  }
  long * operator [] (unsigned long i){
    if (i<rows)
      return &intensity[w*i];
    else if (next) return (*next)[i-rows];
    else return NULL;
  }
  void operator = (xtf_data::xtf_sss_ping::PingsT& pings){
    setEqual(pings);
  }
  void setEqual(xtf_data::xtf_sss_ping::PingsT& pings,long skip=0);
  void operator += (xtf_data::xtf_sss_ping::PingsT& pings);
  void operator = (SidescanSide& pings);
  void operator += (SidescanSide& pings);
  double slant_range(long i){
    if ((i<0)||(i>h))return -1;
    if (i<rows)
      return ((double)slantRange[i])/32.0;
    return next->slant_range(i-rows); 
  }
  long wide(long i){
    if ((i<0)||(i>h))return -1;
    if (i<rows)return w;
    return next->wide(i-rows); 
  }
  /* true if about 0.5% different*/
  bool different(unsigned short slant, unsigned short sr=0){
    if (!sr){
      if ((!rows)||(!slantRange)) return 0;
      sr=slantRange[0];
    }
    long dr=sr-slant;
    if (dr<0)dr=-dr;
    dr=(dr<<10);
    if (sr+slant>0)
      dr/=(sr+slant);
    if (dr<10)return 0;
    return 1;
  } 
  SidescanSide *change(long &len,  unsigned short slant=0, long width=-1 ){
    if (width==-1){
      width=w;
      if (slantRange)
	slant=slantRange[0];
    }
    if (rows) if ((different(slant))||(w!=width))return this;
    len+=rows;
    if (next)
      return next->change(len,slant,width);
    return NULL; 
  }
  void set(SidescanSide & s,long  i){
    if ((i<0)||(i>h)){
      s.clear();
      return;
    }
    if (i>=rows)return next->set(s,i-rows);
    if (s.h!=1){
      s.clear();
      s.intensity=(long*)malloc(w*sizeof(long));
      s.slantRange=(unsigned short*)malloc(sizeof(unsigned short));
      s.rows=1;
      s.h=1;
    }
    s.slantRange[0]=slantRange[i];
    s.w=w;
    memcpy(s.intensity, (*this)[i], w*sizeof(long));
  }  
  int xtf_parse(const boost::filesystem::path& folder)
  {
    if(!boost::filesystem::is_directory(folder)) {
#ifdef VERBOSE_DATA_0
      std::cout << folder << " is not a directory containing" << std::endl;
#endif
      return 1;
    }
    std::list < boost::filesystem::directory_entry>filelist;	
    for (auto& entry :
	   boost::make_iterator_range
	   (boost::filesystem::directory_iterator(folder), {})) {
      if (boost::filesystem::is_directory(entry.path())) continue;
      filelist.push_back(entry);
    }	
    filelist.sort();
    for(boost::filesystem::directory_entry entry :filelist){
#ifdef VERBOSE_DATA_0
      std::cout<<"parsing file "<<entry.path()<<"\n";
#endif
      xtf_data::xtf_sss_ping::PingsT ping=std_data::parse_file< xtf_data::xtf_sss_ping>(entry.path());
      (*this)+=ping;
    }
    return 0;
  }
    
  
/**
This will look thru the pings on the side and locate the index to the nadir.
It will also set to 0 all intensities less then the minalt.  It also sets any intensities below zero or above 2^29 to 0.
pings - the object to check
nadir - a long array of length pings.size() that will hold the index to the nadir for each port side ping,
minalt -  This should be the minimum altitude in m that the sone will see.  All data nearer than this will be set to 0.
minintensityatnadir - This allow you to adjust things if it fails to find the nadir.  The right value can be low like 500 for weak SSH signals at high altitude or high like 100,000 for SSL signals at low altitde.  
returns the number of nadir points detected.  Undetected points will be filled in by adjacent values if present or simply set to minalt.
*/
  int findNadir(long * nadir,  long minintensityatnadir=10000,
		double minalt=10, double maxrange=80);

/**
 RemoveLineArtifact_port(xtf_sss_ping::PingsT& pings, long * nadir, double minArtifactRange minr=30,  double minArtifactRange maxr=90, bool setzero=false, float minintensity=3000, float period=.088, int numberofpeaks=3, float peakwidth=.032)
The arifacts coould be surface reflections that appear as single lines decending in the waterfall image.

Our Sidescan can have an artifact that appears as three bright narrow lines about 8.8 cam appart and 3.2 cm in width.  Which bins varies continously and smoothly in time accross bins.
You can give some hints as to the range that the artifact wanders over and choose to set the values to 0 instead of trying to fill them with 'average' values nearby plus noise.

artif -  This array will be changed to contain the detected bin of the artifact.  Should be allocated to pings.size();  
mininitensity is the minimum filter weighted signal that will be deteted
period - the distance in m between peaks in the artifact
numberofpeaks - the number of peaks in the artifact 
peakwidth -  width of each peak in m.   
returns the number of pings that were adjusted

 **/
  int removeLineArtifact(long * artif,
			 float minintensity=10000,
			 const double minArtifactRange=10,
			 const double maxArtifactRange=100,
			 const bool setzero=false,
			 float period=.088, int numberofpeaks=3,
			 float peakwidth=.032);
  /*
    We use the line between the nadir points  that all intensities are assumed to
    lie. The angle of incidence to this line is then used to normalize the intensities according to a cotangent diffuse reflection rule.

    offset >= 0 is the horizontal distance between the two sonar arrays (assumed parrellel and at the same height in the robot frame 
  nadir_angle- the angle between the bottom of the sidescan beam and the verticle
  i.e beam width= 2 x (pi/2-tilt_angle-nadir_angle)

  */
  void  normalize(long * port_nadir,long * stbd_nadir, double offset=.75,
		  double nadir_angle=.3);

  /*
    Computes a cotangent of the incident angle normalization of the intensities.
    We use the line between the nadir points  that all intensities are assumed 
    to lie.  The intensities can then be  projected to a horizontal line  
    passing thru the point directly verically under the sensor which is now 
    the 0 bin.  These then are interpolated to equal size bins in distance 
    along this horizontal with range resolution m per bin. 

    roll            - Rotation around the forward direction positive lowers 
                      the starboard (right) side.
    port_nadir      - array with the bin number for each ping where the nadir 
                      ends on the port side. 		      
    stbd_nadir      - array with the bin number for each ping where the nadir 
                      ends on the starboard side. 		      
    rangeResoultion - the size of the new uniform horizontal bins
    offset >= 0     - the horizontal distance between the two sonar arrays 
                      (assumed parrallel and at the same height in the robot 
		      frame). 
  		      

    XTF Defines:  z down y forward and x starboard (right) 
    yaw is first and is positive for turns to starboard (right)
    pitch is second and is positive up
    roll is third and is postive when starboard is lowered (with pitch small).
    
    Nautical compass heading is also defined as positiv to the right, 
    north is 0 and east is 90 degreees.
    
    So this is an awkward associsation to axis as in physics and robotics 
    we like to have xyz a right handed system and the order of rotations 
    z y then x.
    
    Most robtics systems have robot frame as:
    x forward, 
    y to the left, and 
    z up,  
    The Euler angles are then based on three rotations: 
    theta: z positive to left, 
    phi: y positive down, and then 
    psi: x lower the right side.

    Yaw, heading, or theta are not relevant here as we work one line at a time.
    Pitch is also not imporant since it will move the projection line but not 
    change the spacing of the bins in each ping.
    Roll is defined as xtf or psi above so positive drops the right side
   

Details:
Notice we define bin number j to be to the right for the starboard sonar 
and left for port.  The unit vectors, j_w  and k_w in the world frame are 
related to those  in the sonar frame. j_s and k_s:

j_w = cos(roll) j_s -sin(roll) k_s
k_w = sin(roll) j_s +cos(roll) k_s 
V_n= nadir sin (nadir_angle) j_r  + nadir cos (nadir_angle) k_r 
V'_n = -(nadir' sin (nadir_angle) + offset) j_r  + nadir' cos (nadir_angle) k_r 
where the V'_n is the nadir for not this sonar ie starboard if this is port
l = (V_n-V'_n)/|v_n-V'_n| = l_y j_r + l_z k_r
n= -l_z j_r + l_y k_r  // the k component is >0
x=V_n . n (dot product)  //should be >0
v_j = x n + s l    where  s= sqrt(j^2-x^2) 
y_j = v_j . j_w  (dot product)  This is the new horizontal range  
    = ( s l_2-x l_3 +, x l_2+ s l_3) . (cos(roll), sin(roll)) 
  */
  void  regularize(const double *roll, long * port_nadir, long * stbd_nadir,
		   double rangeResolution=.05, double offset=.75,
		   double nadir_angle=.3, long width=-1);

  /*
Sets intensities to 0 below the nadir
if nadir=0 this clears below altitude 
  */
  void clean(long *nadir=0){
    if (nadir)
      for (unsigned long i=0; i<h; i++)
	for (int j=0; j<nadir[i]; j++)(*this)[i][j]=0;
  }
  void show_waterfall(int width, int height,
		       long maxPingIntensity,
		      long startbin, long endbin,
		      std::string title="Waterfall");
  
  cv::Mat make_waterfall_image(long width, long maxPingIntensity=2147483647,
			       int rowdownsample=1, int start=0,
			       int end=-1, long binstart=0, long binend=-1);
};

class Sidescan{

public:
  long h;
  SidescanSide side[3];
  Cure::Timestamp *time;  
  Sidescan(const boost::filesystem::path& folder):Sidescan(){
    xtf_parse(folder);    
  }
  Sidescan(xtf_data::xtf_sss_ping::PingsT& pings):Sidescan(){
    (*this)=pings;
  }
  Sidescan(){
    side[0].sonar=0;
    side[1].sonar=1;
    side[2].sonar=2;
    time=NULL;
    h=0;
  }
  ~Sidescan(){
    if (time){
      delete [] time;
      time=NULL;
    }
  }
  SidescanSide & operator [] (int i){
    return side[i];
  }
  void operator = (xtf_data::xtf_sss_ping::PingsT& pings){
    side[1]=pings;
    side[2]=pings;
    h=pings.size();
    if (time){
      delete [] time;
      time=NULL;
    }
    if (h>0){
      time=new Cure::Timestamp[h];
      for (long i=0; i<h; i++){
	time[i].Seconds=(long)((long long)(pings[i].time_stamp_/1000));
	time[i].Microsec=(long)((long long)(pings[i].time_stamp_%1000));
      }
    }
  }
  void operator += (xtf_data::xtf_sss_ping::PingsT& pings){
    //todo this right
    Sidescan s(pings);
    (*this)+=s;
  }
  void operator = (Sidescan& pings){
    if (time){
      delete []time;
      time=NULL;
    }
    h=pings.h;
    side[0]=pings.side[0];
    side[1]=pings.side[1];
    side[2]=pings.side[2];
    
    if (h>0){
      time= new Cure::Timestamp[h];
      if (pings.time)//should work as Timestamp holds only primative types
	memcpy(time,pings.time,h*sizeof(Cure::Timestamp));
    }   
  }
  /*
    appends this with the scans in pings
   */
  void operator += (Sidescan& pings){
    side[0]+=pings.side[0];
    side[1]+=pings.side[1];
    side[2]+=pings.side[2];
    if (pings.h>0){
      Cure::Timestamp* ts= new Cure::Timestamp[pings.h+h];
      if (time){
	memcpy(ts,time,h*sizeof(Cure::Timestamp));
	delete [] time;
      }
      if (pings.time)
	memcpy(&ts[h],pings.time,(pings.h)*sizeof(Cure::Timestamp));
      time=ts;
    }
    h+=pings.h;
  }
  /*
    returns the index of the first scan with time >= t
   */
  long  find(Cure::Timestamp t){
    if (h>0)
      for (long i=0; i<h; i++)
	if (time[i]>=t) return i;
    return -1;
  }
  /*
    sets this to be empty
  */
  void clear(){
    side[0].clear();
    side[1].clear();
    side[2].clear();
    side[0].sonar=0;
    side[1].sonar=1;
    side[2].sonar=2;
    if (time){
      delete [] time;
      time=NULL;
    }
    h=0;
  }
  void  set(Sidescan &ping, Cure::Timestamp start, Cure::Timestamp end=-1){
    ping.clear();
    long ts=find(start);
    long te=find(end);
    if (ts<0)ts=0;
    set(ping,ts,te);
  }
  /*
    ping becomes this for sidescans with index in 
    internval [start,end)  
    end<0 sets end to h
  */
  void  set(Sidescan &ping,long start=0, long end=-1){
    ping.clear();
    if (start<0)start=0;
    if (end<0)end=h;
    if (start<end){
      Sidescan ss;
      SidescanSide s[2];
      ss.h=(end-start);
      ss.time=new Cure::Timestamp[end-start];
      memcpy(ss.time,&time[start],(end-start)*sizeof(Cure::Timestamp));
      for (long i=start; i<end;i++){
	side[1].set(s[0],i);
	side[2].set(s[1],i);
	ss.side[1]+=s[0];
	ss.side[2]+=s[1];
      }
    }
  }
  /*
    this appends pings with additional sidescans with timestamps in 
    interval (start,end] 
  */
  void add(xtf_data::xtf_sss_ping::PingsT& pings, Cure::Timestamp start,
	   Cure::Timestamp end=-1){
    long ts=find(start);
    long te=find(end);
    add(pings,ts,te);
  }
  /*
    this appends pings with additional sidescans with index in 
    internval [start,end)  
    end<0 sets end to h
  */
  void add(xtf_data::xtf_sss_ping::PingsT& pings, long start=0, long end=-1){
    if (start<0) start=0;
    if (end<0)end=h;
    if (end>h)end=h;
    if (start<end)
      for (long i=start; i<end; i++){
	xtf_data::xtf_sss_ping ping;
	ping.time_stamp_=time[i].Seconds*1000+time[i].Microsec/1000;
	for (long j=0; j<side[1].wide(i); j++)
	  ping.port.pings.push_back(side[1][i][j]);
	for (long j=0; j<side[2].wide(i); j++)
	  ping.stbd.pings.push_back(side[2][i][j]);
	ping.port.slant_range=side[1].slant_range(i);
	ping.stbd.slant_range=side[2].slant_range(i);
	pings.push_back(ping);
      }
  }
  int xtf_parse(const boost::filesystem::path& folder)
  {
    long n=0;
    if(!boost::filesystem::is_directory(folder)) {
#ifdef VERBOSE_DATA_0
      std::cout << folder << " is not a directory containing" << std::endl;
#endif
      return 1;
    }
    std::list < boost::filesystem::directory_entry>filelist;	
    for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(folder), {})) {
      if (boost::filesystem::is_directory(entry.path())) continue;
      filelist.push_back(entry);
    }	
    filelist.sort();
    for(boost::filesystem::directory_entry entry :filelist){
#ifdef VERBOSE_DATA_0
      std::cout<<"parsing file "<<entry.path()<<"\n";
#endif
      xtf_data::xtf_sss_ping::PingsT ping=
	std_data::parse_file< xtf_data::xtf_sss_ping>(entry.path()); 
      (*this)+=ping;
     n+=ping.size();
    }
#ifdef VERBOSE_DATA_0
    std::cout<<n<<" pings parsed\n";
#endif
    return 0;
  }
  
  void findNadir(long * p_nadir, long * s_nadir,long minintensityatnadir=10000,
		double minalt=10, double maxrange=80){
    side[1].findNadir(p_nadir, minintensityatnadir,minalt, maxrange);
    side[2].findNadir(s_nadir, minintensityatnadir,minalt, maxrange);
  }
  void  normalize(long * port_nadir,long * stbd_nadir, double offset=.75,
		  double nadir_angle=.3){
     side[1].normalize(port_nadir,stbd_nadir,offset,nadir_angle);
     side[2].normalize(port_nadir,stbd_nadir,offset,nadir_angle);
  }
  void  regularize(const double *roll, long * port_nadir, long * stbd_nadir,
		   double rangeResolution=.05, double offset=.75,
		   double nadir_angle=.3, long width=-1){
    side[1].regularize(roll, port_nadir, stbd_nadir, rangeResolution, offset,
		       nadir_angle, width);
    side[2].regularize(roll, port_nadir, stbd_nadir, rangeResolution, offset,
		       nadir_angle, width);
  }
  
};
  
cv::Mat make_waterfall_image(const xtf_data::xtf_sss_ping::PingsT& pings, long width, long height=0, long maxPingIntensity=65535, long minPingIntensity=0);

  cv::Mat make_waterfall_image(const xtf_data::xtf_sss_ping::PingsT& pings);
} // namespace dtools


#endif 
