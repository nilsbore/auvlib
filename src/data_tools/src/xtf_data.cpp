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
    cv::resize(swath_img, resized_swath_img, cv::Size(512, rows));//resize image
    
    return resized_swath_img;
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

    unsigned int amt;
    char* ptr;
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

#ifdef _MSC_VER
   int infl = open(file.string().c_str(), O_RDONLY | O_BINARY, 0000200);
#else
   int infl = open(file.string().c_str(), O_RDONLY, 0000200);
#endif
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
