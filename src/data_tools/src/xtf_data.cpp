#include <data_tools/xtf_data.h>
extern "C" {
#include <libxtf/xtf_reader.h>
}
//#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
//#include <stdlib.h>
//#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings)
{
    int rows = pings.size();
    int cols = pings[0].port_pings.size() + pings[0].stbd_pings.size();
    cv::Mat swath_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < pings.size(); ++i) {
        for (int j = 0; j < pings[i].port_pings.size(); ++j) {
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, cols-j-1);
            p->z = uchar(255.*(float(pings[i].port_pings[j]) + 32767.)/(2.*32767.));
            p->y = uchar(255.*(float(pings[i].port_pings[j]) + 32767.)/(2.*32767.));
            p->x = uchar(255.*(float(pings[i].port_pings[j]) + 32767.)/(2.*32767.));
        }
        for (int j = 0; j < pings[i].stbd_pings.size(); ++j) {
            cv::Point3_<uchar>* p = swath_img.ptr<cv::Point3_<uchar> >(i, pings[0].stbd_pings.size()-j-1);
            //tie(p->z, p->y, p->x) = uchar(255.*(float(pings[i].port_pings[j]) + 32767.)/(2.*65536));
            p->z = uchar(255.*(float(pings[i].stbd_pings[j]) + 32767.)/(2.*32767.));
            p->y = uchar(255.*(float(pings[i].stbd_pings[j]) + 32767.)/(2.*32767.));
            p->x = uchar(255.*(float(pings[i].stbd_pings[j]) + 32767.)/(2.*32767.));
        }
    }
    cv::Mat resized_swath_img;//dst image
    cv::resize(swath_img, resized_swath_img, cv::Size(rows/8, cols/8));//resize image
    
    return resized_swath_img;
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
   WORD chan;
   int tmp;
   unsigned char *Ptr = (unsigned char *)PingHeader;

   // For backwards-compatibility.  The samples per channel used to
   // be stored in the file header.  


   // skip past the ping header
   Ptr += sizeof(XTFPINGHEADER);

   xtf_sss_ping ping;
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
      for (int i = 0; i < SamplesPerChan; ++i) {
          // we should get port and starboard channel from header definition
          if (ChannelNumber == 0) { // port channel
              ping.port_pings.push_back(Imagery[i]);
          }
          else if (ChannelNumber == 1) { // stbd channel
              ping.stbd_pings.push_back(Imagery[i]);
          }
      }

      // Do whatever processing on the sidescan imagery here.
      //cout << "Processing a side scan ping!!" << endl;
      cout << "Size of short: " << sizeof(short) << endl;
      cout << "Channel number: " << int(ChannelNumber) << endl;
      //cout << "Channel name: " << ChannelName << endl;
      cout << "Bytes per sample: " << int(BytesPerSample) << endl;
      cout << "Samples per chan: " << int(SamplesPerChan) << endl;

      // skip past the imagery;
      Ptr += BytesThisChannel;
   }

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
             << "Y=" << PingHeader->SensorYcoordinate << " "
             << "X=" << PingHeader->SensorXcoordinate << " "
             << "pitch=" << PingHeader->SensorPitch << " "
             << "roll=" << PingHeader->SensorRoll << " "
             << "heading=" << PingHeader->SensorHeading << " "  // [h] Fish heading in degrees
             << "heave=" << PingHeader->Heave << " "            // Sensor heave at start of ping. 
                           // Positive value means sensor moved up.
             << "yaw=" << PingHeader->Yaw << endl;              // Sensor yaw.  Positive means turn to right.
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
   unsigned char* buffer = (unsigned char*)malloc((WORD)32768);
   if (buffer == NULL) {
       cout << "Can't allocate memory!" << endl;
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
