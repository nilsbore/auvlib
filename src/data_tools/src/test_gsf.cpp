// Copyright 2015 Google Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <cassert>

#include <cereal/archives/json.hpp>

#include <gsf.h>


using namespace std;

void WriteArray(std::ostream& o, size_t size, const unsigned char* data,
    const string& name)
{
    if (data == nullptr) {
        o << "  nullptr,  // " << name << ".\n";
    } else {
        assert(size > 0);

        o << "  {\n";
        for (size_t i = 0; i < size - 1; ++i) {
            o << "  " << int(data[i]) << ",\n";
        }
        o << "    " << int(data[size - 1]) << "\n"
          << "  },"
          << "  // " << name << ".\n";
    }
}

void WriteArray(std::ostream& o, size_t size, const unsigned short* data,
    const string& name)
{
    if (data == nullptr) {
        o << "  nullptr,  // " << name << ".\n";
    } else {
        assert(size > 0);

        o << "  {\n";
        for (size_t i = 0; i < size - 1; ++i) {
            o << "  " << data[i] << ",\n";
        }
        o << "    " << data[size - 1] << "\n"
          << "  },"
          << "  // " << name << ".\n";
    }
}

void WriteArray(std::ostream& o, size_t size, const double* data,
    const string& name)
{
    if (data == nullptr) {
        o << "  nullptr,  // " << name << ".\n";
    } else {
        assert(size > 0);

        o << "  // " << name << ".\n  {\n";
        for (size_t i = 0; i < size - 1; ++i) {
            o << "  " << data[i] << ",\n";
        }
        o << "    " << data[size - 1] << "\n"
          << "  },\n";
    }
}

std::ostream& operator<<(std::ostream& o, const gsfSwathBathyPing& ping)
{
    o << "GsfSwathBathyPing(\n"
      << "  {" << ping.ping_time.tv_sec << ", " << ping.ping_time.tv_nsec
      << "},\n"
      << "  " << ping.latitude << ",  // latitude\n"
      << "  " << ping.longitude << ",  // longitude\n"
      << "  " << ping.height << ",  // \n"
      << "  " << ping.sep << ",  // \n"
      << "  " << ping.number_beams << ",  // \n"
      << "  " << ping.center_beam << ",  // \n"
      << "  " << ping.ping_flags << ",  // \n"
      << "  " << ping.reserved << ",  // \n"
      << "  " << ping.tide_corrector << ",  // \n"
      << "  " << ping.gps_tide_corrector << ",  // \n"
      << "  " << ping.depth_corrector << ",  // \n"
      << "  " << ping.heading << ",  // \n"
      << "  " << ping.pitch << ",  // \n"
      << "  " << ping.roll << ",  // \n"
      << "  " << ping.heave << ",  // \n"
      << "  " << ping.course << ",  // \n"
      << "  " << ping.speed << ",  // \n";
      //<< ping.scaleFactors << ",\n";

    WriteArray(o, ping.number_beams, ping.depth, "depth");
    WriteArray(o, ping.number_beams, ping.nominal_depth, "nominal_depth");
    WriteArray(o, ping.number_beams, ping.across_track, "across_track");
    WriteArray(o, ping.number_beams, ping.along_track, "along_track");
    WriteArray(o, ping.number_beams, ping.travel_time, "travel_time");
    WriteArray(o, ping.number_beams, ping.beam_angle, "beam_angle");
    WriteArray(o, ping.number_beams, ping.mc_amplitude, "mc_amplitude");
    WriteArray(o, ping.number_beams, ping.mr_amplitude, "mr_amplitude");
    WriteArray(o, ping.number_beams, ping.echo_width, "echo_width");
    WriteArray(o, ping.number_beams, ping.quality_factor, "quality_factor");
    WriteArray(o, ping.number_beams, ping.receive_heave, "receive_heave");
    WriteArray(o, ping.number_beams, ping.depth_error, "depth_error");
    WriteArray(o, ping.number_beams, ping.across_track_error,
        "across_track_error");
    WriteArray(o, ping.number_beams, ping.along_track_error, "along_track_error");
    WriteArray(o, ping.number_beams, ping.quality_flags, "quality_flags");
    WriteArray(o, ping.number_beams, ping.beam_flags, "beam_flags");
    WriteArray(o, ping.number_beams, ping.signal_to_noise, "signal_to_noise");
    WriteArray(o, ping.number_beams, ping.beam_angle_forward,
        "beam_angle_forward");
    WriteArray(o, ping.number_beams, ping.vertical_error, "vertical_error");
    WriteArray(o, ping.number_beams, ping.horizontal_error, "horizontal_error");
    WriteArray(o, ping.number_beams, ping.sector_number, "sector_number");
    WriteArray(o, ping.number_beams, ping.detection_info, "detection_info");
    WriteArray(o, ping.number_beams, ping.incident_beam_adj, "incident_beam_adj");
    WriteArray(o, ping.number_beams, ping.system_cleaning, "system_cleaning");
    WriteArray(o, ping.number_beams, ping.doppler_corr, "doppler_corr");
    WriteArray(o, ping.number_beams, ping.sonar_vert_uncert, "sonar_vert_uncert");
    o << "  " << ping.sensor_id << ",  // sensor_id.\n";
    o << "Brb inten?" << (ping.brb_inten != nullptr) << "\n";
    // TODO(schwehr) gsfSensorSpecific sensor_data.
    // WriteArray(o, ping.number_beams, ping.brb_inten, "brb_inten");  //
    // gsfBRBIntensity

    o << ");\n\n";
    return o;
    // return o << " (" << position.lng_deg << ", " << position.lat_deg << ")";
}



int main(int argc, char** argv)
{
    int handle;
    //int result = gsfOpen("/home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/DT20081015_221314_gsf", GSF_READONLY, &handle);
    //if (gsfOpen("/home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/DT20081015_221314_gsf/20081015_2213-001.gsf", GSF_READONLY, &handle) != 0 || handle < 0)
    //if (gsfOpen("/home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/renav20090218_DT/20081015_2213-001_nav.gsf", GSF_READONLY, &handle) != 0 || handle < 0)
    if (gsfOpen(argv[1], GSF_READONLY, &handle) != 0 || handle < 0)
    {
        cout << "File could not be opened!" << endl;
        exit(0);
    }
    //cout << "Result: " << result << ", handle: " << handle << endl;

    gsfDataID data_id;
    gsfRecords records;

/*
#define GSF_RECORD_HEADER                                   1u
#define GSF_RECORD_SWATH_BATHYMETRY_PING                    2u
#define GSF_RECORD_SOUND_VELOCITY_PROFILE                   3u
#define GSF_RECORD_PROCESSING_PARAMETERS                    4u
#define GSF_RECORD_SENSOR_PARAMETERS                        5u
#define GSF_RECORD_COMMENT                                  6u
#define GSF_RECORD_HISTORY                                  7u
#define GSF_RECORD_NAVIGATION_ERROR                         8u 
#define GSF_RECORD_SWATH_BATHY_SUMMARY                      9u
#define GSF_RECORD_SINGLE_BEAM_PING                         10u
#define GSF_RECORD_HV_NAVIGATION_ERROR                      11u 
#define GSF_RECORD_ATTITUDE                                 12u
*/
    int counter = 0;
    while (gsfRead(handle, GSF_NEXT_RECORD, &data_id, &records, nullptr, 0) != -1) { // && counter < 1000) {
        switch (data_id.recordID) {
        case (GSF_RECORD_NAVIGATION_ERROR):
            cout << "Got navigation error" << endl;
            break;
        case (GSF_RECORD_SWATH_BATHY_SUMMARY):
            cout << "Got swath bathymetry summary" << endl;
            break;
        case (GSF_RECORD_HISTORY):
            cout << "Got record history" << endl;
            break;
        case (GSF_RECORD_SENSOR_PARAMETERS):
            cout << "Got sensor parameters" << endl;
            break;
        case (GSF_RECORD_COMMENT):
            cout << "Got comment" << endl;
            break;
        case (GSF_RECORD_HEADER):
            cout << "Got header" << endl;
            break;
        case (GSF_RECORD_SWATH_BATHYMETRY_PING):
            cout << "Got swath bathymetry ping" << endl;
            //cout << (gsfSwathBathyPing&)records.mb_ping << endl;
            /*if (records.mb_ping.depth_corrector != 0) {
                cout << "Got depth corrector: " << records.mb_ping.depth_corrector << endl;
            }*/
            break;
        case (GSF_RECORD_SOUND_VELOCITY_PROFILE):
            cout << "Got sound velocity profile" << endl;
            break;
        case (GSF_RECORD_PROCESSING_PARAMETERS):
            cout << "Got processing parameters" << endl;
            break;
        case (GSF_RECORD_SINGLE_BEAM_PING):
            cout << "Got single beam ping" << endl;
            break;
        case (GSF_RECORD_ATTITUDE):
            //cout << "Got attitude" << endl;
            break;
        case (GSF_RECORD_HV_NAVIGATION_ERROR):
            cout << "Got HV navigation error" << endl;
            break;
        default:
            cout << "Got unrecognized record" << endl;
            break;
        }
        ++counter;
    }

    return 0;
}

