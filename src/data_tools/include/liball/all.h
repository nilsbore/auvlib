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

#ifndef ALL_H
#define ALL_H

#ifdef _MSC_VER
  #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
  #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

// this is shared as the first part of all .all datagrams
PACK(struct all_common_header {
    unsigned int bytes; // Number of bytes in datagram
    unsigned char start_id; // Start identifier = STX (Always 02h)
    unsigned char data_type; // Type of datagram = X (58h, 88d)
});

PACK(struct all_common_end {
    // end of repeat cycle
	unsigned char end_ident; // End identifier = ETX (Always 03h)
	unsigned short checksum; // Check sum of data between STX and ETX
});

PACK(struct all_xyz88_datagram {
    // common header, see all_common_header

    // data description
    unsigned short model_nbr; // EM model number (Example: EM 710 = 710)
    unsigned int date; // Date = year*10000 + month*100 + day (Example: Sep 26, 2005 = 20050926)
    unsigned int time; // Time since midnight in milliseconds (Example: 08:12:51.234 = 29570234)
    unsigned short ping_count; // Ping counter (sequential counter)
    unsigned short serial_nbr; // System serial number
    unsigned short heading; // Heading of vessel (at TX time) in 0.01 degress
    unsigned short sound_vel; // Sound speed at transducer in dm/s
    float transducer_depth; // Transmit transducer depth in m re water level at time of ping
    unsigned short nbr_beams; // Number of beams in datagram = N
    unsigned short nbr_valid; // Number of valid detections
    float sampling_freq; // Sampling frequency in Hz
    unsigned char scanning_info; // Scanning info.
    unsigned char spare[3]; // Spare

    // repeat cycle, see all_xyz88_datagram_repeat

    // end of repeat cycle, see all_common_end
});

PACK(struct all_position_datagram {
    // common header, see all_common_header

    // data description
    unsigned short model_nbr; // EM model number (Example: EM 710 = 710)
    unsigned int date; // Date = year*10000 + month*100 + day (Example: Sep 26, 2005 = 20050926)
    unsigned int time; // Time since midnight in milliseconds (Example: 08:12:51.234 = 29570234)
    unsigned short pos_count; // Position counter (sequential counter)
    unsigned short serial_nbr; // System serial number

    int latitude; // Latitude in decimal degrees*20000000 (negative 4S if southern hemisphere) (Example: 32°34’ S = -651333333) 
	int longitude; // Longitude in decimal degrees*10000000 (negative 4S if western hemisphere) (Example: 110.25° E = 1102500000 )

	unsigned short fix_quality; // Measure of position fix quality in cm
	unsigned short speed_over_ground; // Speed of vessel over ground in cm/s
	unsigned short course_over_ground; // Course of vessel over ground in 0.01 deg
	unsigned short heading; // Heading of vessel in 0.01 deg
	unsigned char pos_sys_descriptor; // Position system descriptor
	unsigned char nbr_bytes_input; // Number of bytes in input datagram

    // Position input datagram as received

    // end of repeat cycle, see all_common_end
});

PACK(struct all_depth_datagram {
    // common header, see all_common_header

    // data description
    unsigned short model_nbr; // EM model number (Example: EM 710 = 710)
    unsigned int date; // Date = year*10000 + month*100 + day (Example: Sep 26, 2005 = 20050926)
    unsigned int time; // Time since midnight in milliseconds (Example: 08:12:51.234 = 29570234)
    unsigned short height_count; // Height counter (sequential counter)
    unsigned short serial_nbr; // System serial number

    int height; // Height in cm
    unsigned char height_type; // Height type

    // end of repeat cycle, see all_common_end
});

PACK(struct all_echosounder_depth_datagram {
    // common header, see all_common_header

    // data description
    unsigned short model_nbr; // EM model number (Example: EM 710 = 710)
    unsigned int date; // Date = year*10000 + month*100 + day (Example: Sep 26, 2005 = 20050926)
    unsigned int time; // Time since midnight in milliseconds (Example: 08:12:51.234 = 29570234)
    unsigned short echo_count; // Echo sounder counter (sequential counter)
    unsigned short serial_nbr; // System serial number

    unsigned int meas_date; // Date = year*10000 + month*100 + day (Example: Sep 26, 2005 = 20050926)
    unsigned int meas_time; // Time since midnight in milliseconds (Example: 08:12:51.234 = 29570234)
    unsigned int echo_depth; // Echo sounder depth from waterline in cm
    char source_id; // Source identifier (S, T, 1, 2 or 3)

    // end of repeat cycle, see all_common_end
});

// repeats nbr_beams times as given in all_xyz88_datagram
PACK(struct all_xyz88_datagram_repeat {
    // repeat cycle data
    float depth; // Depth (z) from transmit transducer in m
	float across_track; // Acrosstrack distance (y) in m
	float along_track; // Alongtrack distance (x) in m
	unsigned short detection_window; // Detection window length in samples
	unsigned char quality_factor; // Quality factor
	char iba; // Beam incidence angle adjustment (IBA) in 0.1 deg
	unsigned char detection_info; // Detection information
	char rt_cleaning_info; // Real time cleaning information
	short reflectivity; // Reflectivity (BS) in 0.1 dB resolution (Example: –20.1 dB = FF37h= 65335)
});

PACK(struct all_attitude_datagram {
    unsigned short model_nbr; // EM model number (Example: EM 710 = 710)
    unsigned int date; // Date = year*10000 + month*100 + day (Example: Sep 26, 2005 = 20050926)
    unsigned int time; // Time since midnight in milliseconds (Example: 08:12:51.234 = 29570234)
    unsigned short attitude_count; // Echo sounder counter (sequential counter)
    unsigned short serial_nbr; // System serial number

    unsigned short nbr_entries; // = N 2U 1 – —
    //Repeat cycle – N entries of: 12*N —
});

PACK(struct all_attitude_datagram_repeat {
    unsigned short ms_since_start; // Time in milliseconds since record start 2U 0 to 65534 —
    unsigned short sensor_status; // Sensor status 2U — 1
    short roll; // Roll in 0.01° 2S -18000 to 18000 —
    short pitch; // Pitch in 0.01° 2S -18000 to 18000 —
    short heave; // Heave in cm 2S -1000 to 10000 —
    unsigned short heading; //– Heading in 0.01° 2U 0 to 35999 —
});

#undef PACK

#endif // ALL_H
