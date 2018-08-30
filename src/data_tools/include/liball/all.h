#ifndef ALL_H
#define ALL_H

// this is shared as the first part of all .all datagrams
struct all_common_header {
    unsigned int bytes; // Number of bytes in datagram
    unsigned char start_id; // Start identifier = STX (Always 02h)
    unsigned char data_type; // Type of datagram = X (58h, 88d)
} __attribute__((packed));

struct all_common_end {
    // end of repeat cycle
	unsigned char spare; // Spare (always 0)
	unsigned char end_ident; // End identifier = ETX (Always 03h)
	unsigned short checksum; // Check sum of data between STX and ETX
} __attribute__((packed));

struct all_xyz88_datagram {
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
} __attribute__((packed));

struct all_position_datagram {
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
} __attribute__((packed));

// repeats nbr_beams times as given in all_xyz88_datagram
struct all_xyz88_datagram_repeat {
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
} __attribute__((packed));

#endif // ALL_H
