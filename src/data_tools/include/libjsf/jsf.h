/* Copyright 2019 Yiping Xie (yipingx@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include<stdint.h>

#ifndef JSF_H
#define JSF_H

#ifdef _MSC_VER
  #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
  #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif


// 16-byte header
PACK(struct jsf_msg_header {

    uint16_t start_marker; // Marker for the Start of Header (Always 0x1601)
    uint8_t prot_ver; // Version number of protocol used (e.g.10)
    uint8_t sess_id; // Session Identifier, can be ignored
    uint16_t msg_type; // Message Type (e.g. 80 = Sonar Trace Data)
    uint8_t cmd_type; // Command Type (2 = Normal data source)
    uint8_t subsystem_num; // Subsystem Number, 0 = Sub-bottom, 20 = Single or Lower Frequency Side Scan, 21 = Higher Frequency Side Scan
    uint8_t channel_num; // Channel for a Multi-Channel Subsystem, For Side Scan Subsystem, 0 = Port, 1 = Startboard; For Serial Ports: Port #
    uint8_t seq_num; // Sequence Number, can be ingored
    uint16_t reserved; // Reserved, can be ingored
    uint32_t following_bytes; // Size of followiing Message in Bytes
});


// Message Type: 80 Sonar Data Message (240-byte header)
PACK(struct jsf_sonar_data_msg_header{

    // 0-43 Bytes
    int32_t ping_time_in_sec; //Ping Time in seconds (since the start of time based on time() function) (1/1/1970)
    uint32_t starting_depth; // Srarting Depth (window offset) in samples, usually zero
    uint32_t ping_num; // Ping Number (increments with each ping)
    int16_t reserved_0[2]; // Reserved, do not use
    uint16_t msbs; // Most Significant Bits
    int16_t reserved_1[5]; // Reserved, do not use
    int16_t id_code; // ID Code (always 1),  = Seismic Data
    uint16_t val_flag; // Validiry Flag
    uint16_t reserved_2; // Reserved, do not use
    int16_t data_format; // 0 = 1 short per sample - Envelope Dta; 1 = 2 shorts per sample - Analytic Signal Data (Real, Imaginary)
    int16_t dis_frm_ant_to_tow_point; // Distance from Antenna to Tow point in Centimeters
    int16_t dis_frm_ant_to_tow_point_stbd; // Distance from Antenna to Tow point in Centimeters, Starboard
    int16_t reserved_3[2]; // Reserved, do not use
    
    // 44-89 Bytes: Navigation Data
    // assert(CHAR_BIT * sizeof (float) == 32);
    float pipe_in_km; // Kilometer of pipe 
    int16_t reserved_4[16]; // Reserved
    int32_t x_coord; // X in millimeters or in decimeters or Longitude in 10000 * (Minutes of Arc)
    int32_t y_coord; // Y in millimeters or in decimeters or Latitude in 10000 * (Minutes of Arc)
    int16_t coord_units; // Coordinate Uints; 1 = X, Y in millimeters; 2 = Longitud, Latitude in minutes of arc times 10000; 3 = X, Y in decimeters

    // 90-155 Bytes: Pulse Information
    uint8_t anno_str[24]; // Annotation String (ASCII Data)
    uint16_t spls_num_in_pkt; // Number of data samples in this packet
    uint32_t spl_intvl_in_ns; // Sampling Interval in Nanoseconds
    uint16_t gain_adc; // Gain factor of ADC
    int16_t usr_tx_lvl; // User Transmit Level Setting (0-100) percent
    int16_t reserved_5; // Reserved
    uint16_t tx_pulse_starting_freq_in_dahz; // Transmit pulse starting frequency in dacahertz (daHz) (uints of 10Hz)
    uint16_t tx_pulse_ending_freq_in_dahz; // Transmit pulse ending frequency in dacahertz (daHz) (uints of 10Hz)
    uint16_t sweep_len_in_ms; // Sweep Length in milliseconds
    int32_t pressure_in_mpsi; // Pressure in milliPSI (1 uint = 1/1000 PSI)
    int32_t depth_in_mm; // Depth in millimeters (if not = 0)
    uint16_t spl_freq_in_hz; // Sample Frequency of the data in hertz, modulo 65536 NOTE *
    uint16_t out_pulse_id; // Outgoing pulse identifier
    int32_t al_in_mm; // Altitude in millimeters (if bottom tracking valid) 0 implies not filed
    float sound_speed_in_m_per_s; // Sound Speed in meters per second
    float mixer_freq_in_hz; // Mixer Frequency in Hertz

    // 156-167 Bytes: CPU Time
    int16_t cpu_year; // Year (e.g. 2009) (Should not be used)
    int16_t cpu_day; // Day (1-366) (should not be used)
    int16_t cpu_hour; // Hour (Should not be used)
    int16_t cpu_min; // Minute (Should not be used)
    int16_t cpu_sec; // Sec (Should not be used)
    int16_t time_basis; // Time Basis (always 3)

    // 168-171 Bytes: Weighting Factor
    int16_t weighting_factor_n; // Weighting Factor N (Signed Value!) Defined as 2-N
    int16_t pulse_num_in_water; // Number of pulses in the water

    // 172-179 Bytes: Orientation Sensor Data
    uint16_t compass_heading; // Compass Heading (0 to 360) in uints of 1/100 degree
    int16_t pitch; // Pitch: Scale by 180/32768 to get degrees, + = bow up
    int16_t roll; // Roll: Scale by 180/32768 to get degrees, + = port up
    int16_t towfish_elec_temp; // Tow fish electronics Temperature, in uint of 1/10th degree C

    // 180-185 Bytes: Miscellaneous Data
    int16_t reserved_6; // Reserved
    int16_t trigger_source; // Trigger Source; 0 = Internal; 1 = External; 2 = Coupled
    uint16_t mark_num; // Mark Number; 0 = No Mark

    // 186-199 Bytes: NMEA Navigation Data
    int16_t nav_hour; // Hour (0-23)
    int16_t nav_min; // Minutes(0-59)
    int16_t nav_sec; // Seconds(0-59)
    int16_t course_in_degree; // Course in Degrees (0-360) Factional portion in LSB
    int16_t speed; // Speed in tenths of a knot for an additional factional digit
    int16_t nav_day; // Day (1-366)
    int16_t nav_year; // Year

    // 200-239 Bytes: Other Miscellaneous Data
    uint32_t today_in_ms; // Milliseconds today (since midnight) (use in conjunction with Year/Day to get time of Ping)
    uint16_t max_abs_value_adc; // Maximum Absolute Value of ADC samples in this packet
    int16_t reserved_7[2]; // Reserved
    int8_t sonar_software_ver[6]; // Sonar Software Version Number - ASCII
    int32_t init_spher_corr_factor; // Initial Spherical Correction Factor (Useful for multi-ping / deep application) * 100
    uint16_t packet_num; // Packet Number; Each ping starts with packet 1
    int16_t ad_decimation_times_100; // 100 times the A/D Decimation Factor
    int16_t decimation_factor; // Decimation Factor after the FFT
    int16_t water_temp; // Water Temperature in uints of 1/10 degree C
    float layback_in_m; // Layback in meters
    int32_t reserved_8; // Reserved
    uint16_t cable_out_in_dm; // Cable Out in Decimeters
    uint16_t reserved_9; // Reserved

});

// Message Type: 2080 Doppler Velocity Log Data (DVL)
PACK(struct jsf_dvl_msg_header{
    int32_t time_in_sec; // Time in seconds (since the start of time based on time() function)(1/1/1970)
    int32_t ms_in_cur_sec; // Milliseconds in the current second
    uint8_t reserved_0[4]; // Reserved
    uint32_t flag; // Flags. Indicates which values are present
    int32_t dist_to_bottom_in_cm[4]; // 4 Intergers: Disctance to bottom in cm for up to 4 beams
    int16_t x_vel_wrt_bottom; // X Velocity with respect to the bottom in mm/second; Positive => Starboard or East; -32768 indicates an invalid reading
    int16_t y_vel_wrt_bottom; // Y Velocity; Positive => Forward or North (mm/second)
    int16_t z_vel_wrt_bottom; // Z Velocity; Positive => Upward  (mm/second)
    int16_t x_vel_wrt_water; // X Velocity with respect to a water layer in mm/second; Positive => Starboard or East; 
    int16_t y_vel_wrt_water; // Y Velocity; Positive => Forward or North
    int16_t z_vel_wrt_water; // Z Velocity; Positive => Upward 
    uint16_t depth_in_dm; // Depth from depth sensor in decimeters
    int16_t pitch; // Pitch -180 to +180 degree (units = 0.01 of a degree) + Bow up
    int16_t roll; // Roll -180 to +180 degree (units = 0.01 of a degree) + Port up
    uint16_t heading; // Heading 0 to 360 degree (units = 0.01 of a degree) 
    uint16_t salinity; // Salinity in 1 part per thousand
    int16_t temp; // Temperature in uints of 1/100 of a degree Celsius
    int16_t sound_vel; // Sound velocity in meters per second
    int16_t reserved_1[7]; // Reserved

});


// Message Type: 2090 Situation Message
PACK(struct jsf_situ_msg_header{
    int32_t time_in_sec; // Time in seconds (since the start of time based on time() function) (1/1/1970)
    int32_t ms_in_cur_sec; // Milliseconds in the current second
    int8_t reserved_0[4]; // Reserved
    uint32_t validity_flag; // Validity Flags. Indicates which of the following fields are valid
    uint8_t reserved_1[4]; // Reserved
    uint64_t timestamp_us; // Microsecond timestamp, us since 12:00 am GMT, January 1, 1970
    double latitude; // Latitude in degrees, north is positive
    double longitude; // Longitude in degrees, east is positive
    double depth; // Depth in meters
    double heading; // Heading in degrees
    double pitch; // Pitch in degrees
    double roll; // Roll in degrees
    double x_rel_pos; // X, forward, relative position in meters, surge
    double y_rel_pos; // Y, starboard, relative position in meters, sway
    double z_rel_pos; // Z, downward, relative position in meters, heave
    double x_vel; // X, forward, velocity in meters per second
    double y_vel; // Y, starboard, velocity in meters per second
    double z_vel; // Z, downward, velocity in meters per second
    double north_vel; // North velocity in meters per second
    double east_vel; // East velocity in meters per second
    double down_vel; // Down velocity in meters per second
    double x_ang_rate; // X angular rate in degrees per second port up is positive 
    double y_ang_rate; // Y angular rate in degrees per second bow up is positive 
    double z_ang_rate; // Z angular rate in degrees per second starboard is positive 
    double x_acc; // X, forward, acceleration in meters per second per second
    double y_acc; // Y, starboard, acceleration in meters per second per second
    double z_acc; // Z, downward, acceleration in meters per second per second
    double latitude_std; // Latitude standard deviation in meters
    double longitude_std; // Longitude standard deviation in meters
    double depth_std; // Depth standard deviation in meters
    double heading_std; // Heading standard deviation in degrees
    double pitch_std; // Pitch standard deviation in degrees
    double roll_std; // Roll standard deviation in degrees
    uint16_t reserved_2[16]; // Reserved

});



#undef PACK
#endif // JSF_H