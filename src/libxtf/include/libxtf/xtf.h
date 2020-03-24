#ifndef XTF_H
#define XTF_H

/* Updated October 27, 1998

Triton .XTF file format.  See also the source file DEMO_XTF.C for
example code.  DEMO_XTF.C includes this file for structure
definitions.


1. Overview
-----------
The XTF file format (eXtended Triton Format) was created to answer
the need for saving many different types of sonar, navigation,
telemetry and bathymetry information.  The format can easily be
extended to include various types of data that may be encountered in
the future.


2. Methodology
--------------
An XTF file can be thought of as a "pool" of data.  In a realtime
collection environment, data may be prepared and added to the file
at any time without regard to synchronization between data packets. 
For example, bathymetry data may be logged five times per second
while sonar data is being logged at 10 times per second.  No storage
space is wasted and no "holes" are created in the saved data stream.

While processing an XTF file, the processing software can easily
ignore data packets it doesn't recognize or isn't needed.  For
example, Triton's "Target" program will read an XTF file for sonar
data and skip over any saved bathymetry data.  When a non-sonar data
packet is encountered it simply ignores it and reads in another
packet.  Any software that reads XTF files should also do this
because it guarantees compatibility with files that may contain new
kinds of data that may be included in the future.

Since the "pool" of data in an XTF file is written asynchronously, it
is impossible to calculate a byte offset for a specific record in the
file.  However, there is a straightforward method to quickly search a 
file for any specific data packet.  This method is described later in
this document.


3. Binary data representation
-----------------------------
The following definitions apply to all data written in XTF files:

        float   4-byte single-precision IEEE standard
        double  8-byte double-precision IEEE standard
        short   2-byte signed value
        int     4-byte signed value
        WORD    2-byte unsigned value
        long    4-byte signed value
        DWORD   4-byte unsigned value
        char    1-byte signed value
        BYTE    1-byte unsigned value

All data is written with Intel 80x86 byte ordering (LSB, MSB).  If
an XTF file is to be processed on a Sun, Silicon Graphics or Apple
computer, the order of the bytes in all values must be exactly
reversed.  For example, a float value (4 bytes) would need to be
re-ordered from (1,2,3,4) to (4,3,2,1) in the target machine's memory
before treating the number as a floating-point value.  This
effe6tively converts the value from little-endian (least-significant
byte first) to big-endian (most-significant byte first).


4. Structure
------------
Each XTF file begins with a file header record.  This record is given
by the XTFFILEHEADER structure (below) and is at least 1024
bytes in length.  It can be larger than 1024 bytes when the total
number of channels to be stored in the file is greater than six (the 
number of CHANINFO structures contained in the XTFFILEHEADER structure
causes the XTFFILEHEADER structure to grow larger than 1024 bytes). 
In this event, the total size of the file header record grows in
increments of 1024 bytes until there is enough room to hold all of
the CHANINFO structures.  Two important elements of the file header
are:
        - Number of sonar channels
        - Number of bathymetry channels

These are used to determine how many CHANINFO structures will be 
in the header record.  The CHANINFO structures for all of the sonar
channels will always preceed the structures for the bathymetry
channels.

After the header, data packets may follow in any order.  Each
packet must contain a packet header, which identifies the type of
packet and the number of bytes in the packet.  If the software does
not recognize the packet type, it can skip forward the given number
of bytes to the next packet.  Thus, the packets can be thought of as
a linked list of data elements which vary in size.  To make file
searching easier, all data packets must be an even multiple of 64 
bytes in length.  A data packet typically has the following format:

        - Packet Header (usually 256 bytes).
             (Identifies number of channels in this packet
             and total size of the packet.)
             Each packet begins with a key pattern of bytes,
             called the "magic number", which can be used to 
			 align the data stream to the start of a packet.
          For each channel,
            - Channel header (optional, usually 64 bytes)
            - Channel data (optional, byte count varies)
               
The various kinds of XTF data packets are described below.

XTF_HEADER_SONAR
   - 256 byte ping header (XTFPINGHEADER) which holds total number 
     of bytes for this packet, time, date, towfish telemetry, 
     navigation, etc.  This structure has exactly the same format
     as the XTF_HEADER_BATHY structure.
   - for each channel associated with this ping,
       - 64 byte channel header (XTFPINGCHANHEADER) which holds
         channel ID, number of bytes in the channel data, period, 
         gain codes and processing flags
       - Actual channel data.  Byte count is determined by the
         NumSamples values in the preceeding channel header multiplied
         by the number of bytes per sample, as determined in the 
         1024-byte file header.
     
XTF_HEADER_NOTES
   - 256 byte structure (XTFNOTESHEADER) which holds date, time,
     and ASCII text.  Text that appears in this strcture will be
     displayed in the Isis Parameter window during playback.

XTF_HEADER_BATHY
   - 256 byte header (XTFBATHHEADER) which holds total number 
     of bytes for this packet, time, date, towfish telemetry, 
     navigation, etc.
   - 256 byte packet holds the Bathy data.  Currently, bathy data
     is of the format given by the Reson SeaBat or Odom Echoscan II.
     Refer to the documentation from those companies for details
     on the format for their binary packets.  Isis will store the
     exact binary R-Theta or RI-Theta packeta from these multibeam 
     system as received over the serial port(s).
     
     The structure of XTF_HEADER_BATHY is the same as that for
     XTF_HEADER_SONAR.

XTF_HEADER_ATTITUDE
   - 64 byte header (XTFATTITUDEDATA) which contains Pitch,
     Roll, Heave, Yaw and a time reference given in milliseconds.
     This time reference can be used to match the SeaBat 
     bathymetry data which also contains the same time reference.

XTF_HEADER_ELAC
   - 256 byte header (XTFBATHHEADER) which holds total number 
     of bytes for this packet, time, date, towfish telemetry, 
     navigation, etc.
   - 256 byte packet holds the Bathy data.  Currently, bathy data
     is of the format given by the ELAC BottomChart system.
     Refer to the documentation from ELAC for details
     on the format for their binary data.

XTF_HEADER_RAW_SERIAL     
   - 64-byte header (XTFNOTESHEADER) holds time, date, and
     the exact ASCII data as received from any serial port which
     has ASCII RAW data logging enabled.  This packet is used 
     primarily for diagnostic and system checking.  Use of this
     packet is not generally recommended.

***********************************************************************/
/* Include-file for reading .XTF files.  This file defines the
   structures currently defined in the .XTF file format.
*/

#define XTF_HEADER_SONAR                     0 // sidescan and subbottom
#define XTF_HEADER_NOTES                     1 // notes - text annotation
#define XTF_HEADER_BATHY                     2 // bathymetry (Seabat, Odom)
#define XTF_HEADER_ATTITUDE                  3 // TSS or MRU attitude (pitch, roll, heave, yaw)
#define XTF_HEADER_FORWARD                   4 // forward-look sonar (polar display)
#define XTF_HEADER_ELAC                      5 // Elac multibeam
#define XTF_HEADER_RAW_SERIAL                6 // Raw data from serial port
#define XTF_HEADER_EMBED_HEAD                7 // Embedded header structure
#define XTF_HEADER_HIDDEN_SONAR              8 // hidden (non-displayable) ping
#define XTF_HEADER_SEAVIEW_ANGLES            9 // Bathymetry (angles) for Seaview
#define XTF_HEADER_SEAVIEW_DEPTHS           10 // Bathymetry from Seaview data (depths)
#define XTF_HEADER_HIGHSPEED_SENSOR         11 // used by Klein (Cliff Chase) 0=roll, 1=yaw
#define XTF_HEADER_ECHOSTRENGTH             12 // Elac EchoStrength (10 values)
#define XTF_HEADER_GEOREC                   13 // Used to store mosaic params
#define XTF_HEADER_K5000_BATHYMETRY         14 // Bathymetry data from the Klein 5000
#define XTF_HEADER_HIGHSPEED_SENSOR2        15 // High speed sensor from Klein 5000

#define FMT_XTF 123 // unique ID for all XTF files.

typedef int                 BOOL;
#define FALSE		    0
#define TRUE		    (!FALSE)

typedef unsigned char	    BYTE;
typedef unsigned short      WORD;
#ifdef _MSC_VER
  typedef unsigned long       DWORD;
  typedef signed long         LONG;
#else
  typedef unsigned int        DWORD;
  typedef int                 LONG;
#endif
typedef unsigned int	    UINT;

#ifdef _MSC_VER
  #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
  #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

// Channel information structure (contained in the file header).
// One-time information describing each channel.  64 bytes long.
// This is data pertaining to each channel that will not change
// during the course of a run.
///////////////////////////////////////////////////////////////////////////////
PACK(typedef struct {
   BYTE TypeOfChannel;     // PORT, STBD, SBOT or BATH
   BYTE SubChannelNumber;
   WORD CorrectionFlags;   // 1=raw, 2=Corrected
   WORD UniPolar;          // 0=data is bipolar, 1=data is unipolar

   WORD BytesPerSample;    // 1 or 2
   DWORD Reserved;         // Previously this was SamplesPerChannel.  Isis now supports
                           // the recording of every sample per ping, which means that
                           // number of samples per channel can vary from ping to ping
                           // if the range scale changes.  Because of this, the 
                           // NumSamples value in the XTFPINGCHANHEADER strcture (below)
                           // holds the number of samples to read for a given channel.
                           // For standard analog systems, this Reserved value is still
                           // filled in with 1024, 2048 or whatever the initial value is
                           // for SamplesPerChannel.

   char ChannelName[16];   // Text describing channel.  i.e., "Port 500"

   float VoltScale;        // How many volts is represented by max sample value.  Typically 5.0.
   float Frequency;        // Center transmit frequency
   float HorizBeamAngle;   // Typically 1 degree or so
   float TiltAngle;        // Typically 30 degrees
   float BeamWidth;        // 3dB beam width, Typically 50 degrees

                           // Orientation of these offsets:
                           // Positive Y is forward
                           // Positive X is to starboard
                           // Positive Z is down.  Just like depth.
                           // Positive roll is lean to starboard
                           // Positive pitch is nose up
                           // Positive yaw is turn to right

   float OffsetX;          // These offsets are entered in the 
   float OffsetY;          // Multibeam setup dialog box.
   float OffsetZ;

   float OffsetYaw;        // If the multibeam sensor is reverse
                           // mounted (facing backwards), then
                           // OffsetYaw will be around 180 degrees.
   float OffsetPitch;      
   float OffsetRoll;

   WORD  BeamsPerArray;    // For forward look only (i.e., Sonatech DDS)

   char ReservedArea2[54];

}) CHANINFO;


   
// XTF File header.
// Total of 1024 bytes.
///////////////////////////////////////////////////////////////////////////////
PACK(typedef struct {
   BYTE FileFormat;        // 50 for Q-MIPS file format, 51 for Isis format
   BYTE SystemType;        // Type of system used to record this file.  202=Isis
   char RecordingProgramName[8];    // Example: "Isis"
   char RecordingProgramVersion[8]; // Example: "1.72"
   char SonarName[16];     // Name of server used to access sonar.  Example: "C31_SERV.EXE"
   WORD SonarType;         // K2000=5, DF1000=7, SEABAT=8
   char NoteString[64];    // Notes as entered in the Sonar Setup dialog box
   char ThisFileName[64];  // Name of this file. Example:"LINE12-B.XTF"

   WORD NavUnits;          // 0=METERS or 3=DEGREES

   WORD NumberOfSonarChannels;  // if > 60, header goes to 8K in size
   WORD NumberOfBathymetryChannels;
   WORD NumberOfForwardLookArrays;
   WORD NumberOfEchoStrengthChannels;
   WORD Reserved1;
   WORD Reserved2;
   WORD Reserved3;
   WORD Reserved4;

   // nav system parameters
   ///////////////////////////
   BYTE     ProjectionType[12];       // Not currently used
   BYTE     SpheriodType[10];         // Not currently used
   LONG     NavigationLatency;        // milliseconds, latency of nav system (usually GPS)
                                      // This value is entered on the
                                      // Serial port setup dialog box.  
                                      // When computing a position, Isis will
                                      // take the time of the navigation 
                                      // and subtract this value.

   float    OriginY;                  // Not currently used
   float    OriginX;                  // Not currently used

                                      // Orientation of these offsets:
                                      // Positive Y is forward
                                      // Positive X is to starboard
                                      // Positive Z is down.  Just like depth.
                                      // Positive roll is lean to starboard
                                      // Positive pitch is nose up
                                      // Positive yaw is turn to right

   float    NavOffsetY;               // These offsets are entered in
   float    NavOffsetX;               // the multibeam setup dialog box.
   float    NavOffsetZ;               
   float    NavOffsetYaw;

   float    MRUOffsetY;               // These offsets are entered in
   float    MRUOffsetX;               // the multibeam setup dialog box
   float    MRUOffsetZ;               

   float    MRUOffsetYaw;          
   float    MRUOffsetPitch;
   float    MRUOffsetRoll;        

                          // note: even 128-byte boundary to here

   CHANINFO ChanInfo[6];  // Each CHANINFO struct is 128 bytes.
                          // If more than 6 channels needed, header record
                          // grows 1K in size for each additional 8 channels.
}) XTFFILEHEADER;


// The XTFATTITUDEDATA structure used to store information from a TSS or 
// MRU motion sensor device.  This is usually high-resolution data
// (updating 20 times per second or more) and is needed when processing
// multibeam bathymetric data.  When TSS or MRU is selected as a serial device,
// the data is received and decoded.  As the attitude information is decoded,
// the values are filled into the following structure and then saved to the 
// XTF file.
//
// Attitude data packet, 64 bytes in length.
///////////////////////////////////////////////////////////////////////////////
PACK(typedef struct {
   // 
   // Type of header
   //
   WORD MagicNumber;      // Always set to 0xFACE
   BYTE HeaderType;       // XTF_HEADER_ATTITUDE (3)
   BYTE SubChannelNumber; // When HeaderType is Bathy, indicates which head
   WORD NumChansToFollow; // If Sonar Ping, Number of channels to follow
   WORD Reserved1[2];

   DWORD NumBytesThisRecord; // Total byte count for this ping including this ping header
                             // Note: Isis records data packets in multiples of 64 bytes.
							 // If the size of the data packet is not an exact multiple of
							 // 64 bytes, zeros are padded at the end packet and this value
							 // will be promoted to the next 64-byte granularity.  In all
							 // cases, this value will be the EXACT size of this packet
							 //
   DWORD Reverved2[4];

   // will be followed by attitude data even to 64 bytes
   float Pitch;   // positive value is nose up
   float Roll;    // positive value is roll to starboard
   float Heave;   // positive value is sensor up
                  // Note: The TSS sends heave positive up.  The MRU
                  // sends heave positive down.  In order to make the 
                  // data logging consistent, the sign of the MRU's 
                  // heave is reversed before being stored in this field.

   float Yaw;     // positive value is turn right
   DWORD TimeTag; // time ref. given in milliseconds

   float Heading; // In degrees, as reported by MRU.  
                  // TSS doesn't report heading, so when using a TSS 
                  // this value will be the most recent ship gyro value 
                  // as received from GPS or from any serial port using 
                  // 'G' in the template.

   BYTE  Reserved3[10];

}) XTFATTITUDEDATA;


// Sonar or Bathy Ping header
// The data here can change from ping to ping but will pertain to all
// channels that are at the same time as this ping.  256 bytes in length.
///////////////////////////////////////////////////////////////////////////////
PACK(typedef struct {

   // 
   // Type of header
   //
   WORD MagicNumber;      // Set to 0xFACE
   BYTE HeaderType;       // Typically XTF_HEADER_SONAR (0), XTF_HEADER_BATHY (2),
                          // XTF_HEADER_ATTITUDE
   BYTE SubChannelNumber; // When HeaderType is Bathy, indicates which head
                          // When sonar, which ping of a batch (Klein 5000: 0..4)

   WORD NumChansToFollow; // If Sonar Ping, Number of channels to follow
   WORD Reserved1[2];

   DWORD NumBytesThisRecord; // Total byte count for this ping including this ping header

   //
   // Date and time of the ping
   //
   WORD  Year;          // Computer date when this record was saved
   BYTE  Month;
   BYTE  Day;
   BYTE  Hour;          // Computer time when this record was saved
   BYTE  Minute;
   BYTE  Second;
   BYTE  HSeconds;      // hundredths of seconds (0-99)
   WORD  JulianDay;     // Number of days since January 1

   //
   // General information
   //
   WORD CurrentLineID;     // [i] Current line ID from serial port
   WORD EventNumber;       // [O] Last logged event number

   DWORD PingNumber;       // Counts consecutively from 0 and increments 
                           //   for each update.  Note that the
                           //   counters are different between sonar
                           //   and bathymetery updates.

   float SoundVelocity;    // m/s, Round trip, defaults to 750. 
                           //   Can be changed on Isis menu.  This
                           //   value is never computed and can only be 
                           //   changed manually by the user. Also see 
                           //   ComputedSoundVelocity below.

   float OceanTide;        // [{t}] Ocean tide in meters.  Can be
                           // changed by the user on the Configure 
                           // menu in Isis.
   DWORD Reserved2;        // Reserved for future use
   
   //
   // Raw CTD information.  The Freq values are those sent up by the
   // SeaBird CTD.  The Falmouth Scientific CTD sends up computed data.
   float ConductivityFreq; // [Q] Conductivity frequency in Hz
   float TemperatureFreq;  // [b] Temperature frequency in Hz
   float PressureFreq;     // [0] Pressure frequency in Hz
   float PressureTemp;     // [;] Pressure Temperature (Degrees C)

   // 
   // Computed CTD information.  When using a SeaBird CTD, these
   // values are computed from the raw Freq values (above).
   //
   float Conductivity;     // [{c}] Conductivity in S/m can be computed from [Q]
   float WaterTemperature; // [{w}] Water temperature in C, can be computed from [b]
   float Pressure;         // [{p}] Water pressure in psia, can be computed from [0]
   float ComputedSoundVelocity;  // Meters per second, computed from
                                 // Conductivity, WaterTemperature and 
                                 // Pressure using the Chen Millero
                                 // formula (1977) formula (JASA,62,1129-1135).


   //
   // Sensors information
   //
   float MagX;             // [e] X-axis magnetometer data, mgauss
   float MagY;             // [w] Y-axis magnetometer data, mgauss
   float MagZ;             // [z] Z-axis magnetometer data, mgauss

                           // Auxillary values can be used to store
                           // and display any value at the user's
                           // discretion.  The are not used in
                           // any calculation in Isis, Target or Vista.

   float AuxVal1;          // [1] Auxillary value.  Displayed in the 
   float AuxVal2;          // [2] Auxillary value   "Sensors" window
   float AuxVal3;          // [3] Auxillary value   available by selecting
   float AuxVal4;          // [4] Auxillary value   Window->Text->Sensors.
   float AuxVal5;          // [5] Auxillary value
   float AuxVal6;          // [6] Auxillary value

   float SpeedLog;         // [s] Speed log sensor on towfish - knots. This isn't fish speed!
   float Turbidity;        // [|] turbidity sensor (0 to +5 volts) stored times 10000

   //
   // Ship Navigation information.  These values are stored only
   // and are not part of any equation or computation in Isis.
   //
   float ShipSpeed;        // [v] Speed of ship in knots.  Stored
   float ShipGyro;         // [G] Ship gyro in degrees
   double ShipYcoordinate; // [y] Ship latitude or northing
   double ShipXcoordinate; // [x] Ship longitude or easting

   WORD ShipAltitude;      // Decimeters (meters*10, stored only)
   WORD ShipDepth;         // Decimeters (meters*10, stored only)

   // 
   // Sensor Navigation information
   //
   BYTE FixTimeHour;       // [H] Hour of most recent nav update
   BYTE FixTimeMinute;     // [I] Minute of most recent nav update
   BYTE FixTimeSecond;     // [S] Second of most recent nav update
                           // Note that the time of the nav is
                           // adjusted by the NavLatency stored in 
                           // the XTF file header.
   BYTE Reserved4;
   float SensorSpeed;      // [V] Speed of the in knots.  Used for
                           //   speed correction and position calculation.
   float KP;               // [{K}] Kilometers Pipe
   double SensorYcoordinate; // [E] Sensor latitude or northing
   double SensorXcoordinate; // [N] Sensor longitude or easting
                             // Note: when NavUnits in the file header
                             // is 0, values are in meters (northings
                             // and eastings).  When NavUnits is 3,
                             // values are in Lat/Long.  Also see
                             // the Layback value, below.

   //
   // Tow Cable information
   //
   WORD SonarStatus;       // System status value, sonar dependant (displayed in Status window)
   WORD RangeToFish;       // [?] Slant range to fish * 10.  
                           //    Not currently used.
   WORD BearingToFish;     // [>] Bearing to towfish from ship * 100.  
                           //    Not currently used.
   WORD CableOut;          // [o] Amount of cable payed out in meters
                           //    Not currently used in Isis.
   float Layback;          // [l] Distance over ground from ship to fish.
                           //    When this value is non-zero, Isis
                           //    assumes that SensorYcoordinate and 
                           //    SensorXcoordinate need to be
                           //    adjusted with the Layback.  The sensor 
                           //    position is then computed using the 
                           //    current sensor heading and this layback 
                           //    value.  The result is displayed when a 
                           //    position is computed in Isis. 

   float CableTension;     // [P] Cable tension from serial port. Stored only.

   //      
   // Sensor Attitude information
   //
   float SensorDepth;        // [0] Distance from sea surface to
                             //   sensor.  The deeper the sensor goes, 
                             //   the bigger (positive) this value becomes.

   float SensorPrimaryAltitude; 
                             // [7] Distance from towfish to the sea
                             //   floor.  This is the primary altitude as 
                             //   tracked by the Isis bottom tracker or
                             //   entered manually by the user. 
                             //   Although not recommended, the user can 
                             //   override the Isis bottom tracker by 
                             //   sending the primary altitude over the 
                             //   serial port.  The user should turn the 
                             //   Isis bottom tracker Off when this is done.
                                   
   float SensorAuxAltitude;  // [a] Auxillary altitude.  This is an
                             //   auxillary altitude as transmitted by an 
                             //   altimeter and received over a serial port.
                             //   The user can switch betwen the Primary and 
                             //   Aux altitudes via the "options" button in 
                             //   the Isis bottom track window.

   float SensorPitch;        // [8] Pitch in degrees (positive=nose up)
   float SensorRoll;         // [9] Roll in degrees (positive=roll to stbd)
   float SensorHeading;      // [h] Fish heading in degrees

   // These Pitch, Roll, Heading, Heave and Yaw values are those received 
   // closest in time to this sonar or bathymetry update.  If a TSS or MRU
   // is being used with a multibeam/bathymetry sensor, the user should 
   // use the higher-resolution attitude data found in the XTFATTITUDEDATA 
   // structures.


   //
   // additional attitude data
   //
   float Heave;            // Sensor heave at start of ping. 
                           // Positive value means sensor moved up.
   float Yaw;              // Sensor yaw.  Positive means turn to right.
   DWORD AttitudeTimeTag;  // milliseconds - used to coordinate with millisecond 
                           // time value in Attitude packet

   //
   // Misc.
   //
   float DOT;              // Distance Off Track

   DWORD NavFixMilliseconds; //	millisecond clock value when nav received

   //
   // The Isis computer clock time when this ping was received.
   // May be different from ping time at start of this record if
   // the sonar time-stamped the data and the two systems aren't synced.
   // This time should be ignored in most cases.
   //
   unsigned char ComputerClockHour;
   unsigned char ComputerClockMinute;
   unsigned char ComputerClockSecond;
   unsigned char ComputerClockHsec;

   //
   // Additional Tow Cable and Fish information from Trackpoint
   //
   short FishPositionDeltaX;       // [{DX}] Stored as meters*3.0, supporting +/- 10000.0m (usually from trackpoint)
   short FishPositionDeltaY;       // [{DY}] X,Y offsets can be used instead of logged layback.
   unsigned char FishPositionErrorCode; // Error code for FishPosition delta x,y (typically reported by Trackpoint)

   //
   // Pad to make an even 256 bytes
   //
   BYTE ReservedSpace2[11]; // Currently unused

}) XTFPINGHEADER, XTFBATHHEADER;



// Annotation record
// An annotation record is a line of text which can be saved to the
// file and is displayed in the "Notes" field on the Isis display.
// This text is displayed during playback.  Additionally, this text
// may be printed in realtime or in playback.  This can be activated
// in the Print Annotation dialog box.
///////////////////////////////////////////////////////////////////////////////
PACK(typedef struct {

   WORD MagicNumber;      // Set to 0xFACE
   BYTE HeaderType;       // XTF_HEADER_NOTES (1)
   BYTE SubChannelNumber;
   WORD NumChansToFollow;
   WORD Reserved[2];
   DWORD NumBytesThisRecord; // Total byte count for this update

   //
   // Date and time of the annotation
   //
   WORD  Year;
   BYTE  Month;
   BYTE  Day;
   BYTE  Hour;
   BYTE  Minute;
   BYTE  Second;
   BYTE  ReservedBytes[35];

   char  NotesText[256-56];

}) XTFNOTESHEADER;


// RAW ASCII data received over serial port
// These packets are stored in the XTF file on a per-serial-port
// basis.  To store the raw ASCII data for a given serial port, add 
// the token "{SAVEALL}" to the serial port template.  Use of this
// option is not generally recommended, since Isis already parses the
// data for all usefull information.

///////////////////////////////////////////////////////////////////////////////
PACK(typedef struct {

   WORD MagicNumber;      // Set to 0xFACE
   BYTE HeaderType;       // will be XTF_HEADER_RAW_SERIAL (7)
   BYTE SerialPort;  
   WORD Reserved2[3];
   DWORD NumBytesThisRecord; // Total byte count for this update

   //
   // Date and time raw ASCII data was posted to disk
   //
   WORD  Year;
   BYTE  Month;
   BYTE  Day;
   BYTE  Hour;
   BYTE  Minute;
   BYTE  Second;
   BYTE  HSeconds;      // hundredth of seconds (0-99)
   WORD  JulianDay;     // days since Jan 1.

   DWORD TimeTag;       // millisecond timer value
   WORD  StringSize;    // Number of valid chars in RawAsciiData string
   char  RawAsciiData[64-30]; // will be padded in 64-byte increments to make 
                              // structure an even multiple of 64 bytes

}) XTFRAWSERIALHEADER;


// Ping Channel header 
// This is data that can be unique to each channel from ping to ping.
// Is is stored at the front of each channel of sonar data.
///////////////////////////////////////////////////////////////////////////////
PACK(typedef struct {

   WORD  ChannelNumber;    // Typically, 
                           // 0=port (low frequency)
                           // 1=stbd (low frequency)
                           // 2=port (high frequency)
                           // 3=stbd (high frequency)

   WORD  DownsampleMethod;  // 2=MAX, 4=RMS
   float SlantRange;       // Slant range of the data in meters
   float GroundRange;      // Ground range of the data in meters
                           //   (SlantRange^2 - Altitude^2)
   float TimeDelay;        // Amount of time (in seconds) to the start of recorded data
                           //   almost always 0.0
   float TimeDuration;     // Amount of time (in seconds) recorded
   float SecondsPerPing;   // Amount of time (in seconds) from ping to ping

   WORD  ProcessingFlags;  // 4=TVG, 8=BAC&GAC, 16=Filter, etc...
                           //   almost always 0
   WORD  Frequency;        // Center transmit frequency for this channel.
						   //   when non-zero, replaces value found in file
                           //   header CHANINFO struct ChanInfo->SamplesPerChannel.
                           //   This allows samples per channel to change on the fly.

   WORD InitialGainCode;   // Settings as transmitted by sonar
   WORD GainCode;
   WORD BandWidth;

   //
   // Contact information - updated when contacts are saved through Target.exe
   //
   DWORD ContactNumber;
   WORD  ContactClassification;
   unsigned char ContactSubNumber;
   unsigned char ContactType;


   DWORD NumSamples;            // Number of samples that will follow this structure.  The
                                // number of bytes will be this value multipied by the
								// number of bytes per sample (given in the file header)

   WORD  Reserved;              // Obsolete.
   float ContactTimeOffTrack;	// Time off track to this contact (stored in milliseconds)
   unsigned char ContactCloseNumber;
   unsigned char Reserved2;

   float FixedVSOP;             // Fixed along-track size of each ping, stored in cm.
                                //  on multibeam system with zero beam spread, this value
                                //  needs to be filled in to prevent Isis from calculating
                                //  along-track ground coverage based on beam spread and 
                                //  speed over ground.
   
   short Weight; // Weighting factor given by some sonars

   BYTE  ReservedSpace[4];      // reserved for future expansion
   //BYTE  ReservedSpace[6];      // reserved for future expansion

}) XTFPINGCHANHEADER;
//} XTFPINGCHANHEADER;

#undef PACK

#endif // XTF_H
