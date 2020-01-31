// File: DEMO_XTF.C   September 6, 1996 (modified June 1, 1998)
// Sample program for reading XTF files.  This program may be freely
// distributed.  Triton Technology assumes no liability of any kind
// for any errors related to this program.
// 
// This program has been tested with Microsoft C, versions 7.0 and 8.0
// in small and large memory models.  This is a DOS program but can be
// easily adapted for any operating system.
//
// To compile: CL /c /W3 /AL DEMO_XTF.C
//
//#pragma pack(1)

#include <stdio.h>
#include <fcntl.h>
//#include <conio.h>
//#include <io.h>
#include <stdlib.h>
#include <string.h>
#include <libxtf/xtf.h> // contains structures and definitions for XTF files

// Function prototypes for functions found later in this file.
void ReadXTFFile(int infl);
BOOL ReadXTFHeader(int infl, XTFFILEHEADER *XTFFileHeader);
unsigned int ReadXTFFormatFileData(int infl, unsigned char *buffer);
long FindIsisFmtHeader(unsigned char *buf, long cnt, unsigned char RecordType, int Dir);
BOOL AlignIsisFmtFile(int infl, unsigned char RecordType, unsigned char *TempBuffer);
long GetPingNumberFromIsisFmtFile(int fl, unsigned char RecordType, unsigned char *TempBuffer);
BOOL GoToIsisFmtPing(int infl, long DestPingNumber, unsigned char RecordType, unsigned char *TempBuffer);
long XTFFmtLastPingNumberInFile(int fl, unsigned char RecordType, unsigned char *TempBuffer);

void ProcessXTFHeader(XTFFILEHEADER *XTFFileHeader);
void ProcessAttitudeUpdate(XTFATTITUDEDATA *PingHeader);
void ProcessNotes(XTFNOTESHEADER *PingHeader);
void ProcessMultibeamPing(XTFBATHHEADER *PingHeader);
void ProcessSidescanPing(XTFPINGHEADER *PingHeader);
void ProcessRawSerial(XTFRAWSERIALHEADER *SerialHeader);

int NumSonar=0, NumHidden=0, NumBathy=0, NumAnnotation=0, NumAttitude=0, NumSerial=0;

// Possible types of data when multiple channels are involved.
#define NUM_DATA_TYPES 7
char *ChannelTypes[NUM_DATA_TYPES] = {
   "SUBBOTTOM",
   "PORT",
   "STBD",
   "BATHY",
   "ELAC",
   "SERIAL",
   "UNRECOGNIZED"
};

// For now, assume that the input file will contain 6 or fewer
// channels of data.  This means that the file header will always
// be 1024 bytes.  In the event that a file has more than 6 channels,
// this program will need to be changed.
#define FILEHEADERSIZE (sizeof(XTFFILEHEADER))

int infl=0;
unsigned int amt;
unsigned char *buffer = NULL;
XTFFILEHEADER *XTFFileHeader = NULL;
int NumSonarChans, NumBathyChans;
int StartPing = 0;
char *ptr;

int main(int argc, char *argv[]) {
/****************************************************************************/

   // quick sanity check to make sure the compiler didn't
   // screw up the data structure sizes by changing alignment.
   printf("Size of word: %d", sizeof(WORD));
   printf("Size of long: %d", sizeof(long));
   if (
      sizeof(XTFFILEHEADER)      != 1024 || 
      sizeof(CHANINFO)           != 128  ||
      sizeof(XTFPINGHEADER)      != 256  ||
      sizeof(XTFNOTESHEADER)     != 256  ||
      sizeof(XTFBATHHEADER)      != 256  ||
      sizeof(XTFRAWSERIALHEADER) != 64   ||
      sizeof(XTFPINGCHANHEADER)  != 64
      ) {
         printf("Error: Bad structure size! %d,%d,%d,%d,%d,%d,%d\n",
            sizeof(XTFFILEHEADER)     ,
            sizeof(CHANINFO)          ,
            sizeof(XTFPINGHEADER)     ,
            sizeof(XTFNOTESHEADER)    ,
            sizeof(XTFBATHHEADER)     ,
            sizeof(XTFRAWSERIALHEADER),
            sizeof(XTFPINGCHANHEADER));
         getchar();
         return 0;
   }

   if (argc < 2) {
      printf("Usage: DEMO_XTF <input .XTF file>\n");
      printf("       Reads an XTF file and prints some data about it.\n");
      return 0;
   }

   //strupr(argv[1]);

#ifdef _MSC_VER
   infl = open(argv[1], O_RDONLY | O_BINARY, 0000200);
#else
   infl = open(argv[1], O_RDONLY, 0000200);
#endif
   if (infl <= 0) {
      printf("Error: Can't open %s for reading!\n", argv[1]);
      goto ERROR_OUT;
   }


   //
   // Allocate memory for reading file data
   //
   buffer = (unsigned char *) malloc((WORD)32768);
   if (buffer == NULL) {
      printf("Can't allocate memory!\n");
      goto ERROR_OUT;
   }

   //
   // Allocate memory for storing XTF header
   //
   XTFFileHeader = (XTFFILEHEADER *) malloc((WORD)sizeof(XTFFILEHEADER));
   if (XTFFileHeader == NULL) {
      printf("\nCan't allocate memory for XTF header\n");
      goto ERROR_OUT;
   }


   ReadXTFFile(infl);

   ERROR_OUT:

   if (infl > 0) {
      close(infl);
      infl = 0;
   }
   if (buffer != NULL) {
      free(buffer);
      buffer = NULL;
   }
   if (XTFFileHeader != NULL) {
      free(XTFFileHeader);
      XTFFileHeader = NULL;
   }

   return 0;
}

void ReadXTFFile(int infl) {
/***************************************************************************************/
// Given a handle to on open .XTF file, read through the file and
// print out some data about it.

   //
   // Read the XTF file header
   //

   if (ReadXTFHeader(infl, XTFFileHeader) == FALSE) return;


   ProcessXTFHeader(XTFFileHeader);


   //
   // Read the XTF file one packet at a time
   //

   while ((amt = ReadXTFFormatFileData(infl, buffer)) != 0xFFFF) {
      //
      // Buffer now holds a single record which can be processed 
      // here.  The return value from ReadXTFFormatFileData()
      // holds byte length of the data record (i.e., sidescan ping)
      //

      XTFPINGHEADER *PingHeader = (XTFPINGHEADER *) buffer;

      switch (PingHeader->HeaderType) {
         case XTF_HEADER_SONAR    : ptr = "SONAR "; // sidescan ping
            NumSonar++;
            ProcessSidescanPing((XTFPINGHEADER *) PingHeader);
            break;

         case XTF_HEADER_HIDDEN_SONAR  : ptr = "HIDDEN"; // sidescan ping
            NumHidden++;
            ProcessSidescanPing((XTFPINGHEADER *) PingHeader);
            break;

         case XTF_HEADER_BATHY    : ptr = "BATHY "; // multibeam ping
            NumBathy++;
            ProcessMultibeamPing((XTFBATHHEADER *) PingHeader);
            break;

         case XTF_HEADER_ATTITUDE : ptr = "TSS   "; // TSS or MRU update
            NumAttitude++;
            ProcessAttitudeUpdate((XTFATTITUDEDATA *) PingHeader);
            continue;
            break;

         case XTF_HEADER_NOTES    : ptr = "NOTES "; // just some user-entered annotation
            NumAnnotation++;
            ProcessNotes((XTFNOTESHEADER *) PingHeader);
            continue;
            break;

         case XTF_HEADER_ELAC     : ptr = "ELAC  "; // just some user-entered annotation
            NumBathy++;
            break;

         case XTF_HEADER_RAW_SERIAL : ptr = "SERIAL";
            NumSerial++;
            ProcessRawSerial((XTFRAWSERIALHEADER *) PingHeader);
            continue;
            break;

         default : ptr = "OTHER"; 
            printf("%s  \r", ptr);
            continue;
            break;
      }


      printf("%s %02u/%02u/%02u %02u:%02u:%02u Y=%.6lf X=%.6lf pitch=%.1f roll=%.1f\r",
         ptr,
         PingHeader->Month,
         PingHeader->Day,
         PingHeader->Year,
         PingHeader->Hour,
         PingHeader->Minute,
         PingHeader->Second,
         PingHeader->SensorYcoordinate,
         PingHeader->SensorXcoordinate,
         PingHeader->SensorPitch,
         PingHeader->SensorRoll);

      //if (kbhit()) if (getchar() == 27) {
      if (getchar() == 27) {
         printf("\nUser pressed ESC\n");
         break; // press ESC to quit.
      }
   }

   if (amt == 0xFFFF) {
      printf("\nStopped - read -1 bytes\n");
   }
   else {
      printf("\nDon't know why loop stopped.\n");
   }

   printf("\n\nPacket count: %u sonar, %u hidden, %u bathy, %u annotation, %u attitude, %d raw serial\n",
      NumSonar, NumHidden, NumBathy, NumAnnotation, NumAttitude, NumSerial); 

   printf("\nDone!\n");
}

BOOL ReadXTFHeader(int infl, XTFFILEHEADER *XTFFileHeader) {
/****************************************************************************/
// Read the header out of the XTF file and get ready to read the data that
// follows in the file.  Returns TRUE if all wend OK.


   //
   // Read file header
   //
   if (read(infl, XTFFileHeader, FILEHEADERSIZE) != FILEHEADERSIZE) {
      printf("Error reading file header!\n");
      return FALSE;
   }

   if (XTFFileHeader->FileFormat != FMT_XTF) {
      printf("Bad header ID (%d) -- this file is not an XTF format file!\n", XTFFileHeader->FileFormat);
      return FALSE;
   }

   printf("This file contains %ld sonar pings and %ld bathymetry pings\n\n",
      XTFFmtLastPingNumberInFile(infl, XTF_HEADER_SONAR, buffer),
      XTFFmtLastPingNumberInFile(infl, XTF_HEADER_BATHY, buffer) + 
      XTFFmtLastPingNumberInFile(infl, XTF_HEADER_ELAC, buffer));

   // align back to start of data.  0xFF matches any kind of packet.
   GoToIsisFmtPing(infl, StartPing, 0xFF, buffer);
                                                                     
   return TRUE;

}

void ProcessXTFHeader(XTFFILEHEADER *XTFFileHeader) {
/*****************************************************************************/

   int chan;

   printf("\nXTF File header information:\n");

   NumSonarChans = XTFFileHeader->NumberOfSonarChannels;
   NumBathyChans = XTFFileHeader->NumberOfBathymetryChannels;

   // How many 1024-byte blocks do we need to read in order
   // get all of the channel info structures?  For now,
   // don't bother looking into the structures.  Just get
   // past them so we can start reading the data in the file.
   {
      unsigned int Cnt = (NumSonarChans+NumBathyChans);
      if (Cnt > 6) {
         Cnt -= 6;
         Cnt *= sizeof(CHANINFO);
         Cnt += 1023; // bring to next even 1024-byte increment.
         Cnt /= 1024; // use integer math to truncate fraction.
         read(infl, buffer+1024, Cnt*1024);
      }
   }

   printf("Recording program version: %s\n",XTFFileHeader->RecordingProgramVersion);

   printf("Number of Sonar channels: %d\n", NumSonarChans);

   for (chan=0; chan<NumSonarChans; chan++) {
      int chtype = XTFFileHeader->ChanInfo[chan].TypeOfChannel;
      if (chtype >= NUM_DATA_TYPES) chtype = NUM_DATA_TYPES-1;

      printf("   Sonar channel %d is %s, %d byte(s) per sample\n", 
         chan, 
         ChannelTypes[chtype],
         XTFFileHeader->ChanInfo[chan].BytesPerSample);
   }                                                   

   printf("Number of Bathymetry channels: %d\n", NumBathyChans);
   for (chan=0; chan<NumBathyChans; chan++) {
      printf("   Bathy channel %d is %s, mounted %.1f degrees\n", 
         chan, 
         ChannelTypes[XTFFileHeader->ChanInfo[chan+NumSonarChans].TypeOfChannel],
         XTFFileHeader->ChanInfo[chan+NumSonarChans].OffsetRoll);
   }

   printf("\n");
}

unsigned int ReadXTFFormatFileData(int infl, unsigned char *buffer) {
/*****************************************************************************/
// This function will read data from an XTF file and return buffer with
// a ping of sonar data in it.  This function is limited to pings with a byte
// count of less than 64K.
// Returns the number of bytes in *buffer*.  These bytes will be a 
// Sonar data packet.  The calling program can use this data (which will
// be side scan and/or subbottom data) for anything.

   XTFPINGHEADER *PacketHeader;
   unsigned char *SrcPtr = buffer;
   unsigned long BytesScanned=0L;
   long AmountRemaining;
   long AmountNeeded;
   long RecordLength;
   long AmountRead, tmp, len;

      // Read in 64 bytes just to get going
   if ((AmountRead = read(infl, SrcPtr, 64)) != 64) {
      printf("\nCan't read 64 bytes\n");
      return 0xFFFF;
   }

   AmountRemaining = AmountRead;

   while (((long)BytesScanned < AmountRead) && AmountRemaining) {

      PacketHeader = (XTFPINGHEADER *) SrcPtr;

      RecordLength = PacketHeader->NumBytesThisRecord;

      switch (PacketHeader->HeaderType) {

         case XTF_HEADER_SONAR        :
         case XTF_HEADER_BATHY        :
         case XTF_HEADER_ATTITUDE     :
         case XTF_HEADER_NOTES        :
         case XTF_HEADER_ELAC         :
         case XTF_HEADER_RAW_SERIAL   :
         case XTF_HEADER_EMBED_HEAD   :
         case XTF_HEADER_HIDDEN_SONAR :
         case XTF_HEADER_ECHOSTRENGTH :

               // Make sure enough data sits in memory to have
               // a complete packet
            AmountRemaining = AmountRead - BytesScanned;
            if (AmountRemaining && AmountRemaining < RecordLength) {
               AmountNeeded = RecordLength - AmountRemaining;
               AmountRead  += (len=read(infl, buffer+(WORD)AmountRead, (WORD)AmountNeeded));
               if (len != AmountNeeded) return 0;
               AmountRemaining = AmountRead - BytesScanned;
            }

            SrcPtr       += RecordLength;
            BytesScanned += RecordLength;
            AmountRemaining = AmountRead - BytesScanned;

            break;
      

         default : // Unrecognized header type.  Skip past it.

            printf("\nUnknown data packet: 0x%X\n", (int) (PacketHeader->HeaderType));

            AmountRemaining = AmountRead - BytesScanned;

               // Make sure enough data sits in memory to have
               // a complete note record block
            if (AmountRemaining < RecordLength) {
               AmountNeeded    = RecordLength - AmountRemaining;
               tmp = AmountRead;
               AmountRead     += read(infl, buffer+(WORD)AmountRead, (WORD)AmountNeeded);
               AmountRemaining = AmountRead - BytesScanned;
            }

            if (AmountRemaining >= RecordLength) {
               memcpy(SrcPtr, SrcPtr+(WORD)RecordLength, (WORD)(AmountRemaining - RecordLength));
               AmountRead -= RecordLength;
               tmp = AmountRead;
               AmountRead += (len = read(infl, buffer+(WORD)AmountRead, (WORD)RecordLength));
               if (len != RecordLength) return 0;
               AmountRemaining = AmountRead - BytesScanned;
            }
            break;
      }
   }

   return (WORD)AmountRead;
}

long FindIsisFmtHeader(unsigned char *buf, long cnt, unsigned char RecordType, int Dir) {
/************************************************************************************************/
// Somewhere in buf is (probably) an ping header of type RecordType.
// Find it, and return the offset to it.  If Dir!=0, find backwards (from end)
// Returns offset to the start of the record, or -1 if not found.

   long i;
   
   long start, end, inc;

   if (Dir == 0) {
      start = 0;
      end = cnt-20+1;
      inc = 1;
   }
   else {
      start = cnt-20-1;
      end = -1;
      inc = -1;
   }

      //
      // Sanity check to see if this is actually
      // the start of a data packet.
      //
   for (i=start; i != end; i+=inc) {
      if (
         *((WORD *)&buf[i]) == 0xFACE && // magic number
         (RecordType == 0xFF || buf[i+2] == RecordType) &&

         buf[i+6] == 0    &&
         buf[i+7] == 0    &&

         buf[i+16] < 13   && // month
         buf[i+17] < 33   && // day
         buf[i+18] < 24   && // hour
         buf[i+19] < 60   && // minute
         buf[i+20] < 60) {   // sec

         return i;
      }
   }
   return -1L;
}


BOOL AlignIsisFmtFile(int infl, unsigned char RecordType, unsigned char *TempBuffer) {
/************************************************************************************************/
// Align an Isis-format file to the beginning of a ping with type RecordType
// Returns TRUE if successful, FALSE if failure.

   long Skip;
   int tries;

   for (tries=0; tries < 10; tries++) {
      int amt = read(infl, TempBuffer, 16384);
      if (amt <= 0) break;

      Skip = FindIsisFmtHeader(TempBuffer, amt, RecordType, 0);
      if (Skip != -1) {
         lseek(infl, Skip-amt, 1);
         return TRUE;
      }
   }
   // printf("\nNever found alignment!\n");
   return FALSE;
}
   
long GetPingNumberFromIsisFmtFile(int fl, unsigned char RecordType, unsigned char *TempBuffer) {
/************************************************************************************************/
// Given the current file alignment, find the next ping of type RecordType
// and return the ping number of that ping.
// Returns Ping Number if successful, -1 if failure.

   XTFPINGHEADER PingHeader;

   if (AlignIsisFmtFile(fl, RecordType, TempBuffer)) {
      if (read(fl, &PingHeader, sizeof(XTFPINGHEADER)) != sizeof(XTFPINGHEADER)) {
         return -1L;
      }
      return PingHeader.PingNumber;
   }
   return -1L;
}

BOOL GoToIsisFmtPing(int infl, long DestPingNumber, unsigned char RecordType, unsigned char *TempBuffer) {
/************************************************************************************************/
// Given a ping number, this function will position a file to the indicated point in 
// a Isis format file (.XTF).  Alignment will be done on sonar pings if 
// RecordType==XTF_HEADER_SONAR or Bathy pings if RecordType==XTF_HEADER_BATHY.
// If RecordType == 0xFF, aligns to any packet type.
// Returns TRUE if successful, FALSE otherwise.
//
/* This function used a modified binary search to skip around the file
   and look for packet headers.  This modified binary search looks like
   this pseudo-code:

   ptr = strt of file
   offset = size of file / 2

   do 
      check = ping number at (ptr+offset)
      if (check > SonarPingNumber) offset /= 2;
      else ptr += offset;
   until found

*/
   unsigned long BytesInFile, Offset;
   unsigned long ptr = 0L;
   long check;

   BytesInFile = lseek(infl, 0L, 2);
   if (BytesInFile <= FILEHEADERSIZE) return FALSE;

   BytesInFile -= FILEHEADERSIZE;

   Offset = (BytesInFile/2);

   do {
      lseek(infl, ptr + Offset + FILEHEADERSIZE, 0);
      check = GetPingNumberFromIsisFmtFile(infl, RecordType, TempBuffer);
      if (check == DestPingNumber) break;
      if (check > DestPingNumber || check == -1L) {
         Offset /= 2L;
         if (Offset < 256L) break;
      }
      else {
         ptr += Offset;
      }
   } while (1);

   lseek(infl, ptr+Offset+FILEHEADERSIZE, 0);
   AlignIsisFmtFile(infl, RecordType, TempBuffer);
   return TRUE;
}

long XTFFmtLastPingNumberInFile(int fl, unsigned char RecordType, unsigned char *TempBuffer) {
/************************************************************************************************/
// Align an Isis-format file to the beginning of a ping with type RecordType
// Returns Ping Number if successful, -1 otherwise.

   long Skip;
   int amt;
   int tries;
   XTFPINGHEADER *PingHeaderPtr;

      // Try to read from the file backwards 10 times to
      // see if we encounter a ping with this record type.

   for (tries=0; tries < 10; tries++) {

      lseek(fl, -(16384L * (tries+1)), 2);

      amt = read(fl, TempBuffer, 16384);
      if (amt <= 0) break;

         // Find backwards
      Skip = FindIsisFmtHeader(TempBuffer, amt, RecordType, 1);
      if (Skip != -1) {
         lseek(fl, Skip-amt, 1);
         read(fl, TempBuffer, 256);
         PingHeaderPtr = (XTFPINGHEADER *) TempBuffer;
         return PingHeaderPtr->PingNumber;
      }
   }

   return 0;
}
   
void ProcessSidescanPing(XTFPINGHEADER *PingHeader) {
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

   for (chan=0; chan<PingHeader->NumChansToFollow; chan++) {

      XTFPINGCHANHEADER *ChanHeader;
      unsigned char *Imagery;
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
      Imagery = Ptr;

      // Do whatever processing on the sidescan imagery here.

      // skip past the imagery;
      Ptr += BytesThisChannel;
   }

}

void ProcessMultibeamPing(XTFBATHHEADER *PingHeader) {
/****************************************************************************/
// Put multibeam processing here.
// PingHeader points to a 256-byte bathyheader structure.  That structure
// is followed by the bathy data itself.  When Isis saves bathy data,
// only one packet is saved at a time.  If there are two SEABAT heads,
// the data from each one will be stored with in different records and
// with their own XTFBATHHEADER structures.

   unsigned char *Ptr = (unsigned char *)PingHeader;
   unsigned char *BathyPacket;
   static DWORD LastTimeTag = 0L, BathyTimeTag;
   static DWORD AvgN=0, AvgD=0;

   // skip past the ping header
   Ptr += sizeof(XTFBATHHEADER);

   BathyPacket = Ptr;

   // BathyPacket now points to the raw data sent up by the
   // SEABAT (or Odom ECHOSCAN).  To determine what kind of
   // raw data, we must look at the data directly.
   //
   // See the Reson or Odom documentation for details on their
   // binary data format.

   // BathyPacket points to single binary data packet.  The length
   // of that packet can be determined by the Seabat or Echoscan.
   // Currently, there are only three packets recognized:
   // 1. Seabat R-Theta (140 bytes)
   // 2. Seabat RI-Theta (210 bytes)
   // 3. Echoscan R-Theta (80 bytes)

   // Do whatever Bathymetry processing here.  BathyPacket
   // points to the binary multibeam data.  The relative time
   // for this ping (in milliseconds) is given by
   //
   BathyTimeTag = PingHeader->AttitudeTimeTag;

   if (LastTimeTag) {
      if (BathyTimeTag < LastTimeTag) {
         // printf("\n*****BATHY TIME WENT BACKWARDS by %ld milliseconds!\n",LastTimeTag-BathyTimeTag);
      }
      else {
         AvgN += (BathyTimeTag - LastTimeTag);
         AvgD ++;

         // printf("Bathy time diff: %ld (avg=%ld)\n", BathyTimeTag - LastTimeTag, AvgN/ AvgD);
      }
   }
   LastTimeTag = BathyTimeTag;
   //
   // Use this time tag to correlate the Attitude (TSS or MRU) data
   // with this multibeam data.

}

void ProcessAttitudeUpdate(XTFATTITUDEDATA *AttitudeData) {
/****************************************************************************/
// AttitudeData points to a single 64-byte structure that holds a 
// TSS or MRU update.

   DWORD AttitudeTimeTag = AttitudeData->TimeTag;
   float Pitch           = AttitudeData->Pitch;
   float Roll            = AttitudeData->Roll;
   float Heave           = AttitudeData->Heave;
   float Yaw             = AttitudeData->Yaw;
   static DWORD LastTimeTag;
   static DWORD AvgN=0, AvgD=0;


   //
   // Use AttitudeTimeTag to correlate this attitude update with
   // the multibeam data.  Consider putting the data received by
   // this function in a circular buffer.  Then, use BathyTimeTag
   // as a value to perform interpolation on the attitude data to
   // closely approximate the attitude at BathyTimeTag.
   //

   if (LastTimeTag) {
      if (AttitudeTimeTag < LastTimeTag) {
         // printf("\n******* ATTITUDE TIME JITTERED by %ld milliseconds\n", LastTimeTag - AttitudeTimeTag);
      }
      else {
         AvgN += (AttitudeTimeTag - LastTimeTag);
         AvgD++;
         // printf("Attitude Time dif: %ld (avg=%ld)\n", AttitudeTimeTag - LastTimeTag, AvgN/AvgD);
      }

   }
   LastTimeTag = AttitudeTimeTag;
}

void ProcessNotes(XTFNOTESHEADER *NoteHeader) {
/****************************************************************************/
// Stored in the XTF file whenever the user type in notes during data 
// collection.
//
   printf("\nNote: %s\n", NoteHeader->NotesText);
}

void ProcessRawSerial(XTFRAWSERIALHEADER *SerialHeader) {
/****************************************************************************/
// Raw ASCII serial data as received over a serial port.  Present in file
// only when "SAVERAW" present on Isis' serial template for a specific port.

   printf("\nPort %d: %s\n",
      SerialHeader->SerialPort,
      SerialHeader->RawAsciiData);
}

//#pragma pack()

