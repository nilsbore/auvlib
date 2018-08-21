#ifndef XTF_READER_H
#define XTF_READER_H

// File: DEMO_XTF.C   September 6, 1996 (modified June 1, 1998)
// Sample program for reading XTF files.  This program may be freely
// distributed.  Triton Technology assumes no liability of any kind
// for any errors related to this program.
// 
// This program has been tested with Microsoft C, versions 7.0 and 8.0
// in small and large memory models.  This is a DOS program but can be
// easily adapted for any operating system.
//
// Modified by Nils Bore, nbore@kth.se to compile on Ubuntu, with Clang 5.0
//
// To compile: CL /c /W3 /AL DEMO_XTF.C
//

#include <libxtf/xtf.h> // contains structures and definitions for XTF files

// Function prototypes for functions found later in this file.
void ReadXTFFile(int infl, XTFFILEHEADER *XTFFileHeader, unsigned char* buffer);
BOOL ReadXTFHeader(int infl, XTFFILEHEADER *XTFFileHeader, unsigned char* buffer);
unsigned int ReadXTFFormatFileData(int infl, unsigned char *buffer);
long FindIsisFmtHeader(unsigned char *buf, long cnt, unsigned char RecordType, int Dir);
BOOL AlignIsisFmtFile(int infl, unsigned char RecordType, unsigned char *TempBuffer);
long GetPingNumberFromIsisFmtFile(int fl, unsigned char RecordType, unsigned char *TempBuffer);
BOOL GoToIsisFmtPing(int infl, long DestPingNumber, unsigned char RecordType, unsigned char *TempBuffer);
long XTFFmtLastPingNumberInFile(int fl, unsigned char RecordType, unsigned char *TempBuffer);

void ProcessXTFHeader(int infl, XTFFILEHEADER *XTFFileHeader, unsigned char* buffer);
void ProcessAttitudeUpdate(XTFATTITUDEDATA *PingHeader);
void ProcessNotes(XTFNOTESHEADER *PingHeader);
void ProcessMultibeamPing(XTFBATHHEADER *PingHeader);
void ProcessSidescanPing(XTFPINGHEADER *PingHeader, XTFFILEHEADER *XTFFileHeader);
void ProcessRawSerial(XTFRAWSERIALHEADER *SerialHeader);

#endif // XTF_READER_H
