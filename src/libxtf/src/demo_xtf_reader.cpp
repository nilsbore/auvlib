
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#ifdef _MSC_VER
  #include <io.h>
#else
  #include <unistd.h>
#endif
#include <stdlib.h>
#include <string.h>
extern "C" {
#include <libxtf/xtf_reader.h>
}

using namespace std;

int main(int argc, char** argv)
{
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
         return 0;
   }

   if (argc < 2) {
       cout << "Usage: DEMO_XTF <input .XTF file>" << endl;
       cout << "Reads an XTF file and prints some data about it." << endl;
       return 0;
   }

   int infl = open(argv[1], O_RDONLY, 0000200);
   if (infl <= 0) {
       cout << "Error: Can't open " << argv[1] << " for reading!" << endl;
       return 0;
   }

   //
   // Allocate memory for reading file data
   //
   unsigned char* buffer = (unsigned char*)malloc((WORD)32768);
   if (buffer == NULL) {
       cout << "Can't allocate memory!" << endl;
       return 0;
   }

   //
   // Allocate memory for storing XTF header
   //
   XTFFILEHEADER* XTFFileHeader = (XTFFILEHEADER*)malloc((WORD)sizeof(XTFFILEHEADER));
   if (XTFFileHeader == NULL) {
       cout << "\nCan't allocate memory for XTF header" << endl;
       return 0;
   }

   ReadXTFFile(infl, XTFFileHeader, buffer);

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
