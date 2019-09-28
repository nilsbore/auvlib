/*  Original code from https://github.com/austin-robot/utexas-art-ros-pkg/blob/master/stacks/art_vehicle/art_common/include/art/UTM.h
 *
 *  Conversions between Latitude/Longitude and UTM
 *              (Universal Transverse Mercator) coordinates.
 *
 *  Copyright Chuck Gantz (chuck.gantz@globalstar.com)
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#include <data_tools/lat_long_utm.h>

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI // For windows
  #define M_PI 3.14159265358979323846
#endif

using namespace std;

namespace lat_long_utm {

/**
 * Determine the correct UTM letter designator for the
 * given latitude
 *
 * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
#define WGS84_A		6378137.0		///< major axis
#define WGS84_E		0.0818191908		///< first eccentricity
#define UTM_E2		(WGS84_E*WGS84_E)	///< e^2
#define UTM_K0		0.9996			///< scale factor

static inline char UTMLetterDesignator(double Lat)
{
    char LetterDesignator;

    if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
    else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
    else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
    else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
    else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
    else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
    else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
    else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
    else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
    else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
    else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
    else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
    else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
    else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
    else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
    else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
    else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
    else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
    else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
    else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
    // 'Z' is an error flag, the Latitude is outside the UTM limits
    else LetterDesignator = 'Z';
    return LetterDesignator;
}

/**
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 * Modified by Nils Bore- nbore@kth.se
 */
tuple<double, double, string> lat_long_to_UTM(const double Lat, const double Long)
{
    double UTMNorthing;
    double UTMEasting;
    string UTMZone;

    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double k0 = UTM_K0;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    //Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = (Long+180)-int((Long+180)/360)*360-180;

    double LatRad = Lat*M_PI/180.;
    double LongRad = LongTemp*M_PI/180.;
    double LongOriginRad;
    int    ZoneNumber;

    ZoneNumber = int((LongTemp + 180)/6) + 1;

    if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
        ZoneNumber = 32;

    // Special zones for Svalbard
    if( Lat >= 72.0 && Lat < 84.0 )
    {
        if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
        else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
        else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
        else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
    }
    // +3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
    LongOriginRad = LongOrigin * M_PI/180.;

    //compute the UTM Zone from the latitude and longitude
    char buff[10];
    snprintf(buff, sizeof(buff), "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
    UTMZone = buff;

    eccPrimeSquared = (eccSquared)/(1-eccSquared);

    N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
    T = tan(LatRad)*tan(LatRad);
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
    A = cos(LatRad)*(LongRad-LongOriginRad);

    M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
            - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
           - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
              + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
           + (15*eccSquared*eccSquared/256
              + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
           - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

    UTMEasting = (double)
    (k0*N*(A+(1-T+C)*A*A*A/6
           + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
     + 500000.0);

    UTMNorthing = (double)
    (k0*(M+N*tan(LatRad)
         *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
           + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

    if(Lat < 0)
    {
        //10000000 meter offset for southern hemisphere
        UTMNorthing += 10000000.0;
    }

    return make_tuple(UTMNorthing, UTMEasting, UTMZone);
}

} // namespace lat_long_utm
