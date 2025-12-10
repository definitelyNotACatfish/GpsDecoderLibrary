//-----------------------------------------------------------------------------
//
/// @file
/// @brief This file contains the implementation of gpsDecoder, based on TinyGPS++
///
/// <please insert here the optional more detail description>
///
//-----------------------------------------------------------------------------
// $Author: FKSN $
// $Change: $
// $DateTime: 09.03.2023 $
//
// Creator: FKSN ()
// Compiler: Arduino PlattformIO
//-----------------------------------------------------------------------------

// ******************************************************************
// Includes
// ******************************************************************
#include "GpsDecoder.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>


// ******************************************************************
// Constructor
// ******************************************************************
GpsDecoderClass::GpsDecoderClass()
{
  // Init current frame with 0s
  memset(&currentFrame[0],0,sizeof(currentFrame));

  decodedCharCount = 0;        // Endless increasing number of encoded chars
  sentencesWithFixCount = 0;   // Endless increasing number of fixed sentences
  failedChecksumCount = 0;     // Endless increasing number of failed checksum messages
  passedChecksumCount = 0;     // Endless increasing number of passed checksum messages

  waitForFrameStart = false;   // Reset wait for frame start
}

// ******************************************************************
// Methods
// ******************************************************************

// Decode on passed char until a frame is received
bool GpsDecoderClass::decode(char currentChar)
{
  // Count up encoded char count
  decodedCharCount++;

  // Try to get find a valid sentence in data stream
  switch(currentChar)
  {
    // sentence begin
    case '$':
      // Reset decode state variables
      calculatedChecksum = 0;
      currentFrameOffset = 0;
      memset(currentFrame,0,sizeof(currentFrame));

      // A new frame starts
      waitForFrameStart = true;
      blockReadChecksumInCalculation = false;
    break;


    // * not in the checksum calculation
    case '*':
      // if no frame start has been detected earlier break immediately
      if (!waitForFrameStart) break;

      // Block all other following bytes, there fore they are the checksum of the frame
      blockReadChecksumInCalculation = true;

    // ordinary characters, fill buffer
    default:
      // if no frame start has been detected earlier break immediately
      if (!waitForFrameStart) break;

      // If a * has been received prior, this is probably the checksum of the frame, so don't use it here
      if(!blockReadChecksumInCalculation)
      {
        // Xor current char into calculated checksum
        calculatedChecksum ^= (uint8_t)currentChar;
      }

      // Try to save the current message type into variable, prevent overflow
      if (currentFrameOffset < (sizeof(currentFrame) - 1))
      {
        // Save message type in variable for later
        currentFrame[currentFrameOffset] = currentChar;
        currentFrameOffset++;
      }
    break;

    // Dont care about \r completely
    case '\r':
    break;

    // last line
    case '\n':
      // if no frame start has been detected earlier break immediately
      if (!waitForFrameStart) break;

      // Frame now went through
      waitForFrameStart = false;

      // Close the frame with a 0 termination
      currentFrame[currentFrameOffset] = 0;

      // Check the complete sentence and return if its valid or not
      // Example frame to parse "GPRMC,162614,A,5230.5900,N,01322.3900,E,10.0,90.0,131006,1.2,E,A*13"
      return parseFrame();
    break;
  }

  // Allways return false
  return false;
}


// Process one received frame
// Returns true if new sentence has just passed checksum test and is validated
bool GpsDecoderClass::parseFrame()
{
  // Calc checksum from frame, check if * is existend or not
  char *checksumPos = strstr(currentFrame,"*");
  if(!checksumPos)
  {
    GPS_DECODER_LOG("GPS decoder: No * in frame: \"%s\" found\n", currentFrame);
    return false;
  }
  uint8_t checksum = 16 * fromHex(*(++checksumPos)) + fromHex(*(++checksumPos));

  // Check if checksum is valid
  if (checksum != calculatedChecksum)
  {
    GPS_DECODER_LOG("GPS decoder: Invalid checksum! calc 0x%02x != read 0x%02x\n", calculatedChecksum,checksum);

    // Update failed checksum counter
    failedChecksumCount++;
    return false;
  }

  GPS_DECODER_LOG("GPS decoder: Valid checksum\n");
  // Update valid checksum
  passedChecksumCount++;

  // Cut off checksum now, makes parsing downwards easier
  char *checksumCutOff = strstr(currentFrame,"*");
  *checksumCutOff = 0;

  // Check what type of frame has to be parsed
  // GN = combination of all used satellite systems
  
  // Global Positioning System Fix Data. Time, Position and fix related data for a GPS receiver
  if(!strncmp("GNGGA",currentFrame,5))
  {
    GPS_DECODER_LOG("GPS decoder: Found GGA frame: \"%s\"\n", currentFrame);
    return parseFrameGGA(currentFrame);
  }
  // Recommended Minimum Navigation Information
  else if(!strncmp("GNRMC",currentFrame,5))
  {
    GPS_DECODER_LOG("GPS decoder: Found RMC frame: \"%s\"\n", currentFrame);
    return parseFrameRMC(currentFrame);
  }
  // DOP and active satellites
  else if(!strncmp("GNGSA",currentFrame,5))
  {
    GPS_DECODER_LOG("GPS decoder: Found GSA frame: \"%s\"\n", currentFrame);
    return parseFrameGSA(currentFrame);
  }
  // Satellites in view
  else if(!strncmp("GLGSV",currentFrame,5) || !strncmp("GPGSV",currentFrame,5) || !strncmp("BDGSV",currentFrame,5))
  {
    GPS_DECODER_LOG("GPS decoder: Found GSV frame: \"%s\"\n", currentFrame);
    return parseFrameGSV(currentFrame);
  }
  // Track Made Good and Ground Speed
  else if(!strncmp("GNVTG",currentFrame,5))
  {
    GPS_DECODER_LOG("GPS decoder: Found VTG frame: \"%s\"\n", currentFrame);
    return parseFrameVTG(currentFrame);
  }
  else
  {
    GPS_DECODER_LOG("GPS decoder: Could not decode frame: \"%s\"\n", currentFrame);
    return false;
  }

  // In any case return false
  return false;
}

// Recommended Minimum Navigation Information
bool GpsDecoderClass::parseFrameRMC(char *frame)
{
  char frameLocal[200] = {0};

  char identifierRaw[10] = {0};
  char utcTimeRaw[15] = {0};
  char statusRaw = 0;
  char latitudeRaw[15] = {0};
  char latitudeDirectionRaw = 0;
  char longitudeRaw[15] = {0};
  char longitudeDirectionRaw = 0;
  double speedOverGroundKnotsRaw = 0;
  double trackMadeGood = 0;
  char dateRaw[15] = {0};
  double magenticVariationDegreesRaw = 0;
  char magenticVariationDegreesStatusRaw = 0;

  // Save frame local, because we modify it
  strncpy(frameLocal,frame,sizeof(frameLocal)-1);

  // Jump to first fragment in frame
  char *fragment = strsep(&frame, ",");

  // Loop through all fragments in frame
  for(uint8_t i=0;i<12;i++)
  {
    switch (i)
    {
      case 0:
        strncpy(identifierRaw,fragment,sizeof(identifierRaw)-1);
        break;

      case 1:
        strncpy(utcTimeRaw,fragment,sizeof(utcTimeRaw)-1);
        break;

      case 2:
        statusRaw = *fragment;
        break;

      case 3:
        strncpy(latitudeRaw,fragment,sizeof(latitudeRaw)-1);
        break;

      case 4:
        latitudeDirectionRaw = *fragment;
        break;

      case 5:
        strncpy(longitudeRaw,fragment,sizeof(longitudeRaw)-1);
        break;

      case 6:
        longitudeDirectionRaw = *fragment;
        break;

      case 7:
        speedOverGroundKnotsRaw = atof(fragment);
        break;

      case 8:
        trackMadeGood = atof(fragment);
        break;

      case 9:
        strncpy(dateRaw,fragment,sizeof(dateRaw)-1);
        break;

      case 10:
        magenticVariationDegreesRaw = atof(fragment);
        break;

      case 11:
        magenticVariationDegreesStatusRaw = *fragment;
        continue;

      // something bad happened
      default:
        return false;
    }

    fragment = strsep(&frame, ",");
  }

  // Update timestamp
  if(strlen(utcTimeRaw)>0)
  {
    time.setTime(utcTimeRaw);
    time.commit();
  }
  
  // Update Latitude if valid
  if(statusRaw == 'A')
  {
    location.setLatitude(latitudeRaw);
    location.setLongitude(longitudeRaw);
    location.commit();
  }

  // Update speed
  speed.set(speedOverGroundKnotsRaw);
  speed.commit();

  // Update date
  if(strlen(dateRaw)>0)
  {
    date.setDate(dateRaw);
    date.commit();
  }

  return true;
}

// Global Positioning System Fix Data. Time, Position and fix related data for a GPS receiver
bool GpsDecoderClass::parseFrameGGA(char *frame)
{
  char frameLocal[200] = {0};

  char identifierRaw[10] = {0};
  char utcTimeRaw[15] = {0};
  char latitudeRaw[15] = {0};
  char latitudeDirectionRaw = 0;
  char longitudeRaw[15] = {0};
  char longitudeDirectionRaw = 0;
  uint8_t qualityIndicator = 0;
  uint8_t numberOfSatellitesInView = 0;
  float horizontalDilutionOfPrecision = 0.0;
  float antennaAltitudeBasedOnSeaLevel = 0.0;
  float geodialSeparation = 0.0;
  float ageOfDifeferentialGpsData = 0.0;
  uint16_t differentialReferenceStationId = 0;

  // Save frame local, because we modify it
  strncpy(frameLocal,frame,sizeof(frameLocal)-1);

  // Jump to first fragment in frame
  char *fragment = strsep(&frame, ",");

  // Loop through all fragments in frame
  for(uint8_t i=0;i<15;i++)
  {
    switch (i)
    {
      case 0:
        strncpy(identifierRaw,fragment,sizeof(identifierRaw)-1);
        break;

      case 1:
        strncpy(utcTimeRaw,fragment,sizeof(utcTimeRaw)-1);
        break;

      case 2:
        strncpy(latitudeRaw,fragment,sizeof(latitudeRaw)-1);
        break;

      case 3:
        latitudeDirectionRaw = *fragment;
        break;

      case 4:
        strncpy(longitudeRaw,fragment,sizeof(longitudeRaw)-1);
        break;

      case 5:
        longitudeDirectionRaw = *fragment;
        break;

      case 6:
        qualityIndicator = atoi(fragment);
        break;

      case 7:
        numberOfSatellitesInView = atoi(fragment);
        break;

      case 8:
        horizontalDilutionOfPrecision = atof(fragment);
        break;

      case 9:
        antennaAltitudeBasedOnSeaLevel = atof(fragment);
        break;

      case 10:
        break;

      case 11:
        geodialSeparation = atof(fragment);
        break;

      case 12:
        break;

      case 13:
        ageOfDifeferentialGpsData = atof(fragment);
        break;

      case 14:
        differentialReferenceStationId = atoi(fragment);
        // Skip last strsep
        continue;

      // something bad happened
      default:
        return false;
    }

    fragment = strsep(&frame, ",");
  }

  // Update time and commit it
  if(strlen(utcTimeRaw)>0)
  {
    time.setTime(utcTimeRaw);
    time.commit();
  }
  
  // Update position and commit
  if(strlen(latitudeRaw)>0 && strlen(longitudeRaw)>0)
  {
    location.setLatitude(latitudeRaw);
    location.setLongitude(longitudeRaw);
    location.commit();
  }

  // Set altitude
  altitude.set((int32_t)round(antennaAltitudeBasedOnSeaLevel));
  altitude.commit();

  return true;
}

// GPS DOP and active satellites
bool GpsDecoderClass::parseFrameGSA(char *frame)
{
  // https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GSA.html
  char frameLocal[200] = {0};

  char identifierRaw[10] = {0};
  char selectionMode = 0;             // M = manual, A = Automatic
  uint8_t fixedTypeLocal = 0;         // 1= not available, 2 = 2D, 3 = 3D fix
  uint8_t activeSatellites[12] = {0};
  double pdopLocal = 0;
  double hdopLocal = 0;
  double vdopLocal = 0;
  int8_t systemId = -1;               // 1=GPS, 2=GLONASS, 3=Galileo, 4=Baidu, 0=QZZ

  // Save frame local, because we modify it
  strncpy(frameLocal,frame,sizeof(frameLocal)-1);

  // Jump to first fragment in frame
  char *fragment = strsep(&frame, ",");

  // Loop through all fragments in frame
  for(uint8_t i=0;i<19;i++)
  {
    switch (i)
    {
      case 0:
        strncpy(identifierRaw,fragment,sizeof(identifierRaw)-1);
        break;

      case 1:
        selectionMode = *fragment;
        break;

      case 2:
        fixedTypeLocal = atoi(fragment);
        break;

      case 3:
      case 4:
      case 5:  
      case 6:  
      case 7:  
      case 8:  
      case 9:  
      case 10:  
      case 11:  
      case 12:  
      case 13:  
      case 14:  
        activeSatellites[i-3] = atoi(fragment);
        break;

      case 15:
        pdopLocal = atof(fragment);
        break;

      case 16:
        hdopLocal = atof(fragment);
        break;

      case 17:
        vdopLocal = atof(fragment);
        break;

      case 18:
        if(fragment == NULL){continue;}   // If no data stop here
        systemId = atof(fragment);
        continue;

      // something bad happened
      default:
        return false;
    }

    fragment = strsep(&frame, ",");
  }


  // Update hdop
  hdop.set(hdopLocal);
  hdop.commit();

  // Update vdop
  vdop.set(vdopLocal);
  vdop.commit();
  
  // Update pdop
  pdop.set(pdopLocal);
  pdop.commit();

  // Update fixed type
  fixedType.set(fixedTypeLocal);
  fixedType.commit();

  // Update active satellites list
  switch (systemId)
  {
    case 1: // GPS
      for (uint8_t i = 0; i < 12; i++)
      {
        satellites.gps.listOfActiveSatelliteIds[i].set(activeSatellites[i]);
        satellites.gps.listOfActiveSatelliteIds[i].commit();
      }
      break;

    case 2: // GLONASS
      for (uint8_t i = 0; i < 12; i++)
      {
        satellites.glonass.listOfActiveSatelliteIds[i].set(activeSatellites[i]);
        satellites.glonass.listOfActiveSatelliteIds[i].commit();
      }
      break;

    case 4: // Baidu
      for (uint8_t i = 0; i < 12; i++)
      {
        satellites.baidu.listOfActiveSatelliteIds[i].set(activeSatellites[i]);
        satellites.baidu.listOfActiveSatelliteIds[i].commit();
      }
      break;
    
    default:
      break;
  }
  
  return true;
}

// Satellites in view
bool GpsDecoderClass::parseFrameGSV(char *frame)
{
  char frameLocal[200] = {0};

  char identifierRaw[10] = {0};
  uint8_t totalNumberOfMessages = 0;
  uint8_t messageNumber = 0;            // Seems that maximum 4 sats are in one message, so we use this as static offset
  uint8_t satellitesInView = 0;
  uint8_t satelliteNumber[4] = {0};
  uint8_t satelliteElevation[4] = {0};
  uint16_t satelliteAzimuth[4] = {0};
  uint8_t satelliteSnr[4] = {0};

  // Save frame local, because we modify it
  strncpy(frameLocal,frame,sizeof(frameLocal)-1);

  // Jump to first fragment in frame
  char *fragment = strsep(&frame, ",");

  // Loop through all fragments in frame
  for(uint8_t i=0;i<20;i++)
  {
    switch (i)
    {
      case 0:
        strncpy(identifierRaw,fragment,sizeof(identifierRaw)-1);
        break;

      case 1:
        totalNumberOfMessages = atoi(fragment);
        break;

      case 2:
        messageNumber = atoi(fragment);
        break;

      case 3:
        satellitesInView = atoi(fragment);
        break;

      case 4:
        if(fragment==NULL){continue;} // If no satellite data continue here
        satelliteNumber[0] = atoi(fragment);
        break;
      case 5:
        satelliteElevation[0] = atoi(fragment);
        break;  
      case 6:
        satelliteAzimuth[0] = atoi(fragment);
        break; 
      case 7:
        satelliteSnr[0] = atoi(fragment);
        break;

      case 8:
        if(fragment==NULL){continue;} // If no satellite data continue here
        satelliteNumber[1] = atoi(fragment);
        break;
      case 9:
        satelliteElevation[1] = atoi(fragment);
        break;  
      case 10:
        satelliteAzimuth[1] = atoi(fragment);
        break; 
      case 11:
        satelliteSnr[1] = atoi(fragment);
        break;

      case 12:
        if(fragment==NULL){continue;} // If no satellite data continue here
        satelliteNumber[2] = atoi(fragment);
        break;
      case 13:
        satelliteElevation[2] = atoi(fragment);
        break;  
      case 14:
        satelliteAzimuth[2] = atoi(fragment);
        break; 
      case 15:
        satelliteSnr[2] = atoi(fragment);
        break;

      case 16:
        if(fragment==NULL){continue;} // If no satellite data continue here
        satelliteNumber[3] = atoi(fragment);
        break;
      case 17:
        satelliteElevation[3] = atoi(fragment);
        break;  
      case 18:
        satelliteAzimuth[3] = atoi(fragment);
        break; 
      case 19:
        satelliteSnr[3] = atoi(fragment);
        continue;

      // something bad happened
      default:
        return false;
    }

    fragment = strsep(&frame, ",");
  }

  // Update glonass satellites infos
  if(!strncmp("GLGSV",currentFrame,5))
  {
    // Update satellites in view
    satellites.glonass.numberSatellitesInView.set(satellitesInView);
    satellites.glonass.numberSatellitesInView.commit();

    // Update satellites infos
    for (uint8_t i = 0; i < 4; i++)
    {
      // Calculate position in array based on total sats in view divided by 4
      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].id.set(satelliteNumber[i]);
      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].id.commit();

      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].azimuth.set(satelliteAzimuth[i]);
      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].azimuth.commit();

      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].elevation.set(satelliteElevation[i]);
      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].elevation.commit();

      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].snr.set(satelliteSnr[i]);
      satellites.glonass.listOfSatellitesInView[(messageNumber-1)*4+i].snr.commit();
    }
  }
  // Update gps satellites infos
  else if(!strncmp("GPGSV",currentFrame,5))
  {
    // Update satellites in view
    satellites.gps.numberSatellitesInView.set(satellitesInView);
    satellites.gps.numberSatellitesInView.commit();

    // Update satellites infos
    for (uint8_t i = 0; i < 4; i++)
    {
      // Calculate position in array based on total sats in view divided by 4
      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].id.set(satelliteNumber[i]);
      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].id.commit();

      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].azimuth.set(satelliteAzimuth[i]);
      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].azimuth.commit();

      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].elevation.set(satelliteElevation[i]);
      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].elevation.commit();

      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].snr.set(satelliteSnr[i]);
      satellites.gps.listOfSatellitesInView[(messageNumber-1)*4+i].snr.commit();
    }
  }
  // Update baidu satellites infos
  else if(!strncmp("BDGSV",currentFrame,5))
  {
    // Update satellites in view
    satellites.baidu.numberSatellitesInView.set(satellitesInView);
    satellites.baidu.numberSatellitesInView.commit();

    // Update satellites infos
    for (uint8_t i = 0; i < 4; i++)
    {
      // Calculate position in array based on total sats in view divided by 4
      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].id.set(satelliteNumber[i]);
      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].id.commit();

      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].azimuth.set(satelliteAzimuth[i]);
      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].azimuth.commit();

      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].elevation.set(satelliteElevation[i]);
      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].elevation.commit();

      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].snr.set(satelliteSnr[i]);
      satellites.baidu.listOfSatellitesInView[(messageNumber-1)*4+i].snr.commit();
    }
  }
  
  return true;
}

// Satellites in view
bool GpsDecoderClass::parseFrameVTG(char *frame)
{
  char frameLocal[200] = {0};

  char identifierRaw[10] = {0};
  char valid = 0;
  double trackDegrees = 0;
  double speedKnots = 0;

  // Save frame local, because we modify it
  strncpy(frameLocal,frame,sizeof(frameLocal)-1);

  // Jump to first fragment in frame
  char *fragment = strsep(&frame, ",");

  // Loop through all fragments in frame
  for(uint8_t i=0;i<9;i++)
  {
    switch (i)
    {
      case 0:
        strncpy(identifierRaw,fragment,sizeof(identifierRaw)-1);
        break;

      case 1:
        trackDegrees = atof(fragment);
        break;

      case 2:
        valid = *fragment;
      case 3:
      case 4:
        break;
      
      case 5:
        speedKnots = atof(fragment);
        break;
      
      case 6:
      case 7:
        break;

      case 8:
        continue;

      // something bad happened
      default:
        return false;
    }

    fragment = strsep(&frame, ",");
  }

  // If data is valid
  if(valid == 'T')
  {
    // Update speed
    speed.set((double)speedKnots);
    speed.commit();

    // Update course
    course.set((double)trackDegrees);
    course.commit();
  }

  return true;
}




// ******************************************************************************************************
//
// Internals
//
// ******************************************************************************************************

// Replace in a string, using a needle with its replacement
void GpsDecoderClass::strReplace(char *stack, char *needle, char replacement)
{
  char *rep;
  // As long the needle is found in stack, replace it
  while((rep = strstr(stack,needle))!=NULL)
  {
    // Change current found needle with replacement
    *rep = replacement;
  }
}


// https://stackoverflow.com/questions/58244300/getting-the-error-undefined-reference-to-strsep-with-clang-and-mingw
// MinGw does not hat strsep
char *GpsDecoderClass::strsep(char **stringp, const char *delim)
{
   char *rv = *stringp;
    if (rv) {
        *stringp += strcspn(*stringp, delim);
        if (**stringp)
            *(*stringp)++ = '\0';
        else
            *stringp = 0; }
    return rv;
}


// Parses a ASCII hex nibbel to uint8
uint8_t GpsDecoderClass::fromHex(char a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}


// returns distance in meters between two positions, both specified
// as signed decimal-degrees latitude and longitude. Uses great-circle
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// Courtesy of Maarten Lamers
double GpsDecoderClass::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}


// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// Courtesy of Maarten Lamers
double GpsDecoderClass::courseTo(double lat1, double long1, double lat2, double long2)
{
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += GPS_DECODER_TWO_PI;
  }
  return degrees(a2);
}


// Return course as cardinal
const char *GpsDecoderClass::cardinal(double course)
{
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}


// Parse degrees in that funny NMEA format DDMM.MMMM
void GpsDecoderClass::parseDegrees(const char *term, RawDegreesClass &deg)
{
  uint32_t leftOfDecimal = (uint32_t)atol(term);
  uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
  uint32_t multiplier = 10000000UL;
  uint32_t tenMillionthsOfMinutes = minutes * multiplier;

  deg.deg = (int16_t)(leftOfDecimal / 100);

  while (isdigit(*term))
    ++term;

  if (*term == '.')
    while (isdigit(*++term))
    {
      multiplier /= 10;
      tenMillionthsOfMinutes += (*term - '0') * multiplier;
    }

  deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
  deg.negative = false;
}
