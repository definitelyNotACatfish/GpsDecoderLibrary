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
// $DateClassTimeClass: 09.03.2023 $
//
// Creator: FKSN ()
// Compiler: Arduino PlattformIO 
//-----------------------------------------------------------------------------

#ifndef GPS_DECODER_H_
#define GPS_DECODER_H_

// ******************************************************************
// Includes
// ******************************************************************
#include <stdint.h>
#include <stdio.h>


// ******************************************************************
// Defines
// ******************************************************************
#define GPS_DECODER_DEBUG

#warning needs to be changed later
#ifdef GPS_DECODER_DEBUG
   #define GPS_DECODER_LOG(fmt,...) (printf(fmt,##__VA_ARGS__))
   #define GPS_DECODER_FLUSH()
#else
   #define GPS_DECODER_LOG(fmt,...)
   #define GPS_DECODER_FLUSH()
#endif

#warning needs to be removed later
#define millis()  5


#define GPS_DECODER_MPH_PER_KNOT          1.15077945
#define GPS_DECODER_MPS_PER_KNOT          0.51444444
#define GPS_DECODER_KMPH_PER_KNOT         1.852
#define GPS_DECODER_MILES_PER_METER       0.00062137112
#define GPS_DECODER_KM_PER_METER          0.001
#define GPS_DECODER_FEET_PER_METER        3.2808399
#define GPS_DECODER_MAX_FIELD_SIZE        200

#define GPS_DECODER_DEG_TO_RAD            0.017453292519943295769236907684886
#define GPS_DECODER_RAD_TO_DEG            57.295779513082320876798154814105
#define GPS_DECODER_TWO_PI                6.283185307179586476925286766559
#define radians(deg)                      ((deg)*GPS_DECODER_DEG_TO_RAD)
#define degrees(rad)                      ((rad)*GPS_DECODER_RAD_TO_DEG)
#define sq(x)                             ((x)*(x))



// ******************************************************************
// Class
// ******************************************************************
class GpsDecoderClass
{
   private:
      // Private sub classes
      class IntegerClass
      {
         friend class GpsDecoderClass;

         public:
            bool isValid() const    { return valid; }
            bool isUpdated() const  { return updated; }
            uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)0xffffffff; }
            uint32_t value()        { updated = false; return val; }

            IntegerClass();

         private:
            bool valid, updated;
            uint32_t lastCommitTime;
            uint32_t val, newval;
            void commit();
            void set(const char *term);
            void set(uint32_t val);
      };

      class DecimalClass
      {
         friend class GpsDecoderClass;

         public:
            bool isValid() const    { return valid; }
            bool isUpdated() const  { return updated; }
            uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)0xffffffff; }
            double value()          { updated = false; return val; }

            DecimalClass();

         private:
            bool valid, updated;
            uint32_t lastCommitTime;
            double val, newval;
            void commit();
            void set(double value);
      };

      class AltitudeClass : public DecimalClass
      {
         friend class GpsDecoderClass;

         public:
         double meters()       { return value(); }
         double miles()        { return GPS_DECODER_MILES_PER_METER * value(); }
         double kilometers()   { return GPS_DECODER_KM_PER_METER * value(); }
         double feet()         { return GPS_DECODER_FEET_PER_METER * value(); }
      };

      class DateClass
      {
         friend class GpsDecoderClass;

         private:
            bool valid, updated;
            uint32_t date, newDate;
            uint32_t lastCommitTime;
            void commit();
            void setDate(const char *term);

         public:
            bool isValid() const       { return valid; }
            bool isUpdated() const     { return updated; }
            uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)0xffffffff; }

            uint32_t value()           { updated = false; return date; }
            uint16_t year();
            uint8_t month();
            uint8_t day();

            DateClass();
      };

      class RawDegreesClass
      {
         friend class GpsDecoderClass;

         public:
            RawDegreesClass();
            uint16_t deg;
            uint32_t billionths;
            bool negative;
      };
      
      class LocationClass
      {
         friend class GpsDecoderClass;

         private:
            bool valid=false, updated=false;
            RawDegreesClass rawLatData, rawLngData, rawNewLatData, rawNewLngData;
            uint32_t lastCommitTime;
            void commit();
            void setLatitude(const char *term);
            void setLongitude(const char *term);

         public:
            bool isValid() const    { return valid; }
            bool isUpdated() const  { return updated; }
            uint32_t age() const    { return valid ? millis() - lastCommitTime : (uint32_t)0xffffffff; }
            const RawDegreesClass &rawLat()     { updated = false; return rawLatData; }
            const RawDegreesClass &rawLng()     { updated = false; return rawLngData; }
            double lat();
            double lng();

            LocationClass();
      };

      class SatelliteSystemClass
      {
         friend class GpsDecoderClass;

         class satInViewEntry                                  // Contains one entry of a satellite in view list
         { 
            public:
            IntegerClass id;                                   // Number of the satellite
            IntegerClass elevation;                            // Elevation in 0...90 deg
            IntegerClass azimuth;                              // Azimuth in 0...359 deg
            IntegerClass snr;                                  // SNR 0 - 99 dbHz. 0 if unused
         };

         private:

         public:
            IntegerClass listOfActiveSatelliteIds[12];         // List of active satellite ids
            IntegerClass numberSatellitesInView;               // Total number of satellites in view
            satInViewEntry listOfSatellitesInView[12];         // List of visible satellites
      };

      class SatellitesClass
      {
         friend class GpsDecoderClass;

         private:

         public:
         SatelliteSystemClass gps, baidu, glonass;             // Satellites for each satellite system
      };

      class SpeedClass : public DecimalClass
      {
         friend class GpsDecoderClass;

         public:
         double knots()    { return value(); }
         double mph()      { return GPS_DECODER_MPH_PER_KNOT * value(); }
         double mps()      { return GPS_DECODER_MPS_PER_KNOT * value(); }
         double kmph()     { return GPS_DECODER_KMPH_PER_KNOT * value(); }
      };

      class TimeClass
      {
         friend class GpsDecoderClass;
         public:
            bool isValid() const       { return valid; }
            bool isUpdated() const     { return updated; }
            uint32_t age() const       { return valid ? millis() - lastCommitTime : (uint32_t)0xffffffff; }

            uint32_t value()           { updated = false; return time; }
            uint8_t hour();
            uint8_t minute();
            uint8_t second();
            uint8_t centisecond();

            TimeClass();

         private:
            bool valid, updated;
            uint32_t time, newTime;
            uint32_t lastCommitTime;
            void commit();
            void setTime(const char *term);
      };


      // parsing state variables
      uint8_t calculatedChecksum;                                           // On the fly calculated checksum
      char currentFrame[GPS_DECODER_MAX_FIELD_SIZE];                        // Stores a complete gps frame without $,\r,\n -> "GPRMC,162614,A,5230.5900,N,01322.3900,E,10.0,90.0,131006,1.2,E,A*13"
      uint8_t currentFrameOffset;                                           // Current offset in currentFrame
      bool waitForFrameStart;                                               // Wait for frame start
      bool blockReadChecksumInCalculation;                                  // Block following read chars, if a * has found in frame

      // statistics
      uint32_t decodedCharCount;                                            // Endless increasing number of encoded chars
      uint32_t sentencesWithFixCount;                                       // Endless increasing number of fixed sentences
      uint32_t failedChecksumCount;                                         // Endless increasing number of failed checksum messages
      uint32_t passedChecksumCount;                                         // Endless increasing number of passed checksum messages

      // internal utilities
      uint8_t fromHex(char a);                                              // get nibble from ASCII hex "A" -> 10
      static void parseDegrees(const char *term, RawDegreesClass &deg);     // Parse term into degrees
      static void strReplace(char *stack, char *needle, char replacement);  // Replace a char in a string with another char
      static char *strsep(char **stringp, const char *delim);               // WinGW does not have strsep

      bool parseFrame();                                                    // Parses a frame and updates sub classes. Returns true, if checksum is valid, else false
      bool parseFrameGGA(char *frame);                                      // Subfunction of parseFrame
      bool parseFrameRMC(char *frame);                                      // Subfunction of parseFrame
      bool parseFrameGSA(char *frame);                                      // Subfunction of parseFrame
      bool parseFrameGSV(char *frame);                                      // Subfunction of parseFrame
      bool parseFrameVTG(char *frame);                                      // Subfunction of parseFrame


   public:
      GpsDecoderClass();                                                    // Constructor

      bool decode(char currentChar);                                        // process one character received from GPS

      uint32_t charsProcessed()   const { return decodedCharCount; }        // Returns total number of processed chars, since class has been created
      uint32_t sentencesWithFix() const { return sentencesWithFixCount; }   // Returns total number of fixed sentences, since class has been created
      uint32_t failedChecksum()   const { return failedChecksumCount; }     // Returns total number of failed checksum messages
      uint32_t passedChecksum()   const { return passedChecksumCount; }     // Returns total number of failed checksum messages

      static double distanceBetween(double lat1, double long1, double lat2, double long2);   // Distance between to coordinates
      static double courseTo(double lat1, double long1, double lat2, double long2);          // CourseClass in degrees between course 1 and 2
      static const char *cardinal(double course);                                            // Converts course to cardinal "N", "NW"

      LocationClass location;                                              // Location data
      DateClass date;                                                      // Date data
      TimeClass time;                                                      // Time data
      SpeedClass speed;                                                    // Speed data
      DecimalClass course;                                                 // Course data
      AltitudeClass altitude;                                              // Altitude data
      SatellitesClass satellites;                                          // Satellites data
      DecimalClass hdop;                                                   // HDOP
      DecimalClass vdop;                                                   // VDOP
      DecimalClass pdop;                                                   // PDOP
      IntegerClass fixedType;                                              // 1=Nofix, 2=2D, 3=3d
};




#endif