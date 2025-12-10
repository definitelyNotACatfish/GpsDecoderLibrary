#include <iostream>
#include "gpsDecoder.h"


const char *gpsStream =
  "$GNGGA,165520.000,0.95387,N,0.24919,E,1,07,2.7,101.0,M,48.3,M,,*4C\r\n"
  
  "$GNGLL,0.95387,N,0.24919,E,165520.000,A,A*44\r\n"
  
  "$GNGSA,A,3,10,16,,,,,,,,,,,9.7,2.7,9.3,1*36\r\n"
  "$GNGSA,A,3,21,28,34,37,,,,,,,,,9.7,2.7,9.3,4*3F\r\n"
  "$GNGSA,A,3,67,,,,,,,,,,,,9.7,2.7,9.3,2*32\r\n"
  
  "$GPGSV,2,1,06,08,,,21,10,56,137,27,16,49,200,26,18,,,18,0*65\r\n"
  "$GPGSV,2,2,06,23,,,28,26,18,178,,0*5B\r\n"
  "$BDGSV,1,1,04,21,30,056,33,28,37,280,26,34,30,092,35,37,29,279,36,0*7C\r\n"
  "$GLGSV,1,1,04,70,,,31,86,,,27,85,,,29,67,30,120,29,0*4F\r\n"
  
  "$GNRMC,165520.000,A,5000.95387,N,0.24919,E,0.00,181.50,180323,,,A,V*0F\r\n"
  
  "$GNVTG,181.50,T,,M,0.00,N,0.00,K,A*2E\r\n"
  
  "$GNZDA,165520.000,18,03,2023,00,00*44\r\n"
  
  "$GPTXT,01,01,01,ANTENNA OPEN*25\r\n"
  ;

int main()
{
    GpsDecoderClass gpsDecoder;

    while(*gpsStream)
    {
        gpsDecoder.decode(*gpsStream++);
    }

    // LocationClass location;                                              // Location data
    GPS_DECODER_LOG("location: lat: %f, lng: %f\n",gpsDecoder.location.lat(),gpsDecoder.location.lng());

    // DecimalClass hdop, vdop, pdop
    GPS_DECODER_LOG("hdop: %f, vdop: %f, pdop: %f\n",gpsDecoder.hdop.value(),gpsDecoder.vdop.value(),gpsDecoder.pdop.value());

    // IntegerClass fixedType;                                              // 1=Nofix, 2=2D, 3=3d
    GPS_DECODER_LOG("fixedType: %u\n",gpsDecoder.fixedType.value());

    // DateClass date;                                                      // Date data
    // TimeClass time;                                                      // Time data
    GPS_DECODER_LOG("date: %02u.%02u.%04u %02u:%02u:%02u\n",gpsDecoder.date.day(),gpsDecoder.date.month(),gpsDecoder.date.year(),gpsDecoder.time.hour(),gpsDecoder.time.minute(),gpsDecoder.time.second());

    // SpeedClass speed;                                                    // Speed data
    // DecimalClass course;                                                 // Course data
    // AltitudeClass altitude;                                              // Altitude data
    GPS_DECODER_LOG("speed: %f km/h, course: %f deg, altitude: %f m\n",
      gpsDecoder.speed.value(),
      gpsDecoder.course.value(),
      gpsDecoder.altitude.meters());

    // SatellitesClass satellites;                                          // Satellites data
    GPS_DECODER_LOG("Baidu:\n");
    GPS_DECODER_LOG(" Satellites in view: %u\n", gpsDecoder.satellites.baidu.numberSatellitesInView.value());
    GPS_DECODER_LOG(" List of active satellites ids: ");
    for (uint8_t i = 0; i < 12; i++)
    {
      if(gpsDecoder.satellites.baidu.listOfActiveSatelliteIds[i].value() != 0)
      {
        GPS_DECODER_LOG("%u,", gpsDecoder.satellites.baidu.listOfActiveSatelliteIds[i].value());
      }
    }
    GPS_DECODER_LOG("\n List of satellites in view: \n");
    for (uint8_t i = 0; i < 12; i++)
    {
      if(gpsDecoder.satellites.baidu.listOfSatellitesInView[i].id.value() != 0)
      {
        GPS_DECODER_LOG("   id: %2u  elevation: %3u deg  azimuth: %3u deg  snr: %3u dbHz\n", 
          gpsDecoder.satellites.baidu.listOfSatellitesInView[i].id.value(),
          gpsDecoder.satellites.baidu.listOfSatellitesInView[i].elevation.value(),
          gpsDecoder.satellites.baidu.listOfSatellitesInView[i].azimuth.value(),
          gpsDecoder.satellites.baidu.listOfSatellitesInView[i].snr.value());
      }
    }

    GPS_DECODER_LOG("GPS:\n");
    GPS_DECODER_LOG(" Satellites in view: %u\n", gpsDecoder.satellites.gps.numberSatellitesInView.value());
    GPS_DECODER_LOG(" List of active satellites ids: ");
    for (uint8_t i = 0; i < 12; i++)
    {
      if(gpsDecoder.satellites.gps.listOfActiveSatelliteIds[i].value() != 0)
      {
        GPS_DECODER_LOG("%u,", gpsDecoder.satellites.gps.listOfActiveSatelliteIds[i].value());
      }
    }
    GPS_DECODER_LOG("\n List of satellites in view: \n");
    for (uint8_t i = 0; i < 12; i++)
    {
      if(gpsDecoder.satellites.gps.listOfSatellitesInView[i].id.value() != 0)
      {
        GPS_DECODER_LOG("   id: %2u  elevation: %3u deg  azimuth: %3u deg  snr: %3u dbHz\n", 
          gpsDecoder.satellites.gps.listOfSatellitesInView[i].id.value(),
          gpsDecoder.satellites.gps.listOfSatellitesInView[i].elevation.value(),
          gpsDecoder.satellites.gps.listOfSatellitesInView[i].azimuth.value(),
          gpsDecoder.satellites.gps.listOfSatellitesInView[i].snr.value());
      }
    }

    GPS_DECODER_LOG("GLONASS:\n");
    GPS_DECODER_LOG(" Satellites in view: %u\n", gpsDecoder.satellites.glonass.numberSatellitesInView.value());
    GPS_DECODER_LOG(" List of active satellites ids: ");
    for (uint8_t i = 0; i < 12; i++)
    {
      if(gpsDecoder.satellites.glonass.listOfActiveSatelliteIds[i].value() != 0)
      {
        GPS_DECODER_LOG("%u,", gpsDecoder.satellites.glonass.listOfActiveSatelliteIds[i].value());
      }
    }
    GPS_DECODER_LOG("\n List of satellites in view: \n");
    for (uint8_t i = 0; i < 12; i++)
    {
      if(gpsDecoder.satellites.glonass.listOfSatellitesInView[i].id.value() != 0)
      {
        GPS_DECODER_LOG("   id: %2u  elevation: %3u deg  azimuth: %3u deg  snr: %3u dbHz\n", 
          gpsDecoder.satellites.glonass.listOfSatellitesInView[i].id.value(),
          gpsDecoder.satellites.glonass.listOfSatellitesInView[i].elevation.value(),
          gpsDecoder.satellites.glonass.listOfSatellitesInView[i].azimuth.value(),
          gpsDecoder.satellites.glonass.listOfSatellitesInView[i].snr.value());
      }
    }
  
    return 0;
}