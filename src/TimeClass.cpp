//-----------------------------------------------------------------------------
//
/// @file
/// @brief This file contains the implementation of sub class TimeClass for GpsDecoderClass
///
/// <please insert here the optional more detail description>
///
//-----------------------------------------------------------------------------
// $Author: FKSN $
// $Change: $
// $DateTime: 10.03.2023 $
//
// Creator: FKSN ()
// Compiler: Arduino PlattformIO 
//-----------------------------------------------------------------------------

// ******************************************************************
// Includes
// ******************************************************************
#include "gpsDecoder.h"
#include "stdlib.h"

// ******************************************************************
// Constructor
// ******************************************************************
GpsDecoderClass::TimeClass::TimeClass()
{
   valid = false;
   updated = false;
   time = 0;
}


// ******************************************************************
// Methods
// ******************************************************************
uint8_t GpsDecoderClass::TimeClass::hour()
{
   updated = false;
   return time / 1000000;
}

uint8_t GpsDecoderClass::TimeClass::minute()
{
   updated = false;
   return (time / 10000) % 100;
}

uint8_t GpsDecoderClass::TimeClass::second()
{
   updated = false;
   return (time / 100) % 100;
}

uint8_t GpsDecoderClass::TimeClass::centisecond()
{
   updated = false;
   return time % 100;
}

void GpsDecoderClass::TimeClass::commit()
{
   time = newTime;
   lastCommitTime = millis();
   valid = updated = true;
}

void GpsDecoderClass::TimeClass::setTime(const char *term)
{
   // Convert ascii to double
   double time = atof(term);

   // Convert time from double to uint32_t
   newTime = time*100;
}
