//-----------------------------------------------------------------------------
//
/// @file
/// @brief This file contains the implementation of sub class DateClass for GpsDecoderClass
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
#include <stdlib.h>

// ******************************************************************
// Constructor
// ******************************************************************
GpsDecoderClass::DateClass::DateClass()
{
    valid = false;
    updated = false;
    date = 0;
}


// ******************************************************************
// Methods
// ******************************************************************
uint16_t GpsDecoderClass::DateClass::year()
{
   updated = false;
   uint16_t year = date % 100;
   return year + 2000;
}

uint8_t GpsDecoderClass::DateClass::month()
{
   updated = false;
   return (date / 100) % 100;
}

uint8_t GpsDecoderClass::DateClass::day()
{
   updated = false;
   return date / 10000;
}

void GpsDecoderClass::DateClass::setDate(const char *term)
{
   newDate = atol(term);
}

void GpsDecoderClass::DateClass::commit()
{
   date = newDate;
   lastCommitTime = millis();
   valid = updated = true;
}