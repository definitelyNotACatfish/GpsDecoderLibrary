//-----------------------------------------------------------------------------
//
/// @file
/// @brief This file contains the implementation of sub class LocationClass for GpsDecoderClass
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

// ******************************************************************
// Constructor
// ******************************************************************
GpsDecoderClass::LocationClass::LocationClass()
{
    valid = false;
    updated = false;
}


// ******************************************************************
// Methods
// ******************************************************************
double GpsDecoderClass::LocationClass::lat()
{
   updated = false;
   double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
   return rawLatData.negative ? -ret : ret;
}

double GpsDecoderClass::LocationClass::lng()
{
   updated = false;
   double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
   return rawLngData.negative ? -ret : ret;
}

void GpsDecoderClass::LocationClass::commit()
{
   rawLatData = rawNewLatData;
   rawLngData = rawNewLngData;
   lastCommitTime = millis();
   valid = updated = true;
}

void GpsDecoderClass::LocationClass::setLatitude(const char *term)
{
   GpsDecoderClass::parseDegrees(term, rawNewLatData);
}

void GpsDecoderClass::LocationClass::setLongitude(const char *term)
{
   GpsDecoderClass::parseDegrees(term, rawNewLngData);
}