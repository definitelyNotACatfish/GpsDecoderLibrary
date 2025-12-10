//-----------------------------------------------------------------------------
//
/// @file
/// @brief This file contains the implementation of sub class DecimalClass for GpsDecoderClass
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
GpsDecoderClass::DecimalClass::DecimalClass()
{
    valid = false;
    updated = false;
    val = 0;
}


// ******************************************************************
// Methods
// ******************************************************************
void GpsDecoderClass::DecimalClass::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void GpsDecoderClass::DecimalClass::set(double value)
{
   newval = value;
}