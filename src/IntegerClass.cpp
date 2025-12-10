//-----------------------------------------------------------------------------
//
/// @file
/// @brief This file contains the implementation of sub class IntegerClass for GpsDecoderClass
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
GpsDecoderClass::IntegerClass::IntegerClass()
{
    valid = false;
    updated = false;
    val = 0;
}


// ******************************************************************
// Methods
// ******************************************************************
void GpsDecoderClass::IntegerClass::commit()
{
   val = newval;
   lastCommitTime = millis();
   valid = updated = true;
}

void GpsDecoderClass::IntegerClass::set(const char *term)
{
   newval = atol(term);
}

void GpsDecoderClass::IntegerClass::set(uint32_t val)
{
   newval = val;
}