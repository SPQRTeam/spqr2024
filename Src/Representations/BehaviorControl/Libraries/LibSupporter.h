/**
 * @file LibSupporter.h
 *
 * This file defines a representation that holds the supporter position.
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibSupporter,
{
  GlobalVector2f supporterPosition; // Target point of the Supporter in global coordinates

  , // always put a comma at the end 
});
