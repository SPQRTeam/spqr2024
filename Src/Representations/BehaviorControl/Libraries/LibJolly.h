/**
 * @file LibJolly.h
 *
 * This file defines a representation that holds the jolly position.
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibJolly,
{
  GlobalVector2f jollyPosition; // Target point of the Jolly in global coordinates

  , // always put a comma at the end 
});
