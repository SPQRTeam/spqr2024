/**
 * @file LibDefender.h
 *
 * This file defines a representation that holds the defenders position.
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"


STREAMABLE(LibDefender,
{
  GlobalVector2f defenderonePosition; // Target point of the defenderone in global coordinates
  GlobalVector2f defendertwoPosition; // Target point of the defendertwo in global coordinates
  
  , // always put a comma at the end 
});
