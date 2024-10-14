/**
 * @file LibLibero.h
 *
 * This file defines a representation that holds the libero position.
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibLibero,
{
  GlobalVector2f liberoPosition; // Target point of the Libero in global coordinates

  , // always put a comma at the end 
});
