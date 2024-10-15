/**
 * @file LibLibero.h
 *
 * This file defines a representation that holds some utilities (primarily) for the libero.
 *
 * @author Flavio Volpi
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibLibero,
{
  /** 
   * Provides the libero reference position
   */
  FUNCTION(Vector2f()) getLiberoPosition;
  FUNCTION(Vector2f()) getLiberoPositionGraph;
  ,
});
