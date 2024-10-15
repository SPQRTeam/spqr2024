/**
 * @file LibJolly.h
 *
 * This file defines a representation that holds some utilities (primarily) for the jolly.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibJolly,
{
  /** 
   * Provides the jolly reference position
   */
  FUNCTION(Vector2f()) getJollyPosition;
  FUNCTION(Vector2f()) getJollyPositionGraph;
  FUNCTION(Vector2f()) getJollyPositionSpecial;

  FUNCTION(float(Vector2f position)) inversePassageUtilityModel;
  ,
});
