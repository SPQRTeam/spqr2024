/**
 * @file LibDefender.h
 *
 * This file defines a representation that holds some utilities (primarily) for the defender.
 *
 * @author @neverorfrog
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"


STREAMABLE(LibDefender,
{
  /** 
   * Provides the defender reference position based on 
   * intersection between the center of 
   * the ball and the center of the goal
   */
  FUNCTION(Vector2f()) getDefenderonePosition;
  FUNCTION(Vector2f()) getDefenderonePositionSpecial;
  FUNCTION(Angle(bool ballSeen, Vector2f robotPosition)) getDefenderoneOrientation;

  /** 
   * Provides the defender two reference position based on 
   * the position of the ball and the movements of the teams
   */
  FUNCTION(Vector2f()) getDefendertwoPosition;
  FUNCTION(Vector2f()) getDefendertwoPositionSpecial;
  FUNCTION(Angle(bool ballSeen, Vector2f robotPosition)) getDefendertwoOrientation;

  FUNCTION(Vector2f(Vector2f segmentStart, Vector2f segmentEnd, float minRadius, float maxRadius)) targetOnSemiCircle,
});
