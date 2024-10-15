/**
 * @file LibSpec.h
 *
 * This file defines a representation that holds some generic utility functions for the special balls.
 *
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <tuple>

STREAMABLE(LibSpec,
{

  FUNCTION(bool(Vector2f obs1, Vector2f obs2)) compare_obstacles;

  /**
   * Compute the angle where to kick during attacking corner, returns the couple (angle_to_kick, best_angle), 
   * where angle to kick is the angle from ground line and best angle is the larger angle found between two obstacles
  */
  FUNCTION(std::tuple<float,float>()) calcAngleCorner;

  /**
  * Compute the target point where to kick during attacking corner (in global coordinates)
  */
  FUNCTION(Vector2f(float angle, float radius)) targetCornerPoint;

  /**
  * Compute in which zone (1, 2, 3) the point is (i.e. the most dangerous opponent);
    params
    side: l=left, r=right
    return: 0=most dangerous zone, 1=near dangerous zone, 2=far dangerous zone
  */
  FUNCTION(int(Vector2f point)) calcCornerZone;

  /**
  * Compute the target point where to kick during defense corner (in global coordinates)
  */
  FUNCTION(Vector2f()) targetDefenseCornerPoint;

  /**
   * Compute the orthogonal distance from P to a segment between point A and B in global coordinates
  */
  FUNCTION(double(Vector2f start, Vector2f end, Vector2f pos_self)) pointToSegmentDistance;

  FUNCTION(Vector2f(bool onBall)) getPositionCoveringDangerousOpponent;

  /**
   * Checks if the corridor (practically a 2D cylinder) is covered by an object, which can be a teammate or an opponent
  */
  FUNCTION(bool(const Vector2f start, const Vector2f end, const float width, const Vector2f teammate)) isCorridorCovered;

  FUNCTION(Vector2f(const Vector2f start, const Vector2f end)) nearestPointOnCorridor;

  FUNCTION(Vector2f()) getMostDangerousOpponent;

  FUNCTION(Vector2f(Vector2f leftEnd, Vector2f rightEnd, float width)) freeCorridor;

  FUNCTION(bool(Vector2f start, Vector2f end, float width, Vector2f obstacle)) isObstacleInCorridor;

  FUNCTION(bool()) isGoaliePlaying;


  FUNCTION(Vector2f(const Vector2f robot)) freeKickWall,

});
