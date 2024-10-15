/**
 * @file LibMiscProvider.h
 * 
 * See LibMisc
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MODULE(LibMiscProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamPlayersModel),
  REQUIRES(RobotInfo),
  PROVIDES(LibMisc),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibMiscProvider : public LibMiscProviderBase
{
private:
  
  /**
   * Updates LibMisc
   * @param libMisc The representation provided
   */
  void update(LibMisc& libMisc) override;

  /** Returns -1 if the direction points to the own field, 1 if it points to the opponent field **/
  int localDirectionToField(const Vector2f& localDirection) const;

  /** Provides the distance between 2 Pose2f **/
  float distance(const Pose2f& p1, const Pose2f& p2) const;

  /** Provides the distance between 2 Points (Vector2f) **/   //TODO: Flavio&Valerio
  float distanceVec(const Vector2f& p1, const Vector2f& p2) const;

  //Maps value from interval [fromIntervalMin, fromIntervalMax] to interval [toIntervalMin, toIntervalMax]
  float mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) const;

  /**
   * Checks if currentValue is close enough to target.
   * bound indirectly determines the tolerance.
   */
  bool isValueBalanced(float currentValue, float target, float bound) const;

  /**
   * Calculates the angle of the given point wrt the robot position and orientation.
   * Point to be given in global coordinates.
   */
  float angleToTarget(float x, float y) const;

  /**
   * Transforms a position in global (field) coordinates into local (robot) coordinates.
   */
  Pose2f glob2Rel(float x, float y) const;

  /**
   * Transforms a position in local (robot) coordinates into global (field) coordinates.
   */
  Pose2f rel2Glob(float x, float y) const;

  /**
   * Computes the norm of vector (x,y).
   */
  float norm(float x, float y) const;

  /**
   * Converts an angle from radians to degrees.
   */
  float radiansToDegree(float x) const;
  
  /**
   * Angle of the line passing through two points.
   * WARNING: [torch, 2022]
   *     Something about the implementation doesn't convince me.
   *     See LibMiscProvider.cpp for details.
   *     Anyway, this is used in a couple modules, so I can't change this at the moment.
   *     Future users, make sure this is really what you want before using this.
   */
  float angleBetweenPoints(const Vector2f& p1, const Vector2f& p2) const;

  /**
   * Angle between two vectors in radians 
   */
  float angleBetweenVectors(const Vector2f& v1, const Vector2f& v2) const;

  /**
   * Angle between two vectors in radians in global coordinate (with the same center/start point)
   */
  float angleBetweenGlobalVectors(const Vector2f& start, const Vector2f& end1, const Vector2f& end2) const;

  /**
   * Check if the point is inside the sector (btw startRadius and endRadius) delimited from the vectors 
   * startBounds->endBound1  &  startBounds->endBound2
   */
  bool isInsideGlobalSector(const Vector2f startBounds, const Vector2f endBound1, const Vector2f endBound2, float startRadius, float endRadius, const Vector2f point) const;

  bool isInsideGlobalRectangle(const Vector2f point, const Vector2f bottom, const Vector2f left, const Vector2f right, const Vector2f up) const;

  Vector2f clipTargetOutsideObstacles(const Vector2f& target, const float& radius, const bool& consider_teammate_as_obstacles) const;
  /**
   * Calculate the angle of the given point wrt the robot position and orientation.
  */
  Angle calcAngleToTarget(const Vector2f target) const;

  Angle getMirrorAngle(Vector2f A, Vector2f B, Vector2f C);

};
