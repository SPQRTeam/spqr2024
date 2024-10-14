/**
 * @file LibMiscProvider.h
 * 
 * This file defines a module that holds some generic utility functions.
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
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
  // }),
});

class LibMiscProvider : public LibMiscProviderBase
{
private:
  
  /**
   * @brief Update the LibMisc representation.
   * 
   * @param libMisc The LibMisc representation to update.
   */
  void update(LibMisc& libMisc) override;

  /**
   * @brief Check if currentValue is close enough to target.
   * 
   * @param currentValue The value to check.
   * @param target The target value.
   * @param bound The tolerance.
   * 
   * @return [bool] true if currentValue is within bound of target.
   */
  bool isValueBalanced(float currentValue, float target, float bound) const;

  /**
   * @brief Transforms a position in global (field) coordinates into local (robot) coordinates.
   * 
   * @param x The x coordinate of the position in global coordinates.
   * @param y The y coordinate of the position in global coordinates.
   * 
   * @return [LocalPose2f] The position in local coordinates.
   * 
   * @note This is a wrapper around the Eigen operation theRobotPose.inversePose * targetOnField
   */
  LocalPose2f glob2Rel(float x, float y) const;

  /**
   * @brief Transforms a position in local (robot) coordinates into global (field) coordinates.
   * 
   * @param x The x coordinate of the position in local coordinates.
   * @param y The y coordinate of the position in local coordinates.
   * 
   * @return [GlobalPose2f] The position in global coordinates.
   * 
   * @note This is a wrapper around the Eigen operation theRobotPose * positionRelative
   */
  GlobalPose2f rel2Glob(float x, float y) const;

  /**
   * @brief Converts an angle from radians to degrees.
   * 
   * @param x The angle in radians.
   * 
   * @return [DegAngle] The angle in degrees.
   */
  DegAngle radiansToDegree(RadAngle x) const;
  
  /**
   * @brief Angle between two vectors in radians.
   * 
   * @param v1 The first vector.
   * @param v2 The second vector.
   * 
   * @return [RadAngle] The angle between the two vectors.
   */
  RadAngle angleBetweenVectors(const Vector2f& v1, const Vector2f& v2) const;

  /**
   * @brief Angle between two vectors in radians in global coordinate (with the same center/start point)
   * 
   * @param start The start point of the vectors.
   * @param end1 The end point of the first vector.
   * @param end2 The end point of the second vector.
   * 
   * @return [RadAngle] The angle between the two vectors.
   */
  RadAngle angleBetweenGlobalVectors(const GlobalVector2f& start, const GlobalVector2f& end1, const GlobalVector2f& end2) const;

  /**
   * @brief Check if the point is inside the sector (between startRadius and endRadius) 
   * delimited by the vectors startBounds->endBound1 and startBounds->endBound2
   * 
   * @param startBounds The start point of the sector.
   * @param endBound1 The end point of the first vector.
   * @param endBound2 The end point of the second vector.
   * @param startRadius The start radius of the sector.
   * @param endRadius The end radius of the sector.
   * @param point The point to check.
   * 
   * @return [bool] true if the point is inside the sector.
   */
  bool isInsideGlobalSector(const GlobalVector2f startBounds, const GlobalVector2f endBound1, const GlobalVector2f endBound2, float startRadius, float endRadius, const GlobalVector2f point) const;

  /**
   * @brief Calculate the mirror angle of the vector BC with respect to the vector BA
   * 
   * @param A The start point of the vectors.
   * @param B The end point of the first vector.
   * @param C The end point of the second vector.
   * 
   * @return [RadAngle] The mirror angle.
   */
  RadAngle getMirrorAngle(Vector2f A, Vector2f B, Vector2f C);

  /**
   * @brief Calculate the angle to the target point
   * 
   * @param target The target point.
   * 
   * @return [RadAngle] The angle to the target point.
   */
  RadAngle angleToTarget(const Vector2f target) const;
};
