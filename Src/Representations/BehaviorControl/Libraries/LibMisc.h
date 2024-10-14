/**
 * @file LibMisc.h
 *
 * This file defines a representation that holds some generic utility functions.
 *  
 * Check Eigen.h, BHMath.h or Geometry.h in the Tools/Math folder for more math functions.
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <iostream>

STREAMABLE(LibMisc,
{
  /**
   * @brief Check if currentValue is close enough to target.
   * 
   * @param currentValue The value to check.
   * @param target The target value.
   * @param bound The tolerance.
   * 
   * @return [bool] true if currentValue is within bound of target.
   */
  FUNCTION(bool(float currentValue, float target, float bound)) isValueBalanced;
  
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
  FUNCTION(LocalPose2f(float x, float y)) glob2Rel;

  /**
   * @brief Transforms a position in global (field) coordinates into local (robot) coordinates.
   * 
   * @param v The position in global coordinates.
   * 
   * @return [LocalPose2f] The position in local coordinates.
   * 
   * @note This is a wrapper around the Eigen operation theRobotPose.inversePose * targetOnField
   */
  FUNCTION(LocalPose2f(GlobalVector2f v)) glob2Rel_v;

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
  FUNCTION(GlobalPose2f(float x, float y)) rel2Glob;

  /**
   * @brief Transforms a position in local (robot) coordinates into global (field) coordinates.
   * 
   * @param v The position in local coordinates.
   * 
   * @return [GlobalPose2f] The position in global coordinates.
   * 
   * @note This is a wrapper around the Eigen operation theRobotPose * positionRelative
   */
  FUNCTION(GlobalPose2f(LocalVector2f v)) rel2Glob_v;

  /**
   * @brief Converts an angle from radians to degrees.
   * 
   * @param x The angle in radians.
   * 
   * @return [DegAngle] The angle in degrees.
   */
  FUNCTION(DegAngle(RadAngle x)) radiansToDegree;

  /**
   * @brief Angle between two vectors in radians.
   * 
   * @param v1 The first vector.
   * @param v2 The second vector.
   * 
   * @return [RadAngle] The angle between the two vectors.
   */
  FUNCTION(RadAngle(Vector2f v1, Vector2f v2)) angleBetweenVectors;

  /**
   * @brief Angle between two vectors in radians in global coordinate (with the same center/start point)
   * 
   * @param start The start point of the vectors.
   * @param end1 The end point of the first vector.
   * @param end2 The end point of the second vector.
   * 
   * @return [RadAngle] The angle between the two vectors.
   */
  FUNCTION(RadAngle(GlobalVector2f start, GlobalVector2f end1, GlobalVector2f end2)) angleBetweenGlobalVectors;

  /**
   * @brief Angle between two vectors in radians in global coordinate (with the same center/start point)
   * 
   * @param start The start point of the vectors.
   * @param end1 The end point of the first vector.
   * @param end2 The end point of the second vector.
   * 
   * @return [RadAngle] The angle between the two vectors.
   */
  FUNCTION(bool(GlobalVector2f startBounds, GlobalVector2f endBound1, GlobalVector2f endBound2, float startRadius, float endRadius, GlobalVector2f point)) isInsideGlobalSector;

  /**
   * @brief Calculate the mirror angle of a point C wrt the line AB.
   * 
   * @param A The first point of the line.
   * @param B The second point of the line.
   * @param C The point to calculate the mirror angle of.
   * 
   * @return [RadAngle] The mirror angle of C wrt the line AB.
   */
  FUNCTION(RadAngle(Vector2f A, Vector2f B, Vector2f C)) getMirrorAngle;
  
  /**
   * @brief Calculate the angle of the robot to a target point.
   * 
   * @param target The target point.
   * 
   * @return [RadAngle] The angle of the robot to the target point.
   */
  FUNCTION(RadAngle(Vector2f target)) angleToTarget;

  RadAngle angleToGoal; // angle of this robot to the goal
  RadAngle angleToBall; // angle of this robot to the ball

  , // always add a comma after the last element
});
