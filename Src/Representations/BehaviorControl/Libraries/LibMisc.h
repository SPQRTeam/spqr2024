/**
 * @file LibMisc.h
 *
 * This file defines a representation that holds some generic utility functions that don't go anywhere else.
 * Examples include simple mathematical utilities and more.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <iostream>

STREAMABLE(LibMisc,
{
  /** Returns -1 if the direction points to the own field, 1 if it points to the opponent field **/
  FUNCTION(int(const Vector2f& localDirection)) localDirectionToField;
  
  /** Provides the distance between 2 Pose2f **/
  FUNCTION(float(const Pose2f& p1, const Pose2f& p2)) distance;
  
  /** Provides the distance between 2 Points (Vector2f) **/
  FUNCTION(float(const Vector2f& p1, const Vector2f& p2)) distanceVec;

  /**
   * Maps value from interval [fromIntervalMin, fromIntervalMax] to interval [toIntervalMin, toIntervalMax]
   * NOTE: BHMath.h might have implemented this as well
   */
  FUNCTION(float(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax)) mapToInterval;

  /**
   * Checks if currentValue is close enough to target.
   * bound indirectly determines the tolerance.
   */
  FUNCTION(bool(float currentValue, float target, float bound)) isValueBalanced;
  
  /**
   * Calculates the angle of the given point wrt the robot position and orientation.
   * Point to be given in global coordinates.
   * NOTE: with Eigen, it's the oneliner:  glob2Rel(x,y).translation.angle()
   */
  FUNCTION(float(float x, float y)) angleToTarget;

  /**
   * Transforms a position in global (field) coordinates into local (robot) coordinates.
   * NOTE: with Eigen, it's the oneliner:  theRobotPose.inversePose * targetOnField
   *       where targetOnField = Vector2f(x,y)
   */
  FUNCTION(Pose2f(float x, float y)) glob2Rel;

  /**
   * Transforms a position in local (robot) coordinates into global (field) coordinates.
   * NOTE: with Eigen, it's the oneliner:  theRobotPose * positionRelative
   *       where positionRelative = Vector2f(x,y)
   */
  FUNCTION(Pose2f(float x, float y)) rel2Glob;

  /**
   * Computes the norm of vector (x,y).
   * NOTE: with Eigen, it's the oneliner:  Vector2f(x,y).norm()
   */
  FUNCTION(float(float x, float y)) norm;

  /**
   * Converts an angle from radians to degrees.
   */
  FUNCTION(float(float x)) radiansToDegree;

  /**
   * Angle of the line passing through two points.
   * WARNING: [torch, 2022]
   *     Something about the implementation doesn't convince me.
   *     See LibMiscProvider.cpp for details.
   *     Anyway, this is used in a couple modules, so I can't change this at the moment.
   *     Future users, make sure this is really what you want before using this.
   */
  FUNCTION(float(Vector2f p1, Vector2f p2)) angleBetweenPoints;

  /**
   * Angle between two vectors in radians in global coordinate (with the same center/start point)
   */
  FUNCTION(float(Vector2f start, Vector2f end1, Vector2f end2)) angleBetweenGlobalVectors;

  /**
   * Check if the point is inside the sector (btw startRadius and endRadius) delimited from the vectors (all in global coordinates)
   * startBounds->endBound1  &  startBounds->endBound2
   */
  FUNCTION(bool(Vector2f startBounds, Vector2f endBound1, Vector2f endBound2, float startRadius, float endRadius, Vector2f point)) isInsideGlobalSector;

  FUNCTION(bool(Vector2f point, Vector2f bottom, Vector2f left, Vector2f right, Vector2f up)) isInsideGlobalRectangle;
  /**
   * Angle between two vectors in radians 
   */
  FUNCTION(float(Vector2f v1, Vector2f v2)) angleBetweenVectors;

  /**
   * Takes a target position and a radius for the obstacles and clips the target position to avoid obstacles
   * Uses the obstacles from the TeamPlayersModel
  */
  FUNCTION(Vector2f(Vector2f target, float radius, bool consider_teammate_as_obstacles)) clipTargetOutsideObstacles;
  
  FUNCTION(Angle(Vector2f A, Vector2f B, Vector2f C)) getMirrorAngle;
  /** 
  * Calculates the angle of the given point wrt the robot position and orientation.
  */
  FUNCTION(Angle(Vector2f target)) calcAngleToTarget,

  // Angle of this robot to opponent goal
  (float) angleToGoal,

  // Angle of this robot to the ball
  (float) angleToBall,
});
