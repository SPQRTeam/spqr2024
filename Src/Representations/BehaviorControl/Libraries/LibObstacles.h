/**
 * @file LibObstacles.h
 *
 * This file defines a representation that holds some utilities functions
 * for handling obstacles, including but not limiting to opponents.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibObstacles,
{
  /** 
   * Returns the pose of the nearest opponent.
   * Uses the LOCAL ObstacleModel, so it only considers visible opponents
   * and returns IN LOCAL COORDINATES.
   */
  FUNCTION(Pose2f()) nearestOpponent;

  /** 
   * Returns the pose of the nearest opponent that satisfies a filter condition,
   * to be given as a lambda function.
   * Uses the LOCAL ObstacleModel, so it only considers visible opponents
   * and works IN LOCAL COORDINATES.
   */
  //FUNCTION(Pose2f(std::function<bool(Obstacle)> filterPredicate)) nearestOpponentWithFilter;

  /** Returns whether there is an opponent within radius distance of point. */
  FUNCTION(bool(const Vector2f& point, float radius)) areThereOpponentsNearby;

  /** Returns whether there is an opponent on the own side of the field. */
  FUNCTION(bool()) opponentOnOurField;

  FUNCTION(bool(const Vector2f& point, float radius)) areThereTeammatesNearby;

  /** Returns whether there is any obstacle close to point. */
  FUNCTION(bool(const Vector2f& point)) obstacleExistsAroundPoint,
});
