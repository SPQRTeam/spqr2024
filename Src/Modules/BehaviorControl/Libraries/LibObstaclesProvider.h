/**
 * @file LibObstaclesProvider.h
 * 
 * See LibObstacles
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Tools/Module/Module.h"

MODULE(LibObstaclesProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(ObstacleModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(LibMisc),
  PROVIDES(LibObstacles),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibObstaclesProvider : public LibObstaclesProviderBase
{
private:
  
  /**
   * Updates LibObstacles
   * @param libObstacles The representation provided
   */
  void update(LibObstacles& libObstacles) override;


  // ===== IMPLEMENTATIONS OF LibObstacles =====

  /** 
   * Returns the pose of the nearest opponent.
   * Uses the LOCAL ObstacleModel, so it only considers visible opponents
   * and returns IN LOCAL COORDINATES.
   */
  Pose2f nearestOpponent() const;

  /** 
   * Returns the pose of the nearest opponent that satisfies a filter condition,
   * to be given as a lambda function.
   * Uses the LOCAL ObstacleModel, so it only considers visible opponents
   * and works IN LOCAL COORDINATES.
   */
  Pose2f nearestOpponentWithFilter(std::function<bool(Obstacle)> filterPredicate) const;

  /** Returns whether there is an opponent within radius distance of point. */
  bool areThereOpponentsNearby(const Vector2f& point, float radius) const;

  bool areThereTeammatesNearby(const Vector2f& point, float radius) const;

  /** Returns whether there is an opponent on the own side of the field. */
  bool opponentOnOurField() const;

  /** Returns whether there is any obstacle close to point. */
  bool obstacleExistsAroundPoint(const Vector2f& point) const;


  // ===== FOR INTERNAL USE =====


};
