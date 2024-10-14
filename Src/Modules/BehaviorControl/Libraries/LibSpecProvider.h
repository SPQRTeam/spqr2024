/**
 * @file LibSpecProvider.h
 * 
 * See LibSpec
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"

#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include <list>

#include "Representations/Communication/TeamData.h"
#include <iostream>



MODULE(LibSpecProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(LibMisc),
  REQUIRES(LibObstacles),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamBallModel),
  REQUIRES(ObstacleModel),
  REQUIRES(LibPosition),

  REQUIRES(TeamData),

  USES(OpponentGoalModel),
  PROVIDES(LibSpec),
});

class LibSpecProvider : public LibSpecProviderBase
{
private:
  /**
   * Updates LibSpec
   * @param libSpec The representation provided
   */
  void update(LibSpec& libSpec) override;


  bool compare_obstacles(Vector2f obs1, Vector2f obs2) const;

  /**
   * Compute the angle where to kick during attacking corner, returns the couple (angle_to_kick, best_angle), 
   * where angle to kick is the angle from ground line and best angle is the larger angle found between two obstacles
  */
  std::tuple<float, float> calcAngleCorner() const; 
  
  /**
  * Compute the target point where to kick during attacking corner (in global coordinates)
  */
  Vector2f targetCornerPoint(float angle, float radius) const;
  /**
  * Compute in which zone (1, 2, 3) the point is (i.e. the most dangerous opponent);
    params
    side: l=left, r=right
    return: 0=most dangerous zone, 1=near dangerous zone, 2=far dangerous zone
  */
  int calcCornerZone(const Vector2f& point) const;

  /**
  * Compute the target point where to kick during defense corner (in global coordinates)
  */
  Vector2f targetDefenseCornerPoint() const;

  /**
   * Compute a convenient position for the striker to go to in a situation of defensive kick in
  */
  Vector2f getPositionCoveringDangerousOpponent(bool onBall) const;


  /**
   * Computes the point on the segment between start (most probably the ball) and end (the goal or another obstacle) 
   * with least distance from self, such that the corridor is covered as quickly as possible.
  */
  Vector2f nearestPointOnCorridor(const Vector2f start, const Vector2f end) const;

  /**
   * Orthogonal distance from P to the segment between A and B
  */
  double pointToSegmentDistance(Vector2f start, Vector2f end, Vector2f pos_self) const;

  Vector2f freeCorridor(const Vector2f leftEnd, const Vector2f rightEnd, const float width) const;

  bool isObstacleInCorridor(const Vector2f start, const Vector2f end, const float width, const Vector2f obstacle) const;

  Vector2f getMostDangerousOpponent() const;

  Vector2f freeKickWall(Vector2f robot) const;

  bool isGoaliePlaying() const;

  bool isTherePassKickoffCondition(Vector2f& passTarget);
};
