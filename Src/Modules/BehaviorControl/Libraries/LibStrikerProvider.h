/**
 * @file LibStrikerProvider.h
 * 
 * This file defines a module that provides some utilities (primarily) for the striker.
 */

#pragma once
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/spqr_representations/GameState.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Module/Module.h"

MODULE(LibStrikerProvider,
{,
  REQUIRES(RobotPose),
  USES(OpponentGoalModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamBallModel),
  REQUIRES(ObstacleModel), 
  REQUIRES(BallModel),
  REQUIRES(KickInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBall),   
  REQUIRES(LibMisc),
  REQUIRES(LibSpec),
  REQUIRES(GameState), 
  REQUIRES(RobotInfo), 
  USES(PlayerRole),
  PROVIDES(LibStriker),

  // LOADS_PARAMETERS(
  // {,
  // }),
});

class LibStrikerProvider : public LibStrikerProviderBase
{
private:
  
  /**
   * @brief Updates the LibStriker representation
   * 
   * @param libStriker The LibStriker representation to update
   */
  void update(LibStriker& libStriker) override;

  /**
   * @brief Returns the striker position (global coordinates)
   * 
   * @return The striker position [GlobalVector2f]
   */
  GlobalVector2f getStrikerPosition() const;

  /**
   * @brief Returns the striker position in freeKick situations (global coordinates)
   * 
   * @return The striker position [GlobalVector2f]
   */
  GlobalVector2f getStrikerPositionSpecial() const;

  /**
   * @brief Returns the dribble point for the striker (global coordinates)
   * 
   * @return The dribble point [GlobalVector2f]
   */
  GlobalVector2f getStrikerDribblePoint();

  /**
  * @brief Returns the best kick type for the striker
  * 
  * @param kickAsap a bool attribute, is true when you have to kick as soon as possible
  * @param kickRight a bool attribute, is true when you want to kick with right foot
  * 
  * @return The best kick type [KickInfo::KickType]
  */
  KickInfo::KickType getKick(bool kickAsap, bool kickRight) const;

   /**
   * @brief Get the best kick type to reach the target.
   * 
   * @param target The target point in global coordinates
   * 
   * @return [KickInfo::KickType] The best kick type to reach the target.
   */
  KickInfo::KickType getWalkKick(GlobalVector2f target) const;

  /**
   * @brief Returns whether the robot should kick or not
   * 
   * @param currentlyKicking a bool attribute, is true when the robot is currently kicking
   * @param kickIntervalThreshold a float attribute, the threshold for the kick interval
   * 
   * @return If the robot should kick or not [bool]
   */
  bool shouldKick(bool currentlyKicking, float kickIntervalThreshold) const;


  /** 
   * Returns the global y coord point we are looking at on the opponent groundline
   */
  float projectGazeOntoOpponentGroundline() const;

  /** Provides a vector with the point of beginning and finish of goal areas free from opponent coverage
   * @param myPose pose of the robot
   * @param opponents the opponent vector (global coordinates)
   * @return vector of free areas
   */
  std::vector<FreeGoalTargetableArea> computeFreeAreas(float minimumDiscretizedAreaSize) const;

  /** Provides the best point to shoot at inside the goal.
   * If the opponent goal is completely occluded returns the field center (exactly (0,0))
   * @param shootASAP If set to true, if the robot is near the goal, shoot in the spot nearest to where you're looking at ("As Soon As Possible"), else use the heuristic to decide
   * @param forceHeuristic If set to true, always use the heuristic to decide where to shoot
   * @return the Vector2f of the position selected to shoot
   * **/
  Vector2f goalTarget(bool shootASAP, bool forceHeuristic) const;

  /** Variant of goalTarget that also return the FreeGoalTargetableArea associated to the target point.
    * @param shootASAP If set to true has the robot will shoot to the nearest accessible point, located inside the nearest targetable area
    * @param forceHeuristic If set to true, always use the heuristic to decide where to shoot
    * @return the Vector2f of the position selected to shoot
    * **/
  std::pair<Vector2f, FreeGoalTargetableArea> goalTargetWithArea(bool shootASAP, bool forceHeuristic) const;

  /** @author Emanuele Musumeci
   * Given a certain point in input proivdes its projection on the opponent ground line by the robot's perspective
   * @param x x global coordinate of point to be projected
   * @param y y global coordinate of point to be projected
   * @return Global y coord of the projected point
   */
  float projectPointOntoOpponentGroundline(float x, float y) const;

  /** Tells whether two segments are overlapping or not
   * @param l1 left limit of first segment
   * @param r1 right limit of the first segment
   * @param l2 left limit of the second segment
   * @param r2 right limit of the second segment
   * @return bool value
   * **/
  bool areOverlappingSegmentsOnYAxis (float l1, float r1, float l2, float r2) const;

  /** Provides a float value representing a score for each FreeGoalTargetableArea Determined by computeFreeAreas
   * @param leftLimit left limit of the free targetable area
   * @param rightLimit right limit of the free targetable area
   * @param poles_weight relative weight in the final utility for the poles
   * @param opponents_weight relative weight in the final utility for the opponents
   * @param teammates_weight relative weight in the final utility for the teammates
   * @return value assigned after area evaluation
   * **/
  float areaValueHeuristic(float leftLimit, float rightLimit, float poles_weight = 1, float opponents_weight = 1, float teammates_weight = 1) const;

};
