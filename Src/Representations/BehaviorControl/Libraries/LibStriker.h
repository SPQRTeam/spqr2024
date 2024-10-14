/**
 * @file LibStriker.h
 *
 * This file defines a representation that holds some utilities (primarily) for the striker.
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Modeling/FreeGoalTargetableArea.h"
#include "Representations/Configuration/KickInfo.h"

STREAMABLE(LibStriker,
{

  /**
  * @brief Returns the best kick type for the striker
  * 
  * @param kickAsap a bool attribute, is true when you have to kick as soon as possible
  * @param kickRight a bool attribute, is true when you want to kick with right foot
  * 
  * @return The best kick type [KickInfo::KickType]
  */
  FUNCTION(KickInfo::KickType(bool kickAsap, bool kickRight)) getKick;

  /**
   * @brief Get the best kick type to reach the target.
   * 
   * @param target The target point in global coordinates
   * 
   * @return [KickInfo::KickType] The best kick type to reach the target.
   */
  FUNCTION(KickInfo::KickType(GlobalVector2f target)) getWalkKick;
  
  /**
   * @brief Returns whether the robot should kick or not
   * 
   * @param currentlyKicking a bool attribute, is true when the robot is currently kicking
   * @param kickIntervalThreshold a float attribute, the threshold for the kick interval
   * 
   * @return If the robot should kick or not [bool]
   */
  FUNCTION(bool(bool currentlyKicking, float kickIntervalThreshold)) shouldKick;


  /** 
   * @author Emanuele Musumeci
   * Returns the global y coord point we are looking at on the opponent groundline
   */
  FUNCTION(float()) projectGazeOntoOpponentGroundline;

  /** Provides a vector with the point of beginning and finish of goal areas free from opponent coverage
   * @param myPose pose of the robot
   * @param opponents the opponent vector (global coordinates)
   * @return vector of free areas
   */
  FUNCTION(std::vector<FreeGoalTargetableArea>(float minimumDiscretizedAreaSize)) computeFreeAreas;
  
  /** Provides the best point to shoot at inside the goal.
   * If the opponent goal is completely occluded returns the field center (exactly (0,0))
   * @param shootASAP If set to true, if the robot is near the goal, shoot in the spot nearest to where you're looking at ("As Soon As Possible"), else use the heuristic to decide
   * @param forceHeuristic If set to true, always use the heuristic to decide where to shoot
   * @return the Vector2f of the position selected to shoot
   * **/
  FUNCTION(Vector2f(bool shootASAP, bool forceHeuristic)) goalTarget;
  
  /** Variant of goalTarget that also return the FreeGoalTargetableArea associated to the target point.
    * @param shootASAP If set to true has the robot will shoot to the nearest accessible point, located inside the nearest targetable area
    * @param forceHeuristic If set to true, always use the heuristic to decide where to shoot
    * @return the Vector2f of the position selected to shoot
    * **/
  FUNCTION(std::pair<Vector2f, FreeGoalTargetableArea>(bool shootASAP, bool forceHeuristic)) goalTargetWithArea;


  GlobalVector2f strikerPosition;     // Target point of the Striker in global coordinates
  GlobalVector2f strikerDribblePoint; // Target point for the dribbling in global coordinates

  , // always put a comma at the end 
});
