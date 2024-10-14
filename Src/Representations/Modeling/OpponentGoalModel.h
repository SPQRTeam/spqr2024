/**
 * @file Representations/Modeling/OpponentGoalModel.h
 *
 * Declaration of struct OpponentGoalModel, that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "FreeGoalTargetableArea.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct OpponentGoalModel
 *
 * This representation is used only for debugging purposes
 * 
 * Struct containing modeling info about the opponent goal targetable areas
 */

STREAMABLE(OpponentGoalModel,
{

  /** Draws model on the field */
  void draw() const,

  //Loaded from opponentGoalModelProvider.cfg, see OpponentGoalModelProvider for explanation
  (float) areaSizeMultiplicator,
  (float) goalTargetMinOffsetFromSideMultiplicator,
  (float) goalTargetDistanceThreshold,
  (float) goalTargetObstacleInflation,

  (float) goalPoleMinimumTargetOffset,

  (bool) graphicalDebug,
  (bool) showWalls,
  (bool) showRays,

  (std::vector<FreeGoalTargetableArea>) freeGoalTargetableAreas,   /** Vector of discretized segments of goal line available as targets*/
  (float)(0.f) myGazeProjection,                                   /** Projection of robots gaze on the goal line*/

  (bool) useAreaDiscretization,                                    /** Divide targetable areas into smaller segments all of the same size 
                                                                        (useful for assigning a utility value to each one*/
  (float) goalTargetMinOffsetFromSide,                             /** Minimum offset from side of discretized free area when choosing where to shoot */
  (float) goalTargetAreaMinSize,                                   /** Minimum discretized free area size (discretized areas can be bigger)*/
  (Pose2f) utilityGoalTarget,                                      /** Chosen target */
  (Pose2f) shootASAPGoalTarget,                                    /** Most immediate target */
  (float)(0.1) maxUtilityValue,                                    /** Utility value of the best target available (base on utility function) */
  (float)(0.0) minUtilityValue,                                    /** Utility value of the worst target available (base on utility function) */

  (float)(1.0) utilityPolesWeight,                               /** weight of the position of the poles in the utility */
  (float)(1.0) utilityNearestOpponentWeight,                     /** weight of the nearest opponent in the utility */
  (float)(1.0) utilityOpponentsWeight,                           /** weight of the position of the opponents in the utility */
  (float)(1.0) utilityTeammatesWeight,                           /** weight of the position of the teammates in the utility */
  (float)(0) utilityOpponentsXDistanceThreshold,                 /** max distance from opponent ground line for opponents that count towards the utility */
});
