/**
 * @file Modules/Modeling/OpponentGoalModel/OpponentGoalModelProvider.h
 *
 * This file implements a module that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include <iostream>

MODULE(OpponentGoalModelProvider,
{,
    REQUIRES(LibStriker),
    REQUIRES(FieldDimensions),
    REQUIRES(BallSpecification),
    REQUIRES(GameInfo),
    REQUIRES(PlayerRole),
    PROVIDES(OpponentGoalModel),
    USES(BehaviorStatus),
    LOADS_PARAMETERS(
    {,
      //goalTarget constants
      (bool) GRAPHICAL_DEBUG,                                      /** Shows a graphical debug render in SimRobot */
      (bool) SHOW_WALLS,                                           /** Show targetable areas as 3D colored walls */
      (bool) SHOW_RAYS,                                            /** Show rays for free targetable areas */     

      (bool) USE_AREA_DISCRETIZATION,                              /** Use free areas discretization + utility-based choice of target */ 

      (float) AREA_SIZE_MULTIPLICATOR,                             /** ball_radius * AREA_SIZE_MULTIPLICATOR is the min targetable area size in goalTarget */
      (float) GOAL_TARGET_MIN_OFFSET_FROM_SIDE_MULTIPLICATOR,      /** multiplicator for the minimum offset for target from left/right side of targetable area size in goalTarget */
      (float) GOAL_TARGET_DISTANCE_THRESHOLD,                      /** distance from opponent goal at which the utility-based targeting is deactivated in favour of the shootASAP mode */
      (float) GOAL_TARGET_OBSTACLE_INFLATION,                      /** multiplicator used to inflate the projection of obstacles based on their distance */
    
      (float) GOAL_POLE_MINIMUM_TARGET_OFFSET,                     /** minimum distance of the leftmost/rightmost targetable areas from the poles left/right limit */

      (float) UTILITY_POLES_WEIGHT,                                /** weight of the position of the poles in the utility */
      (float) UTILITY_NEAREST_OPPONENT_WEIGHT,                     /** weight of the nearest opponent in the utility */
      (float) UTILITY_OPPONENTS_WEIGHT,                            /** weight of the position of the opponents in the utility */
      (float) UTILITY_TEAMMATES_WEIGHT,                            /** weight of the position of the teammates in the utility */
      (float) UTILITY_OPPONENTS_X_DISTANCE_THRESHOLD,              /** max distance from opponent ground line for opponents that count towards the utility */
    }),
});

/**
 * @class OpponentGoalModelProvider
 * A module that provides the model of the opponent goal
 */
class OpponentGoalModelProvider: public OpponentGoalModelProviderBase
{
public:
  /** Constructor*/
  OpponentGoalModelProvider();

private:
  bool initDone = false;
  void update(OpponentGoalModel& opponentGoalModel) override;
  void init(OpponentGoalModel& opponentGoalModel);
};
