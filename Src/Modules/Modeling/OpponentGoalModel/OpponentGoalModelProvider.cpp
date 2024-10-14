/**
 * @file Modules/Modeling/OpponentGoalModel/OpponentGoalModelProvider.cpp
 *
 * This module was used for debugging purposes but all the values are now loaded directly 
 * 
 * TODO uncomment this, can't be bothered by an unrelated module while porting the libs
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "OpponentGoalModelProvider.h"

OpponentGoalModelProvider::OpponentGoalModelProvider(){}

void OpponentGoalModelProvider::update(OpponentGoalModel& opponentGoalModel)
{
    if(!initDone)
        init(opponentGoalModel);
        
    #ifdef TARGET_SIM
    if(opponentGoalModel.graphicalDebug && theGameInfo.state == STATE_PLAYING && thePlayerRole.role==PlayerRole::RoleType::striker)
    {
        opponentGoalModel.freeGoalTargetableAreas = theLibStriker.computeFreeAreas(opponentGoalModel.goalTargetAreaMinSize);
        opponentGoalModel.myGazeProjection = theLibStriker.projectGazeOntoOpponentGroundline();
        opponentGoalModel.shootASAPGoalTarget = theLibStriker.goalTarget(true, false);
        /* TODO: Raplace approachAndKick as is no more available
        if(theBehaviorStatus.activity == BehaviorStatus::Activity::approachAndKick && theBehaviorStatus.shootingTo.x() != 0 && theBehaviorStatus.shootingTo.y() != 0)
        {
            opponentGoalModel.utilityGoalTarget = theBehaviorStatus.shootingTo;
        }
        else
        {
            opponentGoalModel.utilityGoalTarget = theLibStriker.goalTarget(false, true);
        }
        */ 
        if(opponentGoalModel.freeGoalTargetableAreas.size()!=0)
        {
            opponentGoalModel.maxUtilityValue = opponentGoalModel.freeGoalTargetableAreas.front().value;
            opponentGoalModel.minUtilityValue = opponentGoalModel.freeGoalTargetableAreas.back().value;
        }
        else
        {
            opponentGoalModel.maxUtilityValue=0;
            opponentGoalModel.minUtilityValue=0;
        }
    }
    #endif
}

void OpponentGoalModelProvider::init(OpponentGoalModel& opponentGoalModel){
    opponentGoalModel.graphicalDebug = GRAPHICAL_DEBUG;
    opponentGoalModel.showWalls = SHOW_WALLS;
    opponentGoalModel.showRays = SHOW_RAYS;

    opponentGoalModel.useAreaDiscretization = (USE_AREA_DISCRETIZATION==1 ? true : false);

    opponentGoalModel.areaSizeMultiplicator = AREA_SIZE_MULTIPLICATOR;
    opponentGoalModel.goalTargetMinOffsetFromSideMultiplicator = GOAL_TARGET_MIN_OFFSET_FROM_SIDE_MULTIPLICATOR;
    opponentGoalModel.goalTargetDistanceThreshold = GOAL_TARGET_DISTANCE_THRESHOLD;
    opponentGoalModel.goalTargetObstacleInflation = GOAL_TARGET_OBSTACLE_INFLATION;
    
    opponentGoalModel.goalPoleMinimumTargetOffset = GOAL_POLE_MINIMUM_TARGET_OFFSET;

    opponentGoalModel.goalTargetAreaMinSize = theBallSpecification.radius*AREA_SIZE_MULTIPLICATOR;
    opponentGoalModel.goalTargetMinOffsetFromSide = opponentGoalModel.goalTargetAreaMinSize * GOAL_TARGET_MIN_OFFSET_FROM_SIDE_MULTIPLICATOR;

    opponentGoalModel.utilityPolesWeight = UTILITY_POLES_WEIGHT;
    opponentGoalModel.utilityNearestOpponentWeight = UTILITY_NEAREST_OPPONENT_WEIGHT;
    opponentGoalModel.utilityOpponentsWeight = UTILITY_OPPONENTS_WEIGHT;
    opponentGoalModel.utilityOpponentsXDistanceThreshold = UTILITY_OPPONENTS_X_DISTANCE_THRESHOLD;
    opponentGoalModel.utilityTeammatesWeight = UTILITY_TEAMMATES_WEIGHT;
    initDone = true;
}

MAKE_MODULE(OpponentGoalModelProvider, modeling);