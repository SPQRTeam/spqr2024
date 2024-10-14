/**
 * @file OwnKickoffStrikerCard.cpp
 *
 * This file implements a behavior for the own kick-off.
 *
 * @author Flavio Volpi
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/BehaviorControl/KickoffState.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Settings.h"
#include <iostream>
#include <Eigen/Geometry>

using Line2 = Eigen::Hyperplane<float,2>;

CARD(OwnKickoffStrikerCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookAtBall),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkPotentialField),
  CALLS(LookAtGlobalBall),
  CALLS(GoToBallAndDribble),
  CALLS(ArmObstacleAvoidance),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  REQUIRES(KickoffState),
  REQUIRES(LibMisc),
  REQUIRES(LibSpec),
  REQUIRES(LibStriker),
  REQUIRES(LibObstacles),
  REQUIRES(KickInfo),
  REQUIRES(ExtendedGameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});


class OwnKickoffStrikerCard : public OwnKickoffStrikerCardBase
{
  bool preconditions() const override
  {
    return theKickoffState.isKickoff && theGameInfo.kickingTeam == Global::getSettings().teamNumber;
  }

  bool postconditions() const override
  {
    return !preconditions();
  }

  Vector2f passTarget;
  KickInfo::KickType kickType;
  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime) goto goToBallAndDribble;
      }
      action
      {
        theLookAtBallSkill();
        theStandSkill();
      }
    }
    
    state(goToBallAndDribble)
    {
      transition
      {
        if(theLibSpec.isTherePassKickoffCondition(passTarget)){
          if(passTarget.y() > 0){
              kickType = KickInfo::walkSidewardsLeftFootToLeft;
              goto passTheBall;
          }
          else{
              kickType = KickInfo::walkSidewardsRightFootToRight;
              goto passTheBall;
          }
        }
      }

      action
      {
        theArmObstacleAvoidanceSkill();
        theGoToBallAndDribbleSkill(theLibMisc.angleToTarget(theLibStriker.strikerDribblePoint), false, 0.7);
      }
    }

    state(passTheBall)
    {
      transition
      {
        if(!theLibSpec.isTherePassKickoffCondition(passTarget))
            goto goToBallAndDribble;
      }
      action
      {
        
        float distance = (theRobotPose.translation - passTarget).norm();
        theGoToBallAndKickSkill(theLibMisc.angleToTarget(passTarget), kickType, true, distance);      
      }
    }
  }

};

MAKE_CARD(OwnKickoffStrikerCard);
