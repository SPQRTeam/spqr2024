/**
 * @file DummyCard.cpp
 *
 * This file implements a basic behavior for the Dummy.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Tools/Settings.h"
#include "Tools/Math/BHMath.h"
#include <Eigen/Geometry>


CARD(OpponentFreeKickStrikerCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(LookForScan),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkPotentialField),
  CALLS(LookAtGlobalBall),
  CALLS(WalkToPoint),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(LibMisc),
  REQUIRES(LibPass),
  REQUIRES(LibStriker),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class OpponentFreeKickStrikerCard : public OpponentFreeKickStrikerCardBase
{
  bool preconditions() const override
  {

    bool isSpecialCase = theGameInfo.setPlay == SET_PLAY_GOAL_KICK
                      || theGameInfo.setPlay == SET_PLAY_KICK_IN
                      || theGameInfo.setPlay == SET_PLAY_PENALTY_KICK
                      || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK;
    
    bool isOurKick = theGameInfo.kickingTeam == Global::getSettings().teamNumber;

    return !isOurKick && isSpecialCase;
  }

  bool postconditions() const override
  {
    return false;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
        bool isSpecialCase = theGameInfo.setPlay == SET_PLAY_GOAL_KICK
                      || theGameInfo.setPlay == SET_PLAY_CORNER_KICK
                      || theGameInfo.setPlay == SET_PLAY_KICK_IN
                      || theGameInfo.setPlay == SET_PLAY_PENALTY_KICK
                      || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK;

        if( !isSpecialCase ) goto pass;
      }

      action
      {
        bool ballSeen = theFieldBall.ballWasSeen(4500);
        theLookForScanSkill();
        Vector2f target_point = theLibMisc.glob2Rel(theLibStriker.strikerPosition.x(), theLibStriker.strikerPosition.y()).translation;
        theWalkToPointSkill(target_point);
      }
    } 

    state(pass){
      transition {}
      action {
        Vector2f passTarget;
        float utility;
        std::tie(passTarget, utility) = theLibPass.getBestPassageSpecial();
        float distance = (theRobotPose.translation - passTarget).norm();
        KickInfo::KickType kickType = theLibStriker.getWalkKick(passTarget);

        theGoToBallAndKickSkill(theLibMisc.angleToTarget(passTarget), kickType, true, distance);
      }
    }
  }
};

MAKE_CARD(OpponentFreeKickStrikerCard);
