/**
 * @file BasicStrikerCard.cpp
 *
 * This file implements a basic striker behavior for the striker.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/spqr_representations/GameState.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Debugging/Debugging.h"
#include <iostream>

CARD(BasicStrikerCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(GoToBallAndDribble),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(Dribble),
  CALLS(LookActive),
  CALLS(LookAtBall),
  CALLS(WalkToBallAndKick),
  CALLS(WalkPotentialField),
  CALLS(LookAtGlobalBall),
  CALLS(Say),
  CALLS(WalkToPoint),
  CALLS(ArmObstacleAvoidance),

  REQUIRES(LibMisc),
  REQUIRES(OpponentGoalModel),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBall),
  REQUIRES(LibStriker),
  REQUIRES(LibObstacles),
  USES(LibDefender),
  REQUIRES(LibPass),
  REQUIRES(FrameInfo),
  REQUIRES(TeamData),
  REQUIRES(GameState),
  REQUIRES(ObstacleModel),
  REQUIRES(PathPlanner),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(OwnTeamInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (float)(80) kickIntervalThreshold,
    (float)(0.5) shouldPassThreshold,
  }),
});

Vector2f MAGIC_VECTOR = Vector2f(-9999, -9999);

class BasicStrikerCard : public BasicStrikerCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  Vector2f chosenTarget;
  KickInfo::KickType chosenKickType;
  bool chosenDoItAsap;
  Vector2f ballPositionAtLastTargetChoice;
  Vector2f passTarget;

  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime && theGameState.state != GameState::State::opponentPenaltyKick) goto goToBallAndDribble;
      }

      action
      {
        ballPositionAtLastTargetChoice = MAGIC_VECTOR;
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(goToBallAndDribble)
    {
      transition
      {
        if(theGameState.state == GameState::State::opponentPenaltyKick) goto start;
        if(theLibStriker.shouldKick(false, kickIntervalThreshold)) goto goToBallAndKick;
        if(std::get<1>(theLibPass.getBestPassage()) > shouldPassThreshold){
          passTarget = std::get<0>(theLibPass.getBestPassage());
          goto goToBallAndPass;
        }
      }

      action
      {
        ballPositionAtLastTargetChoice = MAGIC_VECTOR;
        theArmObstacleAvoidanceSkill();
        theGoToBallAndDribbleSkill(theLibMisc.calcAngleToTarget(theLibStriker.strikerDribblePoint()), false, 0.7);
      }
    }

    state(goToBallAndKick)
    {
      transition
      {
        if(theGameState.state == GameState::State::opponentPenaltyKick) goto start;
        if(!theLibStriker.shouldKick(true, kickIntervalThreshold)) goto goToBallAndDribble;
      }

      action
      {

        Vector2f ballRobotv = theFieldBall.positionOnFieldClipped - theRobotPose.translation;

        if (state_time < 200 || ballRobotv.norm() > 300 || (theFieldBall.positionOnField - ballPositionAtLastTargetChoice).norm() > 500) {
          chosenDoItAsap = theLibObstacles.nearestOpponent().translation.norm() < 750.f && false;

          Vector2f ballGoalv = theFieldBall.positionOnFieldClipped - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
          ballGoalv = ballGoalv.rotate(-pi_2);

          bool kickRight = ballRobotv.dot(ballGoalv) < 0;

          chosenKickType = theLibStriker.getKick(chosenDoItAsap,kickRight);

          chosenTarget = theLibStriker.goalTarget(chosenDoItAsap, true);

          ballPositionAtLastTargetChoice = theFieldBall.positionOnField;
        }

        Angle angle = theLibMisc.calcAngleToTarget(chosenTarget);

        theGoToBallAndKickSkill(angle, chosenKickType, !chosenDoItAsap);
      }
    }

    state(goToBallAndPass)
    {
      transition
      {
        if(theGameState.state == GameState::State::opponentPenaltyKick) goto start;
        if(theLibStriker.shouldKick(false, kickIntervalThreshold)) goto goToBallAndKick;
        if(theLibPass.getPassUtility(passTarget, 0) <= shouldPassThreshold) goto goToBallAndDribble;
      }

      action
      {
        ballPositionAtLastTargetChoice = MAGIC_VECTOR;
        float distance = theLibMisc.distance(passTarget, theRobotPose);
        KickInfo::KickType kickType = theLibPass.getKickType(passTarget);

        theGoToBallAndKickSkill(theLibMisc.calcAngleToTarget(passTarget), kickType, true, distance);
      }
    }
  }



};

MAKE_CARD(BasicStrikerCard);
