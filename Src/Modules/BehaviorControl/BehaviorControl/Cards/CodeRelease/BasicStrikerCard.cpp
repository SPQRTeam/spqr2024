/**
 * @file BasicStrikerCard.cpp
 *
 * This file implements a basic striker behavior for the striker.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/spqr_representations/GameState.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Debugging/Debugging.h"
#include <iostream>

CARD(BasicStrikerCard,
{,
  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookForward),
  CALLS(GoToBallAndKick),
  CALLS(GoToBallAndDribble),
  CALLS(ArmObstacleAvoidance),

  REQUIRES(LibStriker),
  REQUIRES(FieldBall),
  REQUIRES(LibObstacles),
  REQUIRES(LibMisc),
  REQUIRES(LibPass),
  REQUIRES(RobotPose),
  REQUIRES(OpponentGoalModel),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  DEFINES_PARAMETERS(
  {,
    (int)(100) initialWaitTime,
    (float)(80) kickIntervalThreshold,
    (float)(0.5) shouldPassThreshold,
  }),
});

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

  GlobalVector2f kickTarget;
  KickInfo::KickType kickType;
  GlobalVector2f ballPositionAtLastTargetChoice;
  bool doItASAP;
  GlobalVector2f passTarget;

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
        ballPositionAtLastTargetChoice = InvalidGlobalVector2f;
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
        ballPositionAtLastTargetChoice = InvalidGlobalVector2f;
        theArmObstacleAvoidanceSkill();
        theGoToBallAndDribbleSkill(theLibMisc.angleToTarget(theLibStriker.strikerDribblePoint), false, 0.7);
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
        Vector2f ballRobotVector = theFieldBall.positionOnFieldClipped - theRobotPose.translation;

        // Choose the target point for the kick
        if (state_time < 200 ||                                                              // If the goToBallAndKick state has just started
            ballRobotVector.norm() > 300 ||                                                  // If the ball is far from the robot
            (theFieldBall.positionOnField - ballPositionAtLastTargetChoice).norm() > 500) // If the ball has moved a lot since the last target choice
          {
          
          Vector2f ballGoalVector = theFieldBall.positionOnFieldClipped - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
          bool kickRight = ballRobotVector.dot(ballGoalVector.rotate(-pi_2)) < 0;
          
          doItASAP = theLibObstacles.nearestOpponent().translation.norm() < 750.f;
          kickType = theLibStriker.getKick(doItASAP, kickRight);
          kickTarget = theLibStriker.goalTarget(doItASAP, true);
          ballPositionAtLastTargetChoice = theFieldBall.positionOnField;
        }
    
        theGoToBallAndKickSkill(theLibMisc.angleToTarget(kickTarget), kickType, !doItASAP);
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
        ballPositionAtLastTargetChoice = InvalidGlobalVector2f;
        float distance = (theRobotPose.translation - passTarget).norm();
        KickInfo::KickType kickType = theLibStriker.getWalkKick(passTarget);

        theGoToBallAndKickSkill(theLibMisc.angleToTarget(passTarget), kickType, true, distance);
      }
    }
  }
};

MAKE_CARD(BasicStrikerCard);
