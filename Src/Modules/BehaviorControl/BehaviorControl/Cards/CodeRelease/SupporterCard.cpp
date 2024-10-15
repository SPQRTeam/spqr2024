/**
 * @file SupporterCard.cpp
 *
 * This file implements a basic behavior for the supporter.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSupporter.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/Math/BHMath.h"

CARD(SupporterCard,
{,
  CALLS(Activity),
  CALLS(Stand),
  CALLS(WalkToPoint),
  CALLS(TurnToPoint),
  CALLS(LookForward),
  CALLS(LookAtGlobalBall),
  CALLS(ArmObstacleAvoidance),
  CALLS(LookAtBall),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(LibMisc),
  REQUIRES(LibSupporter),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(5000) ballSeenTimeout,
  }),
});

class SupporterCard : public SupporterCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Supporter);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto goToTarget;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(goToTarget)
    {
      transition
      {
        Vector2f target = theLibMisc.glob2Rel(theLibSupporter.getSupporterPosition().x(), theLibSupporter.getSupporterPosition().y()).translation;
        if(target.norm() < 200.f) goto turnAndWait;
      }

      action
      {
        Vector2f target = theLibMisc.glob2Rel(theLibSupporter.getSupporterPosition().x(), theLibSupporter.getSupporterPosition().y()).translation;
        theWalkToPointSkill(target);
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      }
    }

    state(turnAndWait)
    {
      transition
      {
        Vector2f target = theLibMisc.glob2Rel(theLibSupporter.getSupporterPosition().x(), theLibSupporter.getSupporterPosition().y()).translation;
        if(target.norm() > 350.f) goto goToTarget;
      }

      action
      {
        theTurnToPointSkill(theFieldBall.recentBallPositionRelative());
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
        theArmObstacleAvoidanceSkill();

      }
    }

  }
};

MAKE_CARD(SupporterCard);
