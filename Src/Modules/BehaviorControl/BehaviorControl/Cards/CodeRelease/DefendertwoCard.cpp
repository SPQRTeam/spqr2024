/**
 * @file DefendertwoCard.cpp
 * This file implements a basic behavior for the Defenderone.
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

CARD(DefendertwoCard,
{,
  CALLS(Activity),
  CALLS(Stand),
  CALLS(TurnToPoint),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(LookForward),
  CALLS(LookAtBall),
  CALLS(LookAtGlobalBall),
  CALLS(ArmObstacleAvoidance),
  REQUIRES(FieldBall),
  REQUIRES(LibMisc),
  REQUIRES(LibDefender),
  DEFINES_PARAMETERS(
  {,
    (float)(0.7f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(5000) ballSeenTimeout,
  }),
});

class DefendertwoCard : public DefendertwoCardBase
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
    theActivitySkill(BehaviorStatus::Defender);

    initial_state(start)
    {
      transition
      {
        if (state_time > initialWaitTime) goto goToTarget;
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
        Vector2f target = theLibMisc.glob2Rel(theLibDefender.getDefendertwoPosition().x(), theLibDefender.getDefendertwoPosition().y()).translation;
        if(target.norm() < 200) goto turnAndWait;
      }
      
      action
      {
        Vector2f target = theLibMisc.glob2Rel(theLibDefender.getDefendertwoPosition().x(), theLibDefender.getDefendertwoPosition().y()).translation;
        Vector2f relative_ball = theFieldBall.recentBallPositionRelative();
        float angle_to_ball = atan2(relative_ball.y(), relative_ball.x());
        theWalkAtAbsoluteSpeedSkill(Pose2f(angle_to_ball, target.x(), target.y()));
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      }
    }

    state(turnAndWait)
    {
      transition
      {
        Vector2f target = theLibMisc.glob2Rel(theLibDefender.getDefendertwoPosition().x(), theLibDefender.getDefendertwoPosition().y()).translation;
        if(target.norm() > 350) goto goToTarget;
      }
      
      action
      {
        theTurnToPointSkill(theFieldBall.recentBallPositionRelative());
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      }
    }
  }
};

MAKE_CARD(DefendertwoCard);