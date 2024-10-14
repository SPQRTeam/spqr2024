/**
 * @file JollyCard.cpp
 *
 * This file implements a basic behavior for the Jolly.
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibJolly.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

CARD(JollyCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkToPoint),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(TurnToPoint),
  CALLS(LookAtGlobalBall),
  CALLS(LookAtBall),
  CALLS(LookAtPoint),
  CALLS(ArmObstacleAvoidance),
  CALLS(TurnAngle),

  REQUIRES(LibJolly),
  REQUIRES(LibMisc),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),

  DEFINES_PARAMETERS(
  {,
    (int)(100) initialWaitTime,
    (int)(5000) ballSeenTimeout,
  }),
});

class JollyCard : public JollyCardBase
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
    theActivitySkill(BehaviorStatus::Jolly);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime) goto goToTargetForward;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(goToTargetForward)
    {
      transition
      {
        LocalVector2f target = theLibMisc.glob2Rel(theLibJolly.jollyPosition.x(), theLibJolly.jollyPosition.y()).translation;
        if(target.norm() >= 200.f && target.norm() < 1200.f) goto goToTargetSideward;
        else if(target.norm() < 200.f) goto waitAndTurn;
      }

      action
      {
        LocalVector2f target = theLibMisc.glob2Rel(theLibJolly.jollyPosition.x(), theLibJolly.jollyPosition.y()).translation;
        theWalkToPointSkill(target);
        theArmObstacleAvoidanceSkill();
        theLookAtPointSkill(Vector3f(target.x(), target.y(), 100.f));
      }
    }

    state(goToTargetSideward)
    {
      transition
      {
        LocalVector2f target = theLibMisc.glob2Rel(theLibJolly.jollyPosition.x(), theLibJolly.jollyPosition.y()).translation;
        if(target.norm() >= 1000.f) goto goToTargetForward;
        else if(target.norm() < 200.f) goto waitAndTurn;   
      }

      action
      {
        LocalVector2f target = theLibMisc.glob2Rel(theLibJolly.jollyPosition.x(), theLibJolly.jollyPosition.y()).translation;
        LocalVector2f relative_ball = theFieldBall.recentBallPositionRelative();
        float angle_to_ball = atan2(relative_ball.y(), relative_ball.x());
        theWalkAtAbsoluteSpeedSkill(LocalPose2f(angle_to_ball, target.x(), target.y()));
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      }
    }

    state(waitAndTurn)
    {
      transition
      {
        LocalVector2f target = theLibMisc.glob2Rel(theLibJolly.jollyPosition.x(), theLibJolly.jollyPosition.y()).translation;
        if(target.norm() >= 350.f) goto goToTargetForward;
      }

      action
      {
        RadAngle angle = theLibMisc.getMirrorAngle(theFieldBall.recentBallPositionOnField(), theRobotPose.translation, GlobalVector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosCenterGoal));
        theTurnAngleSkill(angle);
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      } 
    }

  }
};

MAKE_CARD(JollyCard);
