/**
 * @file LiberoCard.cpp
 *
 * This file implements a basic behavior for the Libero.
 *
 * @author Emanuele Antonioni
 */

#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibLibero.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"

#include "Tools/Math/BHMath.h"

CARD(LiberoCard,
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

  REQUIRES(GameInfo), 
  REQUIRES(FieldBall),
  REQUIRES(LibMisc),
  REQUIRES(LibLibero),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(5000) ballSeenTimeout,
  }),
});

class LiberoCard : public LiberoCardBase
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
    theActivitySkill(BehaviorStatus::Libero);

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
        Vector2f target = theLibMisc.glob2Rel(theLibLibero.getLiberoPosition().x(), theLibLibero.getLiberoPosition().y()).translation;
        if(target.norm() >= 200.f && target.norm() < 1200.f) goto goToTargetSideward;
        else if(target.norm() < 200.f) goto waitAndTurn;
      }

      action
      {
        Vector2f target = theLibMisc.glob2Rel(theLibLibero.getLiberoPosition().x(), theLibLibero.getLiberoPosition().y()).translation;
        theWalkToPointSkill(target);
        theArmObstacleAvoidanceSkill();
        theLookAtPointSkill(Vector3f(target.x(), target.y(), 100.f));
      }
    }

    state(goToTargetSideward)
    {
      transition
      {
        Vector2f target = theLibMisc.glob2Rel(theLibLibero.getLiberoPosition().x(), theLibLibero.getLiberoPosition().y()).translation;
        if(target.norm() >= 1000.f) goto goToTargetForward;
        else if(target.norm() < 200.f) goto waitAndTurn;   
      }

      action
      {
        Vector2f target = theLibMisc.glob2Rel(theLibLibero.getLiberoPosition().x(), theLibLibero.getLiberoPosition().y()).translation;
        Vector2f relative_ball = theFieldBall.recentBallPositionRelative();
        float angle_to_ball = atan2(relative_ball.y(), relative_ball.x());
        theWalkAtAbsoluteSpeedSkill(Pose2f(angle_to_ball, target.x(), target.y()));
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      }
    }

    state(waitAndTurn)
    {
      transition
      {
        Vector2f target = theLibMisc.glob2Rel(theLibLibero.getLiberoPosition().x(), theLibLibero.getLiberoPosition().y()).translation;
        if(target.norm() >= 350.f) goto goToTargetForward;
      }

      action
      {
        // theTurnToPointSkill(theFieldBall.recentBallPositionRelative());
        Angle angle = theLibMisc.getMirrorAngle(theFieldBall.recentBallPositionOnField(), theRobotPose.translation, Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosCenterGoal));
        theTurnAngleSkill(angle);
        
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      } 
    }
  }

};

MAKE_CARD(LiberoCard);
