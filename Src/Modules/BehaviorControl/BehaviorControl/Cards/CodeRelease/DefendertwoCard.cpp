/**
 * @file DefendertwoCard.cpp
 * 
 * This file implements a basic behavior for the Defendertwo.
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
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
  CALLS(LookForScan),
  CALLS(ArmObstacleAvoidance),
  CALLS(GoToBallAndKick),

  REQUIRES(LibMisc),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(LibDefender),

  DEFINES_PARAMETERS(
  {,
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
        LocalVector2f target = theLibMisc.glob2Rel(theLibDefender.defendertwoPosition.x(), theLibDefender.defendertwoPosition.y()).translation;
        if(target.norm() < 200) goto turnAndWait;
        
        LocalVector2f ball_position = theFieldBall.recentBallPositionRelative();
        if(ball_position.norm() <= 400) goto goToBallAndSpazz;
      }
      
      action
      {
        LocalVector2f target = theLibMisc.glob2Rel(theLibDefender.defendertwoPosition.x(), theLibDefender.defendertwoPosition.y()).translation;
        LocalVector2f relative_ball = theFieldBall.recentBallPositionRelative();
        RadAngle angle_to_ball = atan2(relative_ball.y(), relative_ball.x());
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
        LocalVector2f target = theLibMisc.glob2Rel(theLibDefender.defendertwoPosition.x(), theLibDefender.defendertwoPosition.y()).translation;
        if(target.norm() > 350) goto goToTarget;

        LocalVector2f ball_position = theFieldBall.recentBallPositionRelative();
        if(ball_position.norm() <= 400) goto goToBallAndSpazz;
      }
      
      action
      {
        theTurnToPointSkill(theFieldBall.recentBallPositionRelative());
        theArmObstacleAvoidanceSkill();
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookForScanSkill(180_deg);
      }
    }

    state(goToBallAndSpazz)
    {
      transition
      {
        LocalVector2f ball_position = theFieldBall.recentBallPositionRelative();
        if(ball_position.norm() > 400) goto goToTarget;
      }
      
      action
      {        
        DegAngle robot_rotation = theLibMisc.radiansToDegree(theRobotPose.rotation);
        GlobalVector2f target = GlobalVector2f(4500, theRobotPose.translation.y());
        if(robot_rotation > -60_deg && robot_rotation < 60_deg) target = theLibMisc.rel2Glob(2000.f, 0.f).translation;
        theGoToBallAndKickSkill(theLibMisc.angleToTarget(target), theFieldBall.recentBallPositionRelative().y() > 50 ? KickInfo::KickType::walkForwardsLeftLong : KickInfo::KickType::walkForwardsRightLong, true, 10.f);
      }
    }   
  }
};

MAKE_CARD(DefendertwoCard);