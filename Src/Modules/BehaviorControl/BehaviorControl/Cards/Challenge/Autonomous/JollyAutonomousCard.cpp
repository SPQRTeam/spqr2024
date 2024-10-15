#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"

#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibJolly.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"

#include "Representations/Challenge/HumanCommand.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"

#include <iostream>

CARD(JollyAutonomousCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(Say),
  CALLS(GoToBallAndDribble),
  CALLS(LookAtBall),
  CALLS(WalkToPoint),
  CALLS(GoToBallAndKick),
  CALLS(TurnToPoint),
  CALLS(LookAtAngles),
  CALLS(LookLeftAndRight),
  CALLS(WalkAtRelativeSpeed),

  REQUIRES(RobotPose),
  REQUIRES(ObstacleModel),
  REQUIRES(BallModel),
  REQUIRES(TeamPlayersModel),

  REQUIRES(LibStriker),
  REQUIRES(LibJolly),
  REQUIRES(LibMisc),
  REQUIRES(LibObstacles),
  
  REQUIRES(HumanCommand),
  REQUIRES(FieldBall),
  REQUIRES(TeamData),
  REQUIRES(FieldDimensions),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class JollyAutonomousCard : public JollyAutonomousCardBase
{
  bool preconditions() const override
  {
    if(theTeamData.teammates.size() > 0) {
      float autonomous_ball_distance = (theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      float controlled_ball_distance = (theTeamData.teammates[0].theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      return autonomous_ball_distance >= controlled_ball_distance;
    }
    return false;
  }

  bool postconditions() const override
  {
    if(theTeamData.teammates.size() > 0) {
      float autonomous_ball_distance = (theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      float controlled_ball_distance = (theTeamData.teammates[0].theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      return autonomous_ball_distance >= controlled_ball_distance;
    }
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Autonomous);

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
        Vector2f target = theLibMisc.glob2Rel(theLibJolly.getJollyPosition().x(), theLibJolly.getJollyPosition().y()).translation; 
        if(target.norm() < 250) goto turnAndWait;

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout)) goto searchForBall;
      }

      action
      {
        Vector2f target = theLibMisc.glob2Rel(theLibJolly.getJollyPosition().x(), theLibJolly.getJollyPosition().y()).translation;
        theLookAtBallSkill();
        theWalkToPointSkill(target);
      }
    }

    state(turnAndWait){

      transition
      {
        Vector2f target = theLibMisc.glob2Rel(theLibJolly.getJollyPosition().x(), theLibJolly.getJollyPosition().y()).translation; 
        if(target.norm() > 400) goto goToTarget;
      }

      action
      {
        theLookAtBallSkill();
        theTurnToPointSkill(theFieldBall.recentBallPositionRelative());
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen()) goto goToTarget;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }
};

MAKE_CARD(JollyAutonomousCard);
