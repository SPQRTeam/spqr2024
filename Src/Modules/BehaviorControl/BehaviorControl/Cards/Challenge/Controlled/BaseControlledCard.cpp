#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

#include "Representations/Challenge/HumanCommand.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"


CARD(BaseControlledCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(Say),
  CALLS(GoToBallAndDribble),
  CALLS(LookAtBall),
  CALLS(WalkToPoint),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(GoToBallAndKick),
  CALLS(TurnToPoint),
  CALLS(LookAtAngles),
  CALLS(LookLeftAndRight),

  REQUIRES(HumanCommand),
  REQUIRES(RobotPose),
  REQUIRES(LibStriker),
  REQUIRES(LibMisc),
  REQUIRES(LibPass),
  REQUIRES(TeamData),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
});

class BaseControlledCard : public BaseControlledCardBase
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
    theActivitySkill(BehaviorStatus::Controlled);

    initial_state(start)
    {
      transition
      {
        switch (theHumanCommand.commandBody)
        { 
        case HumanCommand::CommandBody::GoToPosition:
          goto goToPosition;
          break;
        case HumanCommand::CommandBody::Dribble:
          goto dribble;
          break;
        case HumanCommand::CommandBody::GoToBallAndDribble:
          goto goToBallAndDribble;
          break;
        case HumanCommand::CommandBody::Kick:
          goto kick;
          break;
        case HumanCommand::CommandBody::Spazza:
          goto spazza;
          break;
        case HumanCommand::CommandBody::Pass:
          goto pass;
          break;
        case HumanCommand::CommandBody::AskForTheBall:
          goto askForTheBall;
          break;
        case HumanCommand::CommandBody::Turn:
          goto turn;
          break;
        case HumanCommand::CommandBody::SearchTheBall:
          goto turn;
          break;
        case HumanCommand::CommandBody::Stop:
          goto stop;
          break;
        default:
          break;
        }
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(goToPosition)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::GoToPosition)
          goto start;
      }

      action
      {
        Pose2f pose = theLibMisc.glob2Rel(theHumanCommand.x, theHumanCommand.y);
        theWalkToPointSkill(pose);
        switchHeadCommands();
      }
    }

    state(dribble)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::Dribble)
          goto start;
      }

      action
      {
        theGoToBallAndDribbleSkill(calcAngleToTarget(Vector2f(theHumanCommand.x, theHumanCommand.y)));
      }
    }

    state(goToBallAndDribble)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::GoToBallAndDribble)
          goto start;
      }

      action
      {
        theGoToBallAndDribbleSkill(calcAngleToTarget(theLibStriker.strikerMovementPoint()));
      }
    }

    state(kick)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::Kick)
          goto start;
      }

      action
      {
        Vector2f target = Vector2f(theHumanCommand.x, theHumanCommand.y);
        float distance = theLibMisc.distance(target, theRobotPose);
        if(theTeamData.teammates.size() > 0){
          distance = theLibMisc.distance(theTeamData.teammates[0].theRobotPose, theRobotPose);
        }
        theGoToBallAndKickSkill(calcAngleToTarget(target), KickInfo::walkForwardsLeftLong, true, distance);
      }
    }

    state(pass)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::Pass)
          goto start;
      }

      action
      {
        int num = theTeamData.numberOfActiveTeammates;
        int size = theTeamData.teammates.size();
        if(num > 0){
          theSaySkill("Pass the ball");
          OUTPUT_TEXT("I have " << num << " teammates");
          OUTPUT_TEXT(size);
          OUTPUT_TEXT("Passing to a teammate");
          Vector2f target = theTeamData.teammates[0].theRobotPose.translation;
          KickInfo::KickType kickType = theLibPass.getKickType(target);
          float distance = theLibMisc.distance(target, theRobotPose);
          theGoToBallAndKickSkill(calcAngleToTarget(target), kickType, true, distance);
        }
        else{
          theSaySkill("Go to ball and dribble");
          theGoToBallAndDribbleSkill(calcAngleToTarget(theLibStriker.strikerMovementPoint()));
        }
      }
    }

    state(askForTheBall)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::AskForTheBall)
          goto start;
      }

      action
      {
        Vector2f target = theFieldBall.recentBallPositionRelative();
        float angleToBall = atan2(target.y(), target.x());
        theWalkAtAbsoluteSpeedSkill(Pose2f(angleToBall, target.x()/70, target.y()/70));
        theLookAtBallSkill();
      }
    }


    state(spazza)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::Spazza)
          goto start;
      }

      action
      {
        Vector2f target = Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosCenterGoal);
        theGoToBallAndKickSkill(calcAngleToTarget(target), KickInfo::walkForwardsRightLong);
      }
    }

    state(turn)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::Turn)
          goto start;
      }

      action
      {
        theTurnToPointSkill(Vector2f(theHumanCommand.y, theHumanCommand.x));
        switchHeadCommands();   
      }
    }


    state(stop)
    {
      transition
      {
        if (theHumanCommand.commandBody != HumanCommand::CommandBody::Stop)
          goto start;
      }

      action
      {
        theStandSkill();
        switchHeadCommands();
      }
    }

  }

  Angle calcAngleToTarget(Vector2f target) const {
    return (theRobotPose.inversePose * target).angle();
  }

  void switchHeadCommands() const {
      switch(theHumanCommand.commandHead)
        {
          case HumanCommand::CommandHead::LookAtTheBall:
            theLookAtBallSkill();
            break;
          case HumanCommand::CommandHead::LookForward:
            theLookForwardSkill();
            break;
          case HumanCommand::CommandHead::LookLeft:
            theLookAtAnglesSkill(50_deg, 10_deg);
            break;
          case HumanCommand::CommandHead::LookRight:
            theLookAtAnglesSkill(-50_deg, 10_deg);
            break;
          case HumanCommand::CommandHead::Scan:
            theLookLeftAndRightSkill();
            break;
          default:
            theLookAtBallSkill();
            break;
        }
  }

};

MAKE_CARD(BaseControlledCard);
