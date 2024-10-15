#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

#include "Representations/Challenge/HumanCommand.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"

#include <iostream>

CARD(BaseAutonomousCard,
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

  REQUIRES(HumanCommand),
  REQUIRES(RobotPose),
  REQUIRES(LibStriker),
  REQUIRES(LibMisc),
  REQUIRES(TeamData),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  DEFINES_PARAMETERS(
  {,
    // NOTHING
  }),
});

class BaseAutonomousCard : public BaseAutonomousCardBase
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
    theActivitySkill(BehaviorStatus::Autonomous);

    initial_state(start)
    {
      transition
      {
        if(shouldReceive())
          goto receivePassage;

        if(shouldPass())
          goto pass;

        if(shouldScore())
          goto score;
      }

      action
      {
        theStandSkill();
        theLookAtBallSkill();
      }
    }

    state(receivePassage)
    {
      transition
      {
        if(shouldStand())
          goto start;
        
        if(shouldPass())
          goto pass;

        if(shouldScore())
          goto score;
      }

      action
      {
        Vector2f controlledPose = theTeamData.teammates[0].theRobotPose.translation;
        Vector2f target = Vector2f(theTeamData.teammates[0].theHumanCommand.x,theTeamData.teammates[0].theHumanCommand.y);
        if(target.x() >= 4000){
          target = Vector2f(4000,target.y());
        }else if(target.x() <= -4000){
          target = Vector2f(-4000,target.y());
        }
        if(target.y() >= 2500){
          target = Vector2f(target.x(), 2500);
        }else if(target.y() <= -2500){
          target = Vector2f(target.x(), -2500);
        }
        theWalkToPointSkill(theLibMisc.glob2Rel(target.x(), target.y()));
        theLookAtBallSkill();
      }
    }

    state(pass)
    {
      transition
      {
        if(shouldStand())
          goto start;
        
        if(shouldReceive())
          goto receivePassage;

        if(shouldScore())
          goto score;
      }

      action
      {
        Vector2f target = theTeamData.teammates[0].theRobotPose.translation;
        float distance = theLibMisc.distance(target, theRobotPose);
        theGoToBallAndKickSkill(theLibMisc.calcAngleToTarget(target), KickInfo::KickType::walkForwardsLeftLong, true, distance);
        theLookAtBallSkill();
      }
    }

    state(score)
    {
      transition
      {
        if(shouldStand())
          goto start;
        
        if(shouldReceive())
          goto receivePassage;

        if(shouldPass())
          goto pass;
      }

      action
      {
        Vector2f ballPos = theFieldBall.positionOnFieldClipped;
        Vector2f target = Vector2f(4500,500);
        if(ballPos.x() > 1000){
          float distance = theLibMisc.distance(target, theRobotPose);
          theGoToBallAndKickSkill(theLibMisc.calcAngleToTarget(target),KickInfo::KickType::walkForwardsLeftLong, true, distance);
        } else {
          theGoToBallAndDribbleSkill(theLibMisc.calcAngleToTarget(target), false, 0.6);
        }
      }
    }
  }

    bool shouldStand() const {
      if(theTeamData.teammates.size() > 0){
        bool result = theTeamData.teammates[0].theHumanCommand.commandBody == HumanCommand::CommandBody::Stop;
        return result;
      }
      return false;
    }

    bool shouldReceive() const {
      if(theTeamData.teammates.size() > 0){
        bool result = theTeamData.teammates[0].theHumanCommand.commandBody == HumanCommand::CommandBody::Kick;
        return result;
      }
    return false;
    }

    bool shouldPass() const{
      if(theTeamData.teammates.size() > 0){
        bool result = theTeamData.teammates[0].theHumanCommand.commandBody == HumanCommand::CommandBody::AskForTheBall;
        return result;
      }
      return false;
    }

    bool shouldScore() const{
      if(theTeamData.teammates.size() > 0){
        return theTeamData.teammates[0].theHumanCommand.commandBody == HumanCommand::CommandBody::SearchTheBall;
      }
      return false;
    }


};

MAKE_CARD(BaseAutonomousCard);
