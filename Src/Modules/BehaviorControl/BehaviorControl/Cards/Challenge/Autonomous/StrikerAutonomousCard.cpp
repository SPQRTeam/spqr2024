#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"

#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"

#include "Representations/Challenge/HumanCommand.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"

#include <iostream>

CARD(StrikerAutonomousCard,
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

  REQUIRES(RobotPose),
  REQUIRES(ObstacleModel),
  REQUIRES(BallModel),
  REQUIRES(TeamPlayersModel),

  REQUIRES(LibStriker),
  REQUIRES(LibMisc),
  REQUIRES(LibPass),
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
    (float)(80) kickIntervalThreshold,
  }),
});

Vector2f chosenTarget;
KickInfo::KickType chosenKickType;
bool chosenDoItAsap;
Vector2f ballPositionAtLastTargetChoice;

class StrikerAutonomousCard : public StrikerAutonomousCardBase
{
  bool preconditions() const override
  {
    if(theTeamData.teammates.size() > 0) {
      float autonomous_ball_distance = (theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      float controlled_ball_distance = (theTeamData.teammates[0].theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      return autonomous_ball_distance < controlled_ball_distance;
    }
    return true;
  }

  bool postconditions() const override
  {
    if(theTeamData.teammates.size() > 0) {
      float autonomous_ball_distance = (theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      float controlled_ball_distance = (theTeamData.teammates[0].theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();
      return autonomous_ball_distance > controlled_ball_distance;
    }
    return false;
  }

  Vector2f chosenTarget;
  KickInfo::KickType chosenKickType;
  bool chosenDoItAsap;
  Vector2f ballPositionAtLastTargetChoice;
  Vector2f MAGIC_VECTOR = Vector2f(-9999, -9999);

  option
  {
    theActivitySkill(BehaviorStatus::Autonomous);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto goToBallAndDribble;
          
        if(shouldPass())
          goto pass;
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
        if(shouldKick(false)){
          goto goToBallAndKick;
        }

        if(shouldPass())
          goto pass;

        if(!theFieldBall.ballWasSeen(2000))
          goto searchForBall;
      }

      action
      {
        ballPositionAtLastTargetChoice = MAGIC_VECTOR;
        theGoToBallAndDribbleSkill(calcAngleToTarget(theLibStriker.strikerMovementPoint()));
        
      }
    }

    state(goToBallAndKick)
    {
      transition
      {
        if(!shouldKick(true)){
          goto goToBallAndDribble;
        }

        if(shouldPass())
          goto pass;
      }

      action
      {

        Vector2f ballRobotv = theFieldBall.positionOnFieldClipped - theRobotPose.translation;

        if (state_time < 200 || ballRobotv.norm() > 300 || (theFieldBall.positionOnField - ballPositionAtLastTargetChoice).norm() > 500) {
          chosenDoItAsap = theLibObstacles.nearestOpponent().translation.norm() < 750.f;

          Vector2f ballGoalv = theFieldBall.positionOnFieldClipped - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
          ballGoalv = ballGoalv.rotate(-pi_2);

          bool kickRight = ballRobotv.dot(ballGoalv) < 0;

          chosenKickType = theLibStriker.getKick(chosenDoItAsap,kickRight);

          chosenTarget = theLibStriker.goalTarget(chosenDoItAsap, true);

          ballPositionAtLastTargetChoice = theFieldBall.positionOnField;
        }

        Angle angle = calcAngleToTarget(chosenTarget);

        theGoToBallAndKickSkill(angle, chosenKickType, !chosenDoItAsap);
      }
    }

    state(pass)
    {
      transition
      {
        if(!shouldPass())
          goto goToBallAndDribble;
      }

      action
      {
        OUTPUT_TEXT("STANNO CHIEDENDO LA PALLA");
        Vector2f target = theTeamData.teammates[0].theRobotPose.translation;
        float distance = theLibMisc.distance(target, theRobotPose);

        Vector2f ballGoalv = theFieldBall.positionOnFieldClipped - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
        Vector2f ballRobotv = theFieldBall.positionOnFieldClipped - theRobotPose.translation;

        ballGoalv = ballGoalv.rotate(-pi_2);

        bool kickRight = ballRobotv.dot(ballGoalv) < 0;

        theGoToBallAndKickSkill(calcAngleToTarget(target), kickRight ? KickInfo::KickType::walkForwardsRight : KickInfo::KickType::walkForwardsLeft, true, distance);
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto goToBallAndDribble;
      }

      action
      {
        theLookForwardSkill();
        theWalkToPointSkill(theFieldBall.recentBallEndPositionRelative(3000));
      }
    }
  };

  Angle calcAngleToTarget(Vector2f target) const
  {
    return (theRobotPose.inversePose * target).angle();
  }

  bool shouldPass() const{
    if(theTeamData.teammates.size() > 0){
      return theTeamData.teammates[0].theHumanCommand.commandBody != HumanCommand::CommandBody::AskForTheBall;
    }
    return false;
  }


  bool shouldKick(bool currentlyKicking) {
    // currentlyKicking: true if the robot is in GoToBallAndKick state, false if in GoToBallAndDribble state
    // is used to avoid stop kicking when the robot is already kicking
    
    int numberOfOpponentsInFrontOfMe = 0;
    for(auto obs:theObstacleModel.obstacles){
      if(obs.type == Obstacle::opponent){
        if(obs.center.x() > -150 && obs.center.norm() < 2000)
          numberOfOpponentsInFrontOfMe++;
      }
    }

    int numberOfOpponentsInOurHalf = 0;
    for(auto obs:theTeamPlayersModel.obstacles){
      if(obs.type == Obstacle::opponent){
        if(obs.center.x() < 0)
          numberOfOpponentsInOurHalf++;
      }
    }

    // interval: length of the free area (from FreeGoalTargetableArea.h)
    float interval = theLibStriker.goalTargetWithArea(false, false).second.interval;

    // always kick if the robot is near to opponent goal and there are too many obstacles to strategize
    if (theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyArea - (currentlyKicking ? 1000.f : 0.f) &&
        numberOfOpponentsInFrontOfMe > (currentlyKicking ? 2 : 3))
      return true;

    // kick if the robot has a good angle to the goal and the ball is seen enough in the last 60 frames (seenPercentage), 
    // and the robot is not too far from the goal (max 4 meters)
    else if (interval                     > (currentlyKicking ? -1 : kickIntervalThreshold) &&
             theRobotPose.translation.x() > (currentlyKicking ? 0.f : 500.f) &&
             theBallModel.seenPercentage  > (currentlyKicking ? 0 : 40)) 
      return true; 

    // kick if the robot is near to our goal and there are too many opponents in our half (spazzata dalla difesa)
    else if (theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundLine / 2 + (currentlyKicking ? 400.f : 0.f) &&
             numberOfOpponentsInOurHalf   > 1)
      return true;

    else
      return false;
  }

};

MAKE_CARD(StrikerAutonomousCard);
