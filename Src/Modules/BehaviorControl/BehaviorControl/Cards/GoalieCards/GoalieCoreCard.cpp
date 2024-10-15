#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/Libraries/LibTeam.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/Challenge/HumanCommand.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Communication/TeamInfo.h"
#include <iostream>
#include "Tools/BehaviorControl/Interception.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/KickoffState.h"
using namespace std;

#define goalieLine 220

CARD(GoalieCoreCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(InterceptBall),
  CALLS(WalkAtRelativeSpeed),
  CALLS(KeyframeMotion),
  CALLS(WalkPotentialField),
  CALLS(LookAtBall),
  CALLS(WalkToPoint),
  CALLS(TurnToPoint),
  CALLS(WalkAtAbsoluteSpeed),
  CALLS(LookAtGlobalBall),
  CALLS(GoToBallAndKick),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(OwnTeamInfo),
  REQUIRES(LibTeam),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(GameInfo),
  REQUIRES(TeamData),
  REQUIRES(LibMisc),
  REQUIRES(LibCheck),
  REQUIRES(PlayerRole),
  REQUIRES(KickoffState),
  REQUIRES(RobotInfo),
  REQUIRES(HumanCommand),
  DEFINES_PARAMETERS(
  {,
    (int)(5000) ballSeenTimeout,
  }),
});

class GoalieCoreCard : public GoalieCoreCardBase
{

  bool preconditions() const override
  {
    // if kickoff is for opponent, we are in defending session
    int kickingteam = theGameInfo.kickingTeam;
    return theRobotInfo.number == RobotInfo::RoleNumber::autonomous && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber;
  }

  bool postconditions() const override
  {
    return !preconditions();
  }

  option
  {
    theActivitySkill(BehaviorStatus::Autonomous);
    initial_state(start)
    {
      transition
      {
        #ifdef PENALTY_STRIKER_GOALIE            
          goto goaliePose;
        #endif
          goto loop;
        if(shouldPass())
          goto pass;               
      }
      action
      {
        theStandSkill();
        Pose2f globalBall = theLibMisc.glob2Rel(theTeamBallModel.position.x(),theTeamBallModel.position.y());
        theLookAtPointSkill(Vector3f(globalBall.translation.x(), globalBall.translation.y(), 0));
        
      }
    }

    state(goaliePose)
    {
      transition
      {
        if( state_time>100)
          goto loop;
        if(shouldPass())
          goto pass; 
      }
      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(0.f, 1.f,0.f));
      }
    }

    state(loop)
    {  
      transition{
        Vector2f velocity = theBallModel.estimate.velocity;
        Vector2f position = theBallModel.estimate.position;

        #ifdef PENALTY_STRIKER_GOALIE  
        if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){ 
        #endif     

        if(theFieldBall.recentBallPositionOnField().x() > 750.f) goto waitAndTurn;

        double teta = atan(velocity.y()/velocity.x());
        float l = position.x()* (float) tan(teta);
        float side = position.y()-l;
        if( position.x() < 1200.f && velocity.x()<0 && velocity.norm()/std::max(position.norm(),1500.f) > 0.5){
          if(side<=200.f && side >= -200.f)
              goto stopBall;
          else if(side>200.f && side < 1500.f)
              goto goalieDiveLeft;
          else if(side<-200.f && side>-1500.f)
              goto goalieDiveRight;
        }
        #ifdef PENALTY_STRIKER_GOALIE  
        }
        #endif  
        if(shouldPass())
          goto pass;    
      }
        
      action{
        Vector2f globalBallPos = theFieldBall.recentBallPositionOnField();
        float clippedy = clip(globalBallPos.y(), theFieldDimensions.yPosRightGoal, theFieldDimensions.yPosLeftGoal);
        Vector2f globalTarget = Vector2f(theFieldDimensions.xPosOwnGroundLine+goalieLine, clippedy);

        if(theLibTeam.timeSinceBallWasSeen < 2000 || thePlayerRole.current_context != thePlayerRole.search_for_ball){
          if((std::abs(theRobotPose.translation.y() - globalTarget.y()) > 200.f) ||
             (std::abs(theRobotPose.translation.x() - globalTarget.x()) > 150.f)) {
            
            theWalkPotentialFieldSkill(globalTarget, -1, false, 0.5);

          }else{
          
            theStandSkill();

          }
         theLookAtBallSkill();
        }else{

          Pose2f localPose;
          localPose.translation = theLibMisc.glob2Rel(theFieldDimensions.xPosOwnGroundLine + goalieLine, 0.f).translation;
          localPose.rotation = (theRobotPose.inversePose * Vector2f::Zero()).angle();

          theWalkToPointSkill(localPose);
          theLookLeftAndRightSkill();
        }
      }
    }

    state(goalieDiveLeft)
    {
        action
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
          theKeyframeMotionSkill(KeyframeMotionRequest::keeperJumpLeft);
        }
    }

    state(goalieDiveRight)
    {
        action
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
          theKeyframeMotionSkill(KeyframeMotionRequest::keeperJumpLeft, true);
        }
    }

    state(stopBall)
    {
        action
        {
          theLookAtPointSkill(Vector3f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0));
          theKeyframeMotionSkill(KeyframeMotionRequest::genuflectStand);
        }
    }

    state(waitAndTurn)
    {
      transition
      {
        if(theFieldBall.recentBallPositionOnField().x() <= 750.f) goto loop;
      }

      action
      {
        Vector2f global_target = Vector2f(theFieldDimensions.xPosOwnGroundLine+250, theFieldDimensions.yPosCenterGoal);
        Vector2f local_target = theLibMisc.glob2Rel(global_target.x(), global_target.y()).translation;
        
        if(local_target.norm() > 200) theWalkToPointSkill(local_target);
        else theTurnToPointSkill(theFieldBall.recentBallPositionRelative());
        
        if(theFieldBall.timeSinceBallWasSeen < ballSeenTimeout) theLookAtBallSkill();
        else theLookAtGlobalBallSkill();
      } 
    }

state(pass)
    {
      transition
      {
        if(!shouldPass())
          goto loop;
      }

      action
      {
        Vector2f target = theTeamData.teammates[0].theRobotPose.translation;
        float distance = theLibMisc.distance(target, theRobotPose);
        theGoToBallAndKickSkill(theLibMisc.calcAngleToTarget(target), KickInfo::KickType::walkForwardsLeftLong, true, distance);
        theLookAtBallSkill();
      }
    }
  }

  bool shouldPass() const{
    if(theTeamData.teammates.size() > 0){
      bool result = theTeamData.teammates[0].theHumanCommand.commandBody == HumanCommand::CommandBody::AskForTheBall;
      return result;
    }
    return false;
  }
};

MAKE_CARD(GoalieCoreCard);
