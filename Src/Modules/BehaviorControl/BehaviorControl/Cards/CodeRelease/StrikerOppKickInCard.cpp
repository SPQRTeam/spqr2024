/**
 * @file StrikerOppKickInCard.cpp
 *
 * This file implements a basic behavior for the StrikerOppKickIn.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
CARD(StrikerOppKickInCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkToPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(LookAtGlobalBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibMisc),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (float)(1000.f) distanceFromBall,
  }),
});

class StrikerOppKickInCard : public StrikerOppKickInCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_PLAYING && 
        (theGameInfo.setPlay != SET_PLAY_NONE && theGameInfo.setPlay != SET_PLAY_PENALTY_KICK) &&
        theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber;
    
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto followBall;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(followBall)
    {
      transition
      {
      }

      action
      {
        float defended_x = theFieldDimensions.xPosOwnGoal;   // default for most free kicks...
        if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK) {
          defended_x = theFieldDimensions.xPosOwnPenaltyMark;   // ...except the corner kick
        }
        Angle targetAngle = Angle(std::atan2(theFieldBall.recentBallPositionOnField().y(), 
            theFieldBall.recentBallPositionOnField().x()-defended_x));
        LocalPose2f relativize = theLibMisc.glob2Rel(theFieldBall.recentBallPositionOnField().x() - (distanceFromBall*std::cos(targetAngle)),
            theFieldBall.recentBallPositionOnField().y() - (distanceFromBall*std::sin(targetAngle)));
        Pose2f targetPos = Pose2f(Angle::fromDegrees(targetAngle),
                                relativize.translation.x(),
                                relativize.translation.y());

        theWalkToPointSkill(targetPos);
        theLookAtGlobalBallSkill();
      }
    }

  }  
};

MAKE_CARD(StrikerOppKickInCard);
