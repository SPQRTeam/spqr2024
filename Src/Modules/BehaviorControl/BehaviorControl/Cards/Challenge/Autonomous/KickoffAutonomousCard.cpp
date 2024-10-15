#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/KickoffState.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Settings.h"
#include <Eigen/Geometry>

using Line2 = Eigen::Hyperplane<float,2>;

CARD(KickoffAutonomousCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkPotentialField),
  CALLS(LookAtGlobalBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(KickoffState),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class KickoffAutonomousCard : public KickoffAutonomousCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_PLAYING &&
           theGameInfo.secsRemaining > 594 &&
           !(theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_PENALTY_KICK || 
             theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN ||
             theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK);
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
        
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    
  }
};

MAKE_CARD(KickoffAutonomousCard);
