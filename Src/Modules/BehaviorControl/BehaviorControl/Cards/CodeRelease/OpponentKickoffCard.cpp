/**
 * @file OpponentKickoffCard.cpp
 *
 * This file implements a basic behavior for the Dummy.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/KickoffState.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Settings.h"
#include <Eigen/Geometry>

using Line2 = Eigen::Hyperplane<float,2>;

CARD(OpponentKickoffCard,
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
  REQUIRES(KickoffState),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class OpponentKickoffCard : public OpponentKickoffCardBase
{
  bool preconditions() const override
  {
    return !theKickoffState.allowedToEnterCenterCircle && theGameInfo.kickingTeam != Global::getSettings().teamNumber;
  }

  bool postconditions() const override
  {
    return !preconditions();
  }

  option
  {
    theActivitySkill(BehaviorStatus::Dummy);

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

MAKE_CARD(OpponentKickoffCard);
