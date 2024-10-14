/**
 * @file OwnKickoffCard.cpp
 *
 * This file implements a basic behavior for own kickoff.
 *
 * @author Flavio Volpi
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/KickoffState.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"

#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Settings.h"
#include <Eigen/Geometry>


CARD(OwnKickoffCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkPotentialField),
  CALLS(LookAtBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  REQUIRES(KickoffState),
  REQUIRES(ExtendedGameInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class OwnKickoffCard : public OwnKickoffCardBase
{
  bool preconditions() const override
  {
    return theKickoffState.isKickoff && theGameInfo.kickingTeam == Global::getSettings().teamNumber;
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
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    
  }
};

MAKE_CARD(OwnKickoffCard);
