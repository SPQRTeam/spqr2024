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

CARD(GoalieAutonomousCard,
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

class GoalieAutonomousCard : public GoalieAutonomousCardBase
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

      }

      action
      {
        theStandSkill();
        theLookForwardSkill();
      }
    }
  }
};

MAKE_CARD(GoalieAutonomousCard);
