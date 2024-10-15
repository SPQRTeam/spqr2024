
#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

#include "Representations/Challenge/HumanCommand.h"


CARD(TestCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(Say),
  REQUIRES(HumanCommand),
});

class TestCard : public TestCardBase
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
    theActivitySkill(BehaviorStatus::Dummy);

    initial_state(start)
    {
      transition
      {

      }

      action
      {
        // OUTPUT_TEXT("Command: " << theHumanCommand.command);
        // OUTPUT_TEXT("Strategy: " << theHumanCommand.strategy);
        theLookForwardSkill();
        theStandSkill();
      }
    } 
  }
};

MAKE_CARD(TestCard);
