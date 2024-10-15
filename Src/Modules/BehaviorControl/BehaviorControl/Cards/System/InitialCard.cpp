/**
 * @file InitialCard.cpp
 *
 * This file specifies the behavior for a robot in the Initial game state.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(InitialCard,
{,
  CALLS(Activity),
  CALLS(LookAtAngles),
  CALLS(Stand),
  REQUIRES(GameInfo),
  REQUIRES(RawGameInfo),
});

class InitialCard : public InitialCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_INITIAL;
  }

  bool postconditions() const override
  {
    bool exit = theGameInfo.state != STATE_INITIAL;
    return exit;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::initial);
    theLookAtAnglesSkill(0.f, 0.f, 150_deg);
    theStandSkill(/* high: */ true);
  }
};

MAKE_CARD(InitialCard);
