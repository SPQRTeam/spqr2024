/**
 * @file SetCard.cpp
 *
 * This file specifies the behavior for a robot in the Set game state.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"

CARD(ReadyCard,
{,
  CALLS(Activity),
  CALLS(LookActive),
  CALLS(LookAtPoint),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkPotentialField),
  CALLS(WalkToKickoffPose),
  CALLS(Say),
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(LibPosition),
});

class ReadyCard : public ReadyCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_READY;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_READY;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::ready);
    theWalkToKickoffPoseSkill(theLibPosition.myReadyPosition());
  }
};

MAKE_CARD(ReadyCard);
