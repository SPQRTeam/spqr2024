/**
 * @file GameplayCard.cpp
 *
 * This file implements a card that represents the behavior of a robot in states in which it is not forced to do nothing.
 *
 * @author Arne Hasselbring
 */

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/PlayerRole.h"

CARD(GameplayCard,
{,
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(PlayerRole),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) controlled,
    (DeckOfCards<CardRegistry>) autonomous,
  }),
});

class GameplayCard : public GameplayCardBase
{
  bool preconditions() const override
  {
    return  theGameInfo.state == STATE_PLAYING;
  }

  bool postconditions() const override
  {
    return  theGameInfo.state != STATE_PLAYING;
  }

  void execute() override
  {
    if(theRobotInfo.number==RobotInfo::RoleNumber::autonomous){
      dealer.deal(autonomous)->call();
      setState("autonomous");
    }
    if(theRobotInfo.number==RobotInfo::RoleNumber::controlled){
      dealer.deal(controlled)->call();
      setState("controlled");  
    }      
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(GameplayCard);