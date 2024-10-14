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
    (DeckOfCards<CardRegistry>) ownKickoff,
    (DeckOfCards<CardRegistry>) opponentKickoff,
    (DeckOfCards<CardRegistry>) ownFreeKick,
    (DeckOfCards<CardRegistry>) opponentFreeKick,
    (DeckOfCards<CardRegistry>) normalPlay,
    (DeckOfCards<CardRegistry>) ownPenaltyKick,
    (DeckOfCards<CardRegistry>) opponentPenaltyKick,
    (DeckOfCards<CardRegistry>) goalie,
    (DeckOfCards<CardRegistry>) striker,
    (DeckOfCards<CardRegistry>) defenderone,
    (DeckOfCards<CardRegistry>) defendertwo,
    (DeckOfCards<CardRegistry>) supporter,
    (DeckOfCards<CardRegistry>) jolly,
    (DeckOfCards<CardRegistry>) searcher,
    (DeckOfCards<CardRegistry>) libero,
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
   
  if(thePlayerRole.current_context == PlayerRole::search_for_ball){
    if(theRobotInfo.number == 1){
      dealer.deal(goalie)->call();
      setState("goalie");
    }else{
      dealer.deal(searcher)->call();
      setState("searcher");
    }
     
  }else{
    if(thePlayerRole.role == PlayerRole::striker){
      dealer.deal(striker)->call();
      setState("striker");
    }else if(thePlayerRole.role == PlayerRole::supporter){
      dealer.deal(supporter)->call();
      setState("supporter");
    }else if(thePlayerRole.role == PlayerRole::jolly){
      dealer.deal(jolly)->call();
      setState("jolly");
    }else if(thePlayerRole.role == PlayerRole::defenderone){
      dealer.deal(defenderone)->call();
      setState("defenderone");
    }else if(thePlayerRole.role == PlayerRole::defendertwo){
      dealer.deal(defendertwo)->call();
      setState("defendertwo");
    }else if(thePlayerRole.role == PlayerRole::libero){
      dealer.deal(libero)->call();
      setState("libero");
    }else{
      dealer.deal(goalie)->call();
      setState("goalie");
    }
  }
  
      
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(GameplayCard);