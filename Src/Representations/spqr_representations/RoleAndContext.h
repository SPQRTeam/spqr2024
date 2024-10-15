/**
 * @file RoleAndContext.h
 */

#pragma once


#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"


STREAMABLE(RoleAndContext, COMMA public PureBHumanArbitraryMessageParticle<idRoleAndContext>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  float x,
  
  (int) myRole, //the role of the robot :
  /*///////////////////////////////////
                1 - goalie
                2 - striker
                3 - defender
                4 - supporter
                5 - jolly
                6 - activeSearcher
                7 - passiveSearcher
                0 - other (undefined)
*//////////////////////////////////////
  
  // Context which defines atk or def situations: 0 = Undefined, 1 = Beginning of attack, 2 = attack, 3 = beginning of defense, 4 = defense
  (int) atk_def_Context, 
  // Context which defines what team is holding the ball: 0 = None, 1 = Us, 2 = Them, 3 = Fight
  (int) ball_holding_Context, 
  //Context which defines the game score:  -2 = strongly losing, -1 = losing, 0 = draw, 1 = winning, 2 = strongly winning
  (int) winning_losing_Context, 
   //Context which defines the proportion between total robot number per team: -1 = we are less, 0 = equal, 1 = they are less
  (int) robots_number_Context,
  //Context which defines the proportion between robot involved in current action per team: -1 = we are less, 0 = equal, 1 = they are less
  (int) robot_action_number_Context,
  
});
