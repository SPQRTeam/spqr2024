/**
 * @file PlayerRole.h
 *
 * This file declares a representation of a player's role.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include <string>
#define magic_value -999999

STREAMABLE(PlayerRole, COMMA BHumanCompressedMessageParticle<PlayerRole>
{
  ENUM(RoleType,
  {,
    // THE PREEXISTING ONES
    // (TODO remove the removable ones at the end of the porting)
    none, // we have a "none" as well. Maybe redundant w/ undefined? Should we pick one? [torch]
    goalkeeper, 
    ballplayer,


    undefined,

    goalie,
    striker,
    defenderone,
    defendertwo,
    supporter,
    jolly,
    libero,

    penaltyStriker,
    penaltyKeeper,
    planStriker,
    planJolly,
    activeSearcher,
    passiveSearcher,
  });

  ENUM(Context,        tmp_role = PlayerRole::undefined;

  {,
    no_context,
    playing,
    search_for_ball,
  });

  std::string getName() const;

  static int getUtilityMatrixRowIndex(RoleType role);

  // /** BHumanMessageParticle functions (SPQR) */
  // void operator>>(BHumanMessage& m) const override;

  // // normally there would be handleArbitraryMessage here, but this case is special
  // // in that the Teammate struct will not have the full thePlayerRole field
  // // but just a role field of type RoleType. No need for the full representation.
  // // In the interest of putting sending and receiving close to each other, however,
  // // this function is put here as the closest possible thing to handleArbitraryMessage.
  // static RoleType specialHandlingOfArbitraryMessage(InMessage& m);

  /**
   * Compatibility function for (B-Human) 2019 behavior.
   * @return Whether the robot is goalkeeper.
   */
  static bool roleIsGoalkeeper(RoleType r) {
    return r == goalkeeper || r == goalie;
  }
  bool isGoalkeeper() const
  {
    return roleIsGoalkeeper(role);
  }

  /**
   * Compatibility function for (B-Human) 2019 behavior.
   * @return Whether the robot plays the ball.
   */
  static bool rolePlaysTheBall(RoleType r) {
    return r == striker;
  }
  bool playsTheBall() const
  {
    return rolePlaysTheBall(role);
  },

  (RoleType)(none) role, /**< The role type. */
  (RoleType)(undefined) lastRole,
  (Context)(no_context) current_context,
  (std::vector<Pose2f>)(std::vector<Pose2f>(7,Pose2f(magic_value,magic_value,magic_value))) robots_poses,
});
