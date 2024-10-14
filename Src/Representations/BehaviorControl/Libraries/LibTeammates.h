/**
 * @file LibTeammates.h
 *
 * A representation that contains information about teammate positions
 *
 * @author Lukas Malte Monnerjahn
 */

#pragma once
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibTeammates,
{
  /*
    =====================================================================================================================
    OUR STUFF
    =====================================================================================================================
  */

  /**
   * Returns the pose of the teamate nearest to this robot.
   */
  FUNCTION(Pose2f()) nearestTeammate,

  /*
    =====================================================================================================================
    THE PREEXISTING STUFF
    =====================================================================================================================
  */
  (int) teammatesInOpponentPenaltyArea,     /**< How many teammates are in the opponents penalty area */
  (int) nonKeeperTeammatesInOwnPenaltyArea, /**< How many non keeper teammates are in the own penalty area */
});