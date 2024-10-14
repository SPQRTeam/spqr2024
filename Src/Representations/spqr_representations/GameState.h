/**
 * @file GameState.h
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @class GameState
 *
 */

STREAMABLE(GameState,
{
  ENUM(State,
  {,
    not_playing,
    playing,
    ownCorner,
    opponentCorner,
    ownKickIn,
    opponentKickIn,
    ownPenaltyKick,
    opponentPenaltyKick,
    ownPushingKick,
    opponentPushingKick,
    ownGoalKick,
    opponentGoalKick,
  }),

  (State) state,
});

