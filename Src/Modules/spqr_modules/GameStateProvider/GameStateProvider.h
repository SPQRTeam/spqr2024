/**
* @file GameStateProvider.cpp
*   This file implements the new team coordination module updated in 2023
* @author @neverorfrog
*/

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/spqr_representations/GameState.h"
#include "Tools/Module/Module.h"

MODULE(GameStateProvider,
{,

REQUIRES(GameInfo),
REQUIRES(GameState), //just for enum
REQUIRES(OwnTeamInfo),
REQUIRES(OpponentTeamInfo),

PROVIDES(GameState),
});

class GameStateProvider : public GameStateProviderBase 
{
    public:
        void update(GameState& state);
};
