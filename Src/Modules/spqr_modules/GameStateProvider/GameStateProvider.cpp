/**
* @file GameStateProvider.cpp
* @author Flavio Maiorana
*/

#include "GameStateProvider.h"

MAKE_MODULE(GameStateProvider, behaviorControl);

void GameStateProvider::update(GameState& state){
    bool isPlaying = theGameInfo.state == STATE_PLAYING;
    bool ownTeam = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber;
    bool opponentTeam = theGameInfo.kickingTeam == theOpponentTeamInfo.teamNumber;
    bool isKickIn = theGameInfo.setPlay == SET_PLAY_KICK_IN;
    bool isCorner = theGameInfo.setPlay == SET_PLAY_CORNER_KICK;
    bool isPenaltyKick = theGameInfo.setPlay == SET_PLAY_PENALTY_KICK;
    bool isPushingKick = theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK;
    bool isGoalKick = theGameInfo.setPlay == SET_PLAY_GOAL_KICK;
                                
    if(isPlaying)
    {
        if (ownTeam)
        {
            if(isKickIn) state.state = GameState::State::ownKickIn;
            else if(isCorner) state.state = GameState::State::ownCorner;
            else if(isPenaltyKick) state.state = GameState::State::ownPenaltyKick;
            else if(isPushingKick) state.state = GameState::State::ownPushingKick;
            else if(isGoalKick) state.state = GameState::State::ownGoalKick;
            else state.state = GameState::State::playing;
        } 
        else if(opponentTeam)
        {
            if(isKickIn) state.state = GameState::State::opponentKickIn;
            else if(isCorner) state.state = GameState::State::opponentCorner;
            else if(isPenaltyKick) state.state = GameState::State::opponentPenaltyKick;
            else if(isPushingKick) state.state = GameState::State::opponentPushingKick;
            else if(isGoalKick) state.state = GameState::State::opponentGoalKick;
            else state.state = GameState::State::playing;
        }
    }
    else
    {
        state.state = GameState::State::not_playing;
    }
}


