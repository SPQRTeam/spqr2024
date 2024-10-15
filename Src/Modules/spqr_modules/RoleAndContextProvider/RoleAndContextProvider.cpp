// BROKEN 2023 
#include "RoleAndContextProvider.h"

#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCheck.h"


RoleAndContextProvider::RoleAndContextProvider(){
    SPQR::ConfigurationParameters();
}


std::tuple<float,int> RoleAndContextProvider::minOpponent_BallDistance(){
    float minDist = 90000;
    float dist = 0;
    int actionOpponents = 0; //number of opponents that are artecipating to the action
    for(auto obstacle : theTeamPlayersModel.obstacles){
        if(obstacle.type == Obstacle::opponent){
            dist = theLibMisc.distance( Pose2f(obstacle.center), Pose2f(theTeamBallModel.position) );
            if(dist < minDist ){
                minDist = dist;
            }
            if(dist < 1500.f){
                actionOpponents ++;
            }
        }    
    }
    return std::make_tuple(minDist, actionOpponents);
}

std::tuple<int,int,int,int> RoleAndContextProvider::score_andActivePlayers(){

    int ourPlayers = 0;
    int theirPlayers = 0;
    int ourScore = theOwnTeamInfo.score;
    int theirScore = theOpponentTeamInfo.score;

    for(auto player : theOwnTeamInfo.players){
        if(player.penalty == 0){
            ourPlayers ++;
        }//if penalty
    
    }// for players
    
    for(auto player : theOpponentTeamInfo.players){
        if(player.penalty == 0){
            theirPlayers ++;
        } // if penalty
    } // for player

    return std::make_tuple(ourScore,theirScore,ourPlayers,theirPlayers);
}

std::tuple<float,int, int> RoleAndContextProvider::minTeammate_BallDistance(){
    float minDist = 90000;
    float dist = 0;
    int robNum = 0;
    int actionTeammates = 0; //number of teammates that are artecipating to the action
    for(auto teammate : theTeamData.teammates){
        dist = theLibMisc.distance( teammate.theRobotPose, Pose2f(theTeamBallModel.position) );
        if(dist < minDist ){
            minDist = dist;
            robNum = teammate.number;
        }
        if(dist < 1500.f){
            actionTeammates ++;
        }
    }
    return std::make_tuple(minDist, robNum, actionTeammates);
}
void RoleAndContextProvider::update(RoleAndContext& rac) {
   
    //////////////////// ROLE ///////////////////////////////////////
	switch(thePlayerRole.role){
        case PlayerRole::goalie:rac.myRole = 1; break;
        case PlayerRole::striker:rac.myRole = 2; break;
        case PlayerRole::defenderone:rac.myRole = 3; break;
        case PlayerRole::supporter:rac.myRole = 4; break;
        case PlayerRole::jolly:rac.myRole = 5; break;
        case PlayerRole::activeSearcher:rac.myRole = 6; break;
        case PlayerRole::passiveSearcher:rac.myRole = 7; break;
        default: rac.myRole = 0; break;
    }
    
    ///////////////// BALL HOLDING CONTEXT + ROBOT_ACTION_NUMBER ////////////////////////
    float minOppBDist;
    float minTmBDist; 
    float myDist = theBallModel.estimate.position.norm();
    int closestTeammateToBall;
    int actionTeammates;
    int actionOpponents;
    std::tie(minOppBDist,actionOpponents) = minOpponent_BallDistance();
    std::tie(minTmBDist, closestTeammateToBall, actionTeammates) = minTeammate_BallDistance();
    
    if(myDist < 1500.f){
        actionTeammates ++;
    }

    if(minOppBDist < 500.f){
    
        if(myDist < 500.f || minTmBDist < 500.f){
    
            rac.ball_holding_Context = 3; //Fight
    
        }else{
    
            rac.ball_holding_Context = 2; //Them
    
        }
    
    }else{
    
        if(myDist < 500.f || minTmBDist < 500.f){
    
            rac.ball_holding_Context = 1; //Us
    
        }else{
    
            rac.ball_holding_Context = 0; //None
    
        }
    
    }
    
    if(actionTeammates > actionOpponents){
    
        rac.robot_action_number_Context = 1;
    
    }else if(actionTeammates < actionOpponents){
    
        rac.robot_action_number_Context = -1;
    
    }else{
    
        rac.robot_action_number_Context = 0;
    
    }

    ////////////////// ATK - DEF CONTEXT /////////////////////////////////////////////////
    if( rac.ball_holding_Context == 2 || rac.ball_holding_Context == 3 ){
    
        if(theTeamBallModel.position.x() > 0){
    
            rac.atk_def_Context = 3; // beginning of defense
    
        }else{
    
            rac.atk_def_Context = 4; // defense
    
        }

    }else if( rac.ball_holding_Context == 1 ){
    
        if(theTeamBallModel.position.x() > 0){
    
            rac.atk_def_Context = 2; // attack
    
        }else{
    
            rac.atk_def_Context = 1; // beginning of attack
    
        }
    }else{
    
        rac.atk_def_Context = 0; //undefined
    
    }

    //////////////// WINNING - LOSING + ROBOT NUMBER ///////////////////////////////////
    int ourScore;
    int theirScore;
    int ourPlayers;
    int theirPlayers;
    std::tie(ourScore,theirScore,ourPlayers,theirPlayers) = score_andActivePlayers();

    if( ( ourScore - theirScore) < -2 ){
   
        rac.winning_losing_Context = -2; //strongly losing
   
    }else if ( ( ourScore - theirScore) < 0 ){
   
        rac.winning_losing_Context = -1; // losing
   
    }else if ( ( ourScore - theirScore) == 0 ){
   
        rac.winning_losing_Context = 0; //draw
   
    }else if ( ( ourScore - theirScore) > 0 ){

        rac.winning_losing_Context = 1; //winning
   
    }else if ( ( ourScore - theirScore) > 2 ){

        rac.winning_losing_Context = 2; //strongly winning
   
    }

    if( ourPlayers < theirPlayers ){

        rac.robots_number_Context = -1; //we are less    

    }else if ( ourPlayers == theirPlayers ){

        rac.robots_number_Context = 0; // equal

    }else if( ourPlayers > theirPlayers ){

        rac.robots_number_Context = 1; // we are more

    }

    
}

MAKE_MODULE(RoleAndContextProvider, modeling);
