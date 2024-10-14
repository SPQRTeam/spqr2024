/**
 * @file TeamPossessionProvider.cpp
 * @author Daniele Affinita
 */


#include "TeamPossessionProvider.h"
#include <cassert>

TeamPossessionProvider::TeamPossessionProvider(){
    decay = 1 - std::exp(-decaySpeed);
}


void TeamPossessionProvider::update(TeamPossession& teamPossession){
  GlobalVector2f ballV(theFieldBall.recentBallPositionOnField());
  GlobalVector2f localBallV(theFieldBall.recentBallPositionRelative());

  
  GlobalPose2f opponentP = theLibObstacles.nearestOpponentToPoint(ballV);
  GlobalVector2f opponentV(opponentP.translation);

  float opponentDist = (ballV - opponentV).norm();
  float myDist = localBallV.norm();

  teamPossession.possessionBalance = opponentDist - myDist;
  teamPossession.laggyPossessionBalance = lerp(prevLaggyBalance, teamPossession.possessionBalance, decay);

  prevLaggyBalance = teamPossession.laggyPossessionBalance;
}

MAKE_MODULE(TeamPossessionProvider, modeling);
