/**
 * @file LibDefenderProvider.cpp
 * 
 * This file implements a module that computes the defenders position.
 */

#include "LibDefenderProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <iostream>

MAKE_MODULE(LibDefenderProvider, behaviorControl);

void LibDefenderProvider::update(LibDefender& libDefender)
{
  libDefender.defenderonePosition = getDefenderonePosition();
  libDefender.defendertwoPosition = getDefendertwoPosition();

  DECLARE_DEBUG_DRAWING3D("module:LibDefenderProvider:defenderonePosition", "field");
  DECLARE_DEBUG_DRAWING3D("module:LibDefenderProvider:defendertwoPosition", "field");
  if(thePlayerRole.role == PlayerRole::defenderone){
    CYLINDER3D("module:LibDefenderProvider:defenderonePosition", libDefender.defenderonePosition.x(), libDefender.defenderonePosition.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 20.0f, ColorRGBA::blue);
  }
  if(thePlayerRole.role == PlayerRole::defendertwo){
    CYLINDER3D("module:LibDefenderProvider:defendertwoPosition", libDefender.defendertwoPosition.x(), libDefender.defendertwoPosition.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 20.0f, ColorRGBA::cyan);
  }   
}

GlobalVector2f LibDefenderProvider::getDefenderonePosition() const {
  GlobalVector2f target;

  // Free kick
  if(theGameState.state == GameState::State::ownCorner          ||
     theGameState.state == GameState::State::ownGoalKick        ||
     theGameState.state == GameState::State::ownKickIn          ||
     theGameState.state == GameState::State::ownPushingKick     ||
     theGameState.state == GameState::State::ownPenaltyKick     ||
     
     theGameState.state == GameState::State::opponentCorner     ||
     theGameState.state == GameState::State::opponentGoalKick   ||
     theGameState.state == GameState::State::opponentKickIn     ||
     theGameState.state == GameState::State::opponentPushingKick     
  ){
    target = getDefenderonePositionSpecial();
  }

  // Normal play
  else{
    const GlobalVector2f ballPos = theFieldBall.recentBallPositionOnField();
    
    if(ballPos.x() > 0.f) target = Vector2f(-3600.f, 550.f);
    else{
      const GlobalVector2f leftGoalPost{theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal - 100.f};
      const GlobalVector2f goalCenter{theFieldDimensions.xPosOwnGroundLine, 0.f};
      target = targetOnSemiCircle(ballPos, leftGoalPost, 800, 900);
    }

  }

  return (target.hasNaN() || !theFieldDimensions.isInsideField(target)) ? GlobalVector2f(-3000.f, -1000.f) : target;
}

GlobalVector2f LibDefenderProvider::getDefenderonePositionSpecial() const {
  GlobalVector2f target;

  switch(theGameState.state){
    case GameState::State::ownGoalKick:{
      target = GlobalVector2f(-3600.f, 550.f);
      break;
    }
    case GameState::State::opponentCorner:{
      // Left Corner
      if(theFieldBall.recentBallPositionOnField().y()>0) 
        target = GlobalVector2f(theFieldDimensions.xPosOwnGroundLine + 200, theFieldDimensions.yPosLeftPenaltyArea - 200);
      
      // Right Corner
      else 
        target = GlobalVector2f(theFieldDimensions.xPosOwnGroundLine + 200, theFieldDimensions.yPosRightPenaltyArea + 200);
      break;
    }
    default:{
      const GlobalVector2f leftGoalPost{theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal - 100.f};
      const GlobalVector2f goalCenter{theFieldDimensions.xPosOwnGroundLine, 0.f};
      const GlobalVector2f ballPos = theFieldBall.recentBallPositionOnField();
      target = targetOnSemiCircle(ballPos, leftGoalPost, 800, 900);
      break;
    }
  }
  return (target.hasNaN() || !theFieldDimensions.isInsideField(target)) ? GlobalVector2f(-3000.f, -1000.f) : target;
}

GlobalVector2f LibDefenderProvider::getDefendertwoPosition() const {
  GlobalVector2f target;
  
  // FreeKick
  if(theGameState.state == GameState::State::ownCorner          ||
     theGameState.state == GameState::State::ownGoalKick        ||
     theGameState.state == GameState::State::ownKickIn          ||
     theGameState.state == GameState::State::ownPushingKick     ||
     theGameState.state == GameState::State::ownPenaltyKick     ||
     
     theGameState.state == GameState::State::opponentCorner     ||
     theGameState.state == GameState::State::opponentGoalKick   ||
     theGameState.state == GameState::State::opponentKickIn     ||
     theGameState.state == GameState::State::opponentPushingKick     
  ){
    target = getDefendertwoPositionSpecial();
  }

  // Normal play
  else{
    const GlobalVector2f ballPos = theFieldBall.recentBallPositionOnField();
    
    if(ballPos.x() > 0.f){
      target = GlobalVector2f(-2200.f, -350.f);
    }
    else{
      const GlobalVector2f rightGoalPost{theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal + 100.f};
      const GlobalVector2f goalCenter{theFieldDimensions.xPosOwnGroundLine, 0.f};
      target = targetOnSemiCircle(ballPos, rightGoalPost, 1900, 2100);
    }
  }

  return (target.hasNaN() || !theFieldDimensions.isInsideField(target)) ? GlobalVector2f(-3000.f, 1000.f) : target;
}

GlobalVector2f LibDefenderProvider::getDefendertwoPositionSpecial() const {
  GlobalVector2f target;
  switch(theGameState.state){
    case GameState::State::ownGoalKick:{
      target = GlobalVector2f(-2200.f, -350.f);
      break;
    }
    case GameState::State::opponentCorner:{
      target = theLibSpec.targetDefenseCornerPoint();
      break;
    }
    default:{
      const GlobalVector2f rightGoalPost{theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal + 100.f};
      const GlobalVector2f goalCenter{theFieldDimensions.xPosOwnGroundLine, 0.f};
      const GlobalVector2f ballPos = theFieldBall.recentBallPositionOnField();
      target = targetOnSemiCircle(ballPos, rightGoalPost, 1900, 2100);
      break;
    }
  }

  return (target.hasNaN() || !theFieldDimensions.isInsideField(target)) ? GlobalVector2f(-3000.f, 1000.f) : target;
}

GlobalVector2f LibDefenderProvider::targetOnSemiCircle(GlobalVector2f segmentStart, GlobalVector2f segmentEnd, float minRadius, float maxRadius) const {  
  if(segmentStart.x() < -3500.f) segmentStart = GlobalVector2f(-3500.f, segmentStart.y());

  // Line from segment start (ball pos or an opponent) to segment end (for now right or left goal post)
  const Geometry::Line line(segmentStart, segmentEnd - segmentStart);

  // Radius computation
  const float startDistance = theFieldBall.distanceToOwnPenaltyArea;
  float alpha = startDistance/theFieldDimensions.xPosOpponentPenaltyArea;
  if (alpha > 1) alpha = 1;
  float radius{};
  if (startDistance == -1.f){
    radius = minRadius;
  } else{
    radius = minRadius + alpha*(maxRadius - minRadius);
  }

  // Circle with center in goal 
  GlobalVector2f center{theFieldDimensions.xPosOwnGroundLine + 200.f, 0.f};
  const Geometry::Circle circle(center, radius); 
  
  GlobalVector2f target{};
  //Data preparation
  const GlobalVector2f d = line.direction;
  const GlobalVector2f q = line.base - circle.center; 
  const float a = std::pow(d.x(),2) + std::pow(d.y(),2);
  const float b = 2*(d.x()*q.x() + d.y()*q.y());
  const float c = std::pow(q.x(),2) + std::pow(q.y(),2) - std::pow(radius,2);

  // Solving snd order equation
  const float delta = std::pow(b,2) - 4*a*c;
  if (delta >= 0) {
    const float first = (-b + std::sqrt(delta)) / (2*a);
    const float second =(-b - std::sqrt(delta)) / (2*a);

    const GlobalVector2f firstIntercept = line.base + first * d;
    const GlobalVector2f secondIntercept = line.base + second * d;  

    target = firstIntercept.x() > theFieldDimensions.xPosOwnGroundLine ? firstIntercept : secondIntercept;
  } else {
    target = GlobalVector2f(-3000.f, 0.f);
  }
  if(target.x() < -4000.f) target = GlobalVector2f(-4000.f, target.y());

  return target.hasNaN() ? GlobalVector2f(-3000.f, 0.f) : target;
}







