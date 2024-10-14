/**
 * @file LibLiberoProvider.cpp
 * 
 * This file implements a module that computes the libero position.
 */

#include "LibLiberoProvider.h"

MAKE_MODULE(LibLiberoProvider, behaviorControl);

LibLiberoProvider::LibLiberoProvider(){
  // Load the graphs from file
  char dirGraphLiberoName[260];
  char dirGraphCentralName[260];
  sprintf(dirGraphLiberoName, "%s/Config/Graphs/positionGraphLibero.txt", File::getBHDir());
  sprintf(dirGraphCentralName, "%s/Config/Graphs/positionGraphCentral.txt", File::getBHDir());
  liberoPositionGraph.loadFromFile(dirGraphLiberoName);
  centralPositionGraph.loadFromFile(dirGraphCentralName);

  // Initialize the current target
  current_target = liberoPositionGraph.getClosestNode(theRobotPose.translation)->getPosition();
}

void LibLiberoProvider::update(LibLibero& libLibero)
{
  libLibero.liberoPosition = getLiberoPosition();

  DECLARE_DEBUG_DRAWING3D("module:LibLiberoProvider:liberoPosition", "field");
  if(thePlayerRole.role == PlayerRole::libero){
    CYLINDER3D("module:LibLiberoProvider:liberoPosition", libLibero.liberoPosition.x(), libLibero.liberoPosition.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 20.0f, ColorRGBA::white);
  }
}

GlobalVector2f LibLiberoProvider::getLiberoPosition() {
  GlobalVector2f target;
  
  // Free kick
  if(theGameState.state == GameState::State::ownCorner         ||
    theGameState.state == GameState::State::ownGoalKick        ||
    theGameState.state == GameState::State::ownKickIn          ||
    theGameState.state == GameState::State::ownPushingKick     ||
    theGameState.state == GameState::State::ownPenaltyKick     ||
    
    theGameState.state == GameState::State::opponentCorner     ||
    theGameState.state == GameState::State::opponentGoalKick   ||
    theGameState.state == GameState::State::opponentKickIn     ||
    theGameState.state == GameState::State::opponentPushingKick     
  ){
      target = getLiberoPositionSpecial();
  }

  // Normal play (use the graph)
  else{
    target = getLiberoPositionGraph();
  }
  
  return target.hasNaN() ? GlobalVector2f(0.f, -1000.f): target;
}

GlobalVector2f LibLiberoProvider::getLiberoPositionGraph() {
  // Choose the current graph depending on the ball position
  GlobalVector2f ball_position = theFieldBall.recentBallPositionOnField();
  bool is_libero_graph_valid = !liberoPositionGraph.checkPointInsideGraphRectangle(ball_position);
  bool is_central_graph_valid = !centralPositionGraph.checkPointInsideGraphRectangle(ball_position);
  spqr::UndirectedGraph current_graph = (!is_libero_graph_valid && is_central_graph_valid) ? centralPositionGraph : liberoPositionGraph;

  // Get the closest node of the graph to the robot
  const spqr::Node *current_node = current_graph.getClosestNode(theRobotPose.translation);

  // Find the best node to go to using the passage utility
  const spqr::Node *best_node = current_node;
  float best_passage_utility = theLibPass.getInversePassUtility(current_node->getPosition());

  float current_hysteresis = hysteresis_utility;
  for (spqr::Node * neighbor : current_node->getNeighbors()){
    float neighbor_utility = theLibPass.getInversePassUtility(neighbor->getPosition());
    
    // Update the best node if the utility is higher than the current best node + hysteresis
    if (neighbor_utility > best_passage_utility + current_hysteresis){
      best_node = neighbor;
      best_passage_utility = neighbor_utility;
      current_hysteresis = 0;
    }
  }

  // Update the current target if the time has passed
  if(theFrameInfo.getTimeSince(last_time_modified_node) > hysteresis_time){
    current_target = best_node->getPosition();
    last_time_modified_node = theFrameInfo.time;
  }

  return current_target.hasNaN() ? GlobalVector2f(0.f, -1000.f): current_target;
}

GlobalVector2f LibLiberoProvider::getLiberoPositionSpecial() {
  GlobalVector2f target = GlobalVector2f(0.f, -1000.f);
  GlobalVector2f bestBall = theFieldBall.recentBallPositionOnField();

  // Opponent goal kick
  if(theGameState.state == GameState::State::opponentGoalKick){
    target = GlobalVector2f(750.0, -2200.0);
  }

  // Normal position (using the graph) with check if the position is too near to the ball (to avoid penalties)
  else{
    target = getLiberoPositionGraph();
    if((theFieldBall.recentBallPositionOnField() - target).norm() < 800.f){
      if(theFieldDimensions.isInsideField(target-GlobalVector2f(800.f,0.f)) &&
        (theFieldBall.recentBallPositionOnField() - (target-GlobalVector2f(800.f,0.f))).norm() < 800.f){
        target = target - GlobalVector2f(800.f, 0.f);
      }
      else if(theFieldDimensions.isInsideField(target-GlobalVector2f(0.f,800.f)) &&
        (theFieldBall.recentBallPositionOnField() - (target-GlobalVector2f(0.f,800.f))).norm() < 800.f){
        target = target - GlobalVector2f(0.f, 800.f);
      }
      else if(theFieldDimensions.isInsideField(target+GlobalVector2f(0.f,800.f)) &&
        (theFieldBall.recentBallPositionOnField() - (target+GlobalVector2f(0.f,800.f))).norm() < 800.f){
        target = target + GlobalVector2f(0.f, 800.f);
      }
      else{
        target = target + GlobalVector2f(800.f, 0.f); 
      }
    }
  }

  return target.hasNaN() ? GlobalVector2f(0.f, -1000.f): target;
}

