/**
 * @file LibJollyProvider.cpp
 * 
 * See LibJolly
 *
 * @author Francesco Petri
 */

#include "LibJollyProvider.h"

MAKE_MODULE(LibJollyProvider, behaviorControl);

LibJollyProvider::LibJollyProvider(){
  char dirGraphJollyName[260];
  char dirGraphCentralName[260];
  sprintf(dirGraphJollyName, "%s/Config/Graphs/positionGraphJolly.txt", File::getBHDir());
  sprintf(dirGraphCentralName, "%s/Config/Graphs/positionGraphCentral.txt", File::getBHDir());

  jollyPositionGraph.loadFromFile(dirGraphJollyName);
  centralPositionGraph.loadFromFile(dirGraphCentralName);

  current_target = new Vector2f(0,0);
  *current_target = jollyPositionGraph.getClosestNode(theRobotPose.translation)->getPosition();
}

void LibJollyProvider::update(LibJolly& libJolly)
{
  DECLARE_DEBUG_DRAWING3D("module:LibJollyProvider:jollyUtility", "field");
  DECLARE_DEBUG_DRAWING3D("module:UndirectedGraph", "field");
  libJolly.getJollyPosition = [this]() -> Vector2f {
    return getJollyPosition();
  };
  libJolly.getJollyPositionSpecial = [this]() -> Vector2f {
    return getJollyPositionSpecial();
  };
  libJolly.getJollyPositionGraph = [this]() -> Vector2f {
    return getJollyPositionGraph();
  };
}


Vector2f LibJollyProvider::getJollyPosition() {

  Vector2f target;
  
  // If it's a freeKick situation
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
    target = getJollyPositionSpecial();
  }
  else{
    target = LibJollyProvider::getJollyPositionGraph();
  }
  return target.hasNaN() ? Vector2f(0.f, 1000.f) : target;
}


Vector2f LibJollyProvider::getJollyPositionGraph() {

  Vector2f ball_position = theFieldBall.teamPositionOnField;
  bool is_jolly_graph_valid = !jollyPositionGraph.checkPointInsideGraphRectangle(ball_position);
  bool is_central_graph_valid = !centralPositionGraph.checkPointInsideGraphRectangle(ball_position);

  spqr::UndirectedGraph current_graph = (!is_jolly_graph_valid && is_central_graph_valid) ? centralPositionGraph : jollyPositionGraph;
  const spqr::Node *current_node = current_graph.getClosestNode(theRobotPose.translation);

  const spqr::Node *best_node = current_node;
  float best_passage_utility = theLibPass.getInversePassUtility(current_node->getPosition());

  float current_hysteresis = hysteresis_utility;
  for (spqr::Node * neighbor : current_node->getNeighbors()){

    float neighbor_utility = theLibPass.getInversePassUtility(neighbor->getPosition());
    
    if (neighbor_utility > best_passage_utility + current_hysteresis){
      best_node = neighbor;
      best_passage_utility = neighbor_utility;
      current_hysteresis = 0;
    }
  }

  if(theFrameInfo.getTimeSince(last_time_modified_node) > hysteresis_time){
    *current_target = best_node->getPosition();
    last_time_modified_node = theFrameInfo.time;
  }

  Vector2f target = *current_target;

  return target;
}

// per il ownCorner usa la funzione fatta ad hoc,
// per tutti gli altri ownFreeKicks usa default,
// per gli opponentFreeKicks fai funzione che controlla se la posizione è troppo vicina alla palla
Vector2f LibJollyProvider::getJollyPositionSpecial() {
  Vector2f target = Vector2f(1000.f, 0.f);
  Vector2f bestBall = theFieldBall.recentBallPositionOnField();

  if(theGameState.state==GameState::State::ownCorner){
    std::tuple<float, float> angles = theLibSpec.calcAngleCorner();
    float angle_to_kick = std::get<0>(angles);
    float radius = 2500;
    target = theLibSpec.targetCornerPoint(angle_to_kick, radius);
  }
  else if(theGameState.state == GameState::State::opponentGoalKick){
    Vector2f ball = theFieldBall.recentBallPositionOnField();
    (ball.y() > 0) ? target = ball - Vector2f(1000.f, 0.f) : target = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosCenterGoal);
  }
  else{
    target = getJollyPositionGraph();
    // Check if this position is too near to the ball during free kick.
    // In this case, move the point back/left/right/forward
    if((theFieldBall.recentBallPositionOnField() - target).norm() < 800.f){
      if(theFieldDimensions.isInsideField(target-Vector2f(800.f,0.f)) &&
        (theFieldBall.recentBallPositionOnField() - (target-Vector2f(800.f,0.f))).norm() < 800.f){
        target = target - Vector2f(800.f, 0.f);
      }
      else if(theFieldDimensions.isInsideField(target-Vector2f(0.f,800.f)) &&
        (theFieldBall.recentBallPositionOnField() - (target-Vector2f(0.f,800.f))).norm() < 800.f){
        target = target - Vector2f(0.f, 800.f);
      }
      else if(theFieldDimensions.isInsideField(target+Vector2f(0.f,800.f)) &&
        (theFieldBall.recentBallPositionOnField() - (target+Vector2f(0.f,800.f))).norm() < 800.f){
        target = target + Vector2f(0.f, 800.f);
      }
      else{
        target = target + Vector2f(800.f, 0.f); 
      }
    }
  }

  
  return target;
}

