/**
 * @file LibLiberoProvider.cpp
 * 
 * See LibLibero
 *
 * @author Flavio Volpi
 */

#include "LibLiberoProvider.h"

MAKE_MODULE(LibLiberoProvider, behaviorControl);

LibLiberoProvider::LibLiberoProvider(){
    char dirGraphLiberoName[260];
    char dirGraphCentralName[260];
    sprintf(dirGraphLiberoName, "%s/Config/Graphs/positionGraphLibero.txt", File::getBHDir());
    sprintf(dirGraphCentralName, "%s/Config/Graphs/positionGraphCentral.txt", File::getBHDir());

    liberoPositionGraph.loadFromFile(dirGraphLiberoName);
    centralPositionGraph.loadFromFile(dirGraphCentralName);

    current_target = new Vector2f(0,0);
    *current_target = liberoPositionGraph.getClosestNode(theRobotPose.translation)->getPosition();
}

void LibLiberoProvider::update(LibLibero& libLibero)
{
    DECLARE_DEBUG_DRAWING3D("module:LibLiberoProvider:liberoPosition", "field");
    DECLARE_DEBUG_DRAWING("module:LibLiberoProvider:liberoPosition", "drawingOnField");
    DECLARE_DEBUG_DRAWING3D("module:UndirectedGraph", "field");

    libLibero.getLiberoPosition = [this]() -> Vector2f {
        return getLiberoPosition();
    };
}

Vector2f LibLiberoProvider::getLiberoPosition() {
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
        target = getLiberoPositionSpecial();
    }
    else{
        target = getLiberoPositionGraph();

    }
    
    return target.hasNaN() ? Vector2f(0.f, -1000.f): target;
}

Vector2f LibLiberoProvider::getLiberoPositionGraph() {

  Vector2f ball_position = theFieldBall.recentBallPositionOnField();
  bool is_libero_graph_valid = !liberoPositionGraph.checkPointInsideGraphRectangle(ball_position);
  bool is_central_graph_valid = !centralPositionGraph.checkPointInsideGraphRectangle(ball_position);
  
  spqr::UndirectedGraph current_graph = (!is_libero_graph_valid && is_central_graph_valid) ? centralPositionGraph : liberoPositionGraph;
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

Vector2f LibLiberoProvider::getLiberoPositionSpecial() {
    Vector2f target = Vector2f(0.f, -1000.f);
    Vector2f bestBall = theFieldBall.recentBallPositionOnField();

    // if(theGameState.state==GameState::State::ownCorner){
    //     std::tuple<float, float> angles = theLibSpec.calcAngleCorner();
    //     float angle_to_kick = std::get<0>(angles);
    //     float radius = 2500;
    //     target = theLibSpec.targetCornerPoint(angle_to_kick, radius);
    // }
    if(theGameState.state == GameState::State::opponentGoalKick){
        Vector2f ball = theFieldBall.recentBallPositionOnField();
        (ball.y() < 0) ? target = ball - Vector2f(1000.f, 0.f) : target = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosCenterGoal);
    }
    else{
        
        target = getLiberoPositionGraph();

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

