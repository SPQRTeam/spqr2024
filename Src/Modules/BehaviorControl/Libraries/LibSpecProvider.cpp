/**
 * @file LibSpecProvider.cpp
 * 
 * See LibSpec
 *
 */

#include "LibSpecProvider.h"
#include "Tools/Modeling/Obstacle.h"


MAKE_MODULE(LibSpecProvider, behaviorControl);

void LibSpecProvider::update(LibSpec& libSpec)
{
  DECLARE_DEBUG_DRAWING3D("module:LibSpecProvider:targetPosition", "field");
  DECLARE_DEBUG_DRAWING3D("module:LibSpecProvider:corridor", "field");
  DECLARE_DEBUG_DRAWING3D("module:LibSpecProvider:shortestpath", "field");
  DECLARE_DEBUG_DRAWING("module:LibSpecProvider:targetPosition", "drawingOnField");

  libSpec.compare_obstacles = [this](Vector2f obs1, Vector2f obs2) -> bool {
    return compare_obstacles(obs1, obs2);
  };
  libSpec.calcAngleCorner = [this]() -> std::tuple<float, float> {
    return calcAngleCorner();
  };
  libSpec.targetCornerPoint = [this](float angle, float radius) -> Vector2f {
    return targetCornerPoint(angle, radius);
  };
  libSpec.calcCornerZone = [this](const Vector2f& point) -> int {
    return calcCornerZone(point);
  };
  libSpec.targetDefenseCornerPoint = [this]() -> Vector2f {
    return targetDefenseCornerPoint();
  };
  libSpec.getPositionCoveringDangerousOpponent = [this](bool onBall) -> Vector2f {
    return getPositionCoveringDangerousOpponent(onBall);
  };
  libSpec.freeCorridor = [this](const Vector2f leftEnd, const Vector2f rightEnd, const float width) -> Vector2f {
    return freeCorridor(leftEnd, rightEnd, width);
  };
  libSpec.isObstacleInCorridor = [this](const Vector2f start, const Vector2f end, const float width, const Vector2f obstacle) -> bool {
    return isObstacleInCorridor(start, end, width, obstacle);
  };
  libSpec.nearestPointOnCorridor = [this](const Vector2f source, const Vector2f target) -> Vector2f {
    return nearestPointOnCorridor(source, target);
  };
  libSpec.getMostDangerousOpponent = [this]() -> Vector2f {
    return getMostDangerousOpponent();
  };
  libSpec.isGoaliePlaying = [this]() -> bool {
    return isGoaliePlaying();
  };
  libSpec.isTherePassKickoffCondition = [this](Vector2f& target) -> bool {
    return isTherePassKickoffCondition(target);
  };
}

bool LibSpecProvider::compare_obstacles(Vector2f obs1, Vector2f obs2) const{
  Vector2f start = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline); //start on left corner    
  Vector2f endBound1 = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea); //end on right bound goal area 

  float angle1 = theLibMisc.angleBetweenGlobalVectors(start, endBound1, obs1);
  float angle2 = theLibMisc.angleBetweenGlobalVectors(start, endBound1, obs2);

  return angle1 <= angle2;
}


std::tuple<float, float> LibSpecProvider::calcAngleCorner() const{
  Vector2f startBounds;
  Vector2f endBound1;
  Vector2f endBound2;
  
  (theTeamBallModel.position.y() > 0) ? ({   // left corner
    startBounds = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline); //start on left corner
    endBound1 = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea); //end on right bound goal area 
    endBound2 = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea); //end on left corner of penalty area
    
  }) : ({                                   // right corner
    startBounds = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline); //start on right corner
    endBound1 = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoalArea); //end on left bound goal area 
    endBound2 = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea); //end on right corner of penalty area
  });
  
  float radius =  3000;

  std::list<Vector2f> obstaclesDetected;

  for(auto obstacle: theObstacleModel.obstacles){
    Vector2f obstacle_center = theLibMisc.rel2Glob(obstacle.center.x(), obstacle.center.y()).translation;

    if(obstacle.type != Obstacle::teammate && theLibMisc.isInsideGlobalSector(startBounds, endBound1, endBound2, .0f, radius, obstacle_center)){
      obstaclesDetected.push_back(obstacle_center);
    }
  }
  obstaclesDetected.push_back(endBound2);

  obstaclesDetected.sort([this] (const Vector2f obs1, const Vector2f obs2) {
    return compare_obstacles(obs1, obs2); 
  });

  std::list<Vector2f>::iterator it=obstaclesDetected.begin();    
  ++it;

  std::list<float> angle_list;
  float best_angle = 0;
  int counter_best_angle = 0;
  int k = 0;

  for(it; it!=obstaclesDetected.end(); ++it){
    Vector2f obs1 = *prev(it);
    Vector2f obs2 = *(it);
    float angle = theLibMisc.angleBetweenGlobalVectors(startBounds, obs1, obs2);
    angle_list.push_back(angle);
    if(angle > best_angle){
      best_angle = angle;  
      counter_best_angle = k;
    }          
    ++k;
  }
  
  float angle_to_kick = 0;
  std::list<float>::iterator angle = angle_list.begin();
  for(int i=0; i<=counter_best_angle; i++){
    (i==counter_best_angle) ? (angle_to_kick += (*angle/2)) : ( angle_to_kick += (*angle));
    ++angle;
  }
  
  return std::make_tuple(angle_to_kick, best_angle);
}


Vector2f LibSpecProvider::targetCornerPoint(float angle, float radius) const{
  float x, y;
  x = theFieldDimensions.xPosOpponentGroundLine - radius*sin(angle);
  (theTeamBallModel.position[1] > 0) ? 
    (y = theFieldDimensions.yPosLeftSideline - radius*cos(angle)) : (y = theFieldDimensions.yPosRightSideline + radius*cos(angle)); 
  return Vector2f(x,y);
}

double LibSpecProvider::pointToSegmentDistance(Vector2f A, Vector2f B, Vector2f P) const{
    float angle = theLibMisc.angleBetweenGlobalVectors(A, P, B);
    float distance_AP = (P - A).norm();
    float distance_AB = (B - A).norm();
    if(cos(angle) < 0.f || distance_AP*cos(angle) > distance_AB)
      return -1;
    return distance_AP*sin(abs(angle));
}

bool orderObstaclesByXCoordinate(const Obstacle o1, const Obstacle o2) {
    return (o1.center.x() < o2.center.x());
}

Vector2f LibSpecProvider::getPositionCoveringDangerousOpponent(bool onBall) const {
      
    Vector2f end = theFieldBall.recentBallPositionOnField();
    Vector2f start = getMostDangerousOpponent();
    Vector2f target;
    if((end - start).norm() >= 1200.f)
      target = start + (end-start)/4.f;
    else{
      end = Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f);
      target = start + (end-start)/7.f;
    }
    return target; 
}

Vector2f LibSpecProvider::freeKickWall(Vector2f robot) const{
    return Vector2f(0.f,0.f);
}


Vector2f LibSpecProvider::getMostDangerousOpponent() const
{
    //Parameters for checking corridor coverage
    // bool corridorCovered = false;
    // Vector2f source = theFieldBall.recentBallPositionOnField();
    // Vector2f target = Vector2f(theFieldDimensions.xPosOwnGroundLine,0.f);
    // float width = theFieldDimensions.yPosLeftGoal - 100.f;

    // // generate and fill an array of obstacles in absolute coordinates
    // std::vector<Obstacle> orderedObstacles;
    // for(const auto& obstacle : theTeamPlayersModel.obstacles) 
    //     if(obstacle.type == Obstacle::opponent) 
    //         orderedObstacles.push_back(obstacle);
    // std::sort(orderedObstacles.begin(), orderedObstacles.end(), orderObstaclesByXCoordinate);

    // //Take the nearest opponent to own goal (not in the corridor to the goal)
    // int i = 0;
    // for (; i < orderedObstacles.size(); i++){
    //   Vector2f nearestOppToGoal = orderedObstacles[i].center;
    //   if(!theLibObstacles.isCorridorCovered(source,target,width,nearestOppToGoal))
    //     return nearestOppToGoal;
    // }

    // if (i > 0)
    //   return orderedObstacles[0].center; //placeholder if all are covered
    // else
    //   return target;
    
    
    //Take the nearest opponent to own goal
    std::vector<Obstacle> orderedObstacles;
    for(const auto& obstacle : theTeamPlayersModel.obstacles) 
        if(obstacle.type == Obstacle::opponent) 
            orderedObstacles.push_back(obstacle);
    std::sort(orderedObstacles.begin(), orderedObstacles.end(), orderObstaclesByXCoordinate);

    if(orderedObstacles.size() > 0)
      return orderedObstacles[0].center;
    else
      return Vector2f(theFieldDimensions.xPosOwnGroundLine,0.f);

}

Vector2f LibSpecProvider::freeCorridor(const Vector2f leftEnd, const Vector2f rightEnd, const float width) const{
  
  Vector2f myPos = theRobotPose.translation;
  Vector2f target = Vector2f(0.f,0.f);

  //collect list of teammates' and opponents' positions
  std::vector<Vector2f> obstacles;
  std::vector<Vector2f> targets;

  //potential target points on the segment (middle field line or opponent goal line)
  Geometry::Line segment(leftEnd, rightEnd - leftEnd);
  const float length = (rightEnd - leftEnd).norm();
  const Vector2f unitVec = segment.direction / length;
  float n = 40; //number of points inside the segment (has to be even)
  float mid = n/2;
  for(int i = 0; i < mid; ++i){ 
    Vector2f nextPointToRight = leftEnd + ((mid+i)/n)*length * unitVec;
    Vector2f nextPointToLeft = leftEnd + ((mid-i)/n)*length * unitVec;
    targets.push_back(nextPointToRight);
    targets.push_back(nextPointToLeft);
  }

  //potential targets (prendiamo l'intersezione col segmento?)
  for(auto const& obstacle : theTeamPlayersModel.obstacles) {
    Vector2f obstaclePos = obstacle.center;
    const Vector2f midPoint(theFieldDimensions.xPosOpponentGroundLine, 0.f);
    float obsDistanceToGoal = (obstaclePos - midPoint).norm();
    float myDistanceToGoal = (theRobotPose.translation - midPoint).norm();
    if(obsDistanceToGoal > myDistanceToGoal) continue;
    obstacles.push_back(obstacle.center);
  } 

  for(Vector2f target : targets){
    bool isCorridorFree = true;
    for(Vector2f oppPos : obstacles){
      if(isObstacleInCorridor(myPos, target, width, oppPos)){
        isCorridorFree = false; 
        break; //break the internal cycle, it means the current target is not feasible
      }
    }
    if(isCorridorFree == true){
      CIRCLE("module:LibObstaclesProvider:rimando", target.x(), target.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::blue, Drawings::BrushStyle::solidBrush, ColorRGBA::yellow);
      return target;
    } 
  }
  return target;
}

bool LibSpecProvider::isObstacleInCorridor(const Vector2f start, const Vector2f end, const float width, const Vector2f obstacle) const {
    
    Geometry::Line segment(start, end - start);

    float distance{};
    if(segment.direction.x() == 0 && segment.direction.y() == 0)
      distance = (start - obstacle).norm();

    const float d = (obstacle - segment.base).dot(segment.direction) / segment.direction.dot(segment.direction);

    if(d < 0 || d > 1.0f)
      return false;
    else
      distance = abs(Geometry::getDistanceToLine(segment, obstacle));

    return (distance >= 0 && distance < width);
}


Vector2f LibSpecProvider::nearestPointOnCorridor(const Vector2f start, const Vector2f end) const {

      LINE3D("module:LibSpecProvider:corridor", start.x(), start.y(), 0.0, end.x(), end.y(), 0.0, 5, ColorRGBA::red);
      Geometry::Line segment(start, end - start);
      Vector2f robot = theRobotPose.translation;
      const float length = segment.direction.dot(segment.direction);
      const float localProjection = (robot - segment.base).dot(segment.direction) / length;
      const Vector2f unitdir = segment.direction / length;

      Vector2f target{};
      if(localProjection <= 0)
        target = start + 2000.f * unitdir;
      else if(localProjection >= 1.0f)
        target = end - 700.f * unitdir;
      else
        target = start + localProjection * segment.direction;

      LINE3D("module:LibSpecProvider:shortestpath", robot.x(), robot.y(), 0.0, target.x(), target.y(), 0.0, 5, ColorRGBA::blue);
      return target; 
}


/*params
**    return: 0=most dangerous zone, 1=near dangerous zone, 2=far dangerous zone
*/
int LibSpecProvider::calcCornerZone(const Vector2f& point) const{
  if(Geometry::isPointInsideRectangle(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftPenaltyArea),
                                      Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea),
                                      point)
    )
      return 0;
  else if(point.x() < (theFieldDimensions.xPosOwnPenaltyArea/2))
      return 1;
  else
      return 2;
}


Vector2f LibSpecProvider::targetDefenseCornerPoint() const{

  // set the bounds for the sector of defender (startbounds = corner, bound1 = point of defender closer to ground line, bound2 = other)
  Vector2f target_point;
  Vector2f corner;
  (theTeamBallModel.position.y() > 0) ? ({ // left corner
    //preliminary target position (to avoid crash)
    target_point = Vector2f((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnPenaltyMark)/2, (theFieldDimensions.yPosLeftPenaltyArea + theFieldDimensions.yPosLeftGoalArea)/2);
    corner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline);
  }) : ({                                   // right corner
    //preliminary target position (to avoid crash)
    target_point = Vector2f((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnPenaltyMark)/2, (theFieldDimensions.yPosRightPenaltyArea + theFieldDimensions.yPosRightGoalArea)/2);
    corner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline); 
  });

  // Take the most dangerous opponent
  Vector2f dangerous_obstacle = getMostDangerousOpponent();
  if(dangerous_obstacle != Vector2f(theFieldDimensions.xPosOwnGroundLine,0.f)){ // this is the default value
    
    // we have the most dangerous obstacle

    // choose the target point by checking how much is dangerous the obstacle (looking in which zone the obstacle is)
    // (zone = 0: most dangerous, zone = 1: could be dangerous, zone = 2: not really dangerous)
    int cornerZone = calcCornerZone(dangerous_obstacle);
    if(cornerZone == 0){
      target_point = dangerous_obstacle + (corner-dangerous_obstacle)/6.f;
    }
    else if(cornerZone == 1){
      Vector2f ownGoalPoint = Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f);
      target_point = ownGoalPoint + (dangerous_obstacle - ownGoalPoint) / 2.f;
    }
  }

  CYLINDER3D("module:LibSpecProvider:targetPosition", target_point.x(), target_point.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 60.0f, ColorRGBA::violet);
  //CIRCLE("module:LibSpecProvider:targetPosition", target_point.x(), target_point.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::blue, Drawings::BrushStyle::solidBrush, ColorRGBA::blue);

  return target_point;
}

bool LibSpecProvider::isGoaliePlaying() const{
  for(auto mate: theTeamData.teammates){
    if(mate.isGoalkeeper && !mate.isPenalized){
      OUTPUT_TEXT("goalie gioca");
      return true;
    }
  }
  return false;
}

bool LibSpecProvider::isTherePassKickoffCondition(Vector2f& passTarget){
  bool isTherePass = false;
  for(const Teammate& mate : theTeamData.teammates){
      if(mate.theRobotPose.translation.x() > -1200.f && mate.theRobotPose.translation.y() > 500.f){
          isTherePass = true;
          passTarget = Vector2f(0.f, 1500.f);
          break;
      }
      else if(mate.theRobotPose.translation.x() > -1200.f && mate.theRobotPose.translation.y() < -500.f){
          isTherePass = true;
          passTarget = Vector2f(0.f, -1500.f);
          break;
      }
  }
  return isTherePass;
}