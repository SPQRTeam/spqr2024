/**
 * @file LibStrikerProvider.cpp
 * 
 * This file implements a module that provides some utilities (primarily) for the striker.
 */

#include "LibStrikerProvider.h"
#include "Tools/Math/BHMath.h"
#include <iostream>
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"

using Line2 = Eigen::Hyperplane<float,2>;


MAKE_MODULE(LibStrikerProvider, behaviorControl);

void LibStrikerProvider::update(LibStriker& libStriker)
{
  libStriker.projectGazeOntoOpponentGroundline = [this]() -> float {
    return projectGazeOntoOpponentGroundline();
  };
  libStriker.computeFreeAreas = [this](float minimumDiscretizedAreaSize) -> std::vector<FreeGoalTargetableArea> {
    return computeFreeAreas(minimumDiscretizedAreaSize);
  };
  libStriker.goalTarget = [this](bool shootASAP, bool forceHeuristic) -> Vector2f {
    return goalTarget(shootASAP, forceHeuristic);
  };
  libStriker.goalTargetWithArea = [this](bool shootASAP, bool forceHeuristic) -> std::pair<Vector2f, FreeGoalTargetableArea> {
    return goalTargetWithArea(shootASAP, forceHeuristic);
  };
  libStriker.getKick = [this](bool kickAsap, bool kickRight) -> KickInfo::KickType {
    return getKick(kickAsap, kickRight);
  };
  libStriker.getWalkKick = [this](GlobalVector2f target) -> KickInfo::KickType {
    return getWalkKick(target);
  };
  libStriker.shouldKick = [this](bool currentlyKicking, float kickIntervalThreshold) -> bool {
    return shouldKick(currentlyKicking, kickIntervalThreshold);
  };

  libStriker.strikerPosition = getStrikerPosition();
  libStriker.strikerDribblePoint = getStrikerDribblePoint();

  DECLARE_DEBUG_DRAWING3D("module:LibStrikerProvider:strikerPosition", "field");
  DECLARE_DEBUG_DRAWING3D("module:LibStrikerProvider:strikerDribblePoint", "field");
  DECLARE_DEBUG_DRAWING3D("module:LibStrikerProvider:strikerDribblePointVerbose", "field");
  // if(thePlayerRole.role==PlayerRole::striker){
  //   CYLINDER3D("module:LibStrikerProvider:strikerPosition", libStriker.strikerPosition.x(), libStriker.strikerPosition.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 20.0f, ColorRGBA::red);
  // }
}


// TODO maybe rewrite according to better practices (and using Eigen)
float LibStrikerProvider::projectGazeOntoOpponentGroundline() const {
  const float EPS = 1e-6f;
  float rotation = theRobotPose.rotation;
  //To avoid tan singularities
  if(std::abs(rotation) == pi/2) rotation -= EPS;
  return theRobotPose.translation.y() + std::abs(theFieldDimensions.xPosOpponentGoal - theRobotPose.translation.x())*tanf(theRobotPose.rotation);
}

float LibStrikerProvider::projectPointOntoOpponentGroundline(float x, float y) const
{
  return ((theFieldDimensions.xPosOpponentGroundLine - theRobotPose.translation.x())/(x - theRobotPose.translation.x()))*(y - theRobotPose.translation.y()) + theRobotPose.translation.y();
};

bool LibStrikerProvider::areOverlappingSegmentsOnYAxis(float l1, float r1, float l2, float r2) const
{
  return ((l1<l2 && l1>r2) || (r1<l2 && r1>r2) || (l2<l1 && l2>r1) || (r2<l1 && r2>r1));
};

// Note: We are considering only the closest obstacle to the freeArea.
float LibStrikerProvider::areaValueHeuristic(const float leftLimit, const float rightLimit, const float poles_weight, const float opponents_weight, const float teammates_weight) const
{
  // Useful notes for the reader:
  //       leftLimit is the left limit of THIS free area, rightLimit is the right one.
  //       Note that the axis is directed to the left, then leftLimit > righLimit.

  //std::vector<float> opponents_distances;
  //std::vector<float> teammates_distances;
  std::vector<float> distances;
  std::vector<float> weights;


  Pose2f freeAreaPoseleftLimit= Pose2f(theFieldDimensions.xPosOpponentGroundLine,leftLimit);
  Pose2f freeAreaPoserightLimit= Pose2f(theFieldDimensions.xPosOpponentGroundLine,rightLimit);

  float final_utility = 0;

  Obstacle nearest_opponent;
  float nearest_opponent_distance = INFINITY;

  //1) Find nearest opponent
  for(auto obs: theTeamPlayersModel.obstacles)
  {
    //float obs_center_proj = projectPointOntoOpponentGroundline(obs.center.x(),obs.center.y());
    float obs_left_proj = projectPointOntoOpponentGroundline(obs.left.x(),obs.left.y());
    float obs_right_proj = projectPointOntoOpponentGroundline(obs.right.x(),obs.right.y());

    // IT'S NOT POSSIBLE THAT THIS PROJECTION IS INTO SOME FREE_AREAS. Then, just check if it is left or right wrt the obstacle.

    float distance;
    if(obs_right_proj >= leftLimit){ // Obstacle is on the right side of the freeArea
      distance=obs_right_proj-leftLimit;
    }

    if(obs_left_proj<= rightLimit){   // Obstacle is on the left side of the freeArea
      distance=rightLimit-obs_left_proj;
    }

    if(distance < nearest_opponent_distance)
    {
      nearest_opponent = obs;
      nearest_opponent_distance = distance;
    }
  }

  //Add distance from nearest opponent, appropriately weighed
  final_utility = nearest_opponent_distance * theOpponentGoalModel.utilityNearestOpponentWeight;

  //2) Find other distances and weight them appropriately
  for(auto obs : theTeamPlayersModel.obstacles) {

    //Skip the nearest opponent as it is treated differently
    // [2022] TODO compiler says this line has unreachable code? why?
    if(&obs == &nearest_opponent) continue;

    if(theRobotPose.translation.x() > obs.center.x()){
      continue; // if the obstacle is behind me, it's not considered
    }

    float final_distance;
    float final_weight;

    //###########     Considering projection wrt my eyes

    //float obs_center_proj = projectPointOntoOpponentGroundline(obs.center.x(),obs.center.y());
    float obs_left_proj = projectPointOntoOpponentGroundline(obs.left.x(),obs.left.y());
    float obs_right_proj = projectPointOntoOpponentGroundline(obs.right.x(),obs.right.y());

    //Obstacles can be only to the left or to the right of free areas (not inside)

    if(obs_right_proj >= leftLimit){ // Obstacle is on the right side of the freeArea
      final_distance=obs_right_proj-leftLimit;
    }

    if(obs_left_proj<= rightLimit){   // Obstacle is on the left side of the freeArea
      final_distance=rightLimit-obs_left_proj;
    }

    switch(obs.type){
      case Obstacle::Type::opponent:
      {
        //Reweight based on vicinity of the opponent to area midpoint
        Vector2f midpoint(theFieldDimensions.xPosOpponentGroundLine, (leftLimit - rightLimit)/2);
        float opponent_distance_factor = theOpponentGoalModel.utilityOpponentsXDistanceThreshold - (midpoint-obs.center).norm();
        float remapped_opponent_weight = mapToRange(opponent_distance_factor, 0.f, theOpponentGoalModel.utilityOpponentsXDistanceThreshold, 0.f, theOpponentGoalModel.utilityOpponentsWeight);
        //std::cout<<"remapped_opponent_weight: "<<remapped_opponent_weight<<std::endl;
        final_weight = remapped_opponent_weight;

        final_utility += final_weight * final_distance;
        break;
      }
      case Obstacle::Type::teammate:
      {
        final_weight = theOpponentGoalModel.utilityTeammatesWeight;
        final_utility += final_weight * final_distance;
        break;
      }
      default:
      {
        final_weight = 0;
        break;
      }
    }

    distances.push_back(final_distance);
    weights.push_back(final_weight);
  }

  float pole_distance_penalty;
  if(std::abs(theFieldDimensions.yPosLeftGoal - leftLimit) > std::abs(theFieldDimensions.yPosRightGoal - rightLimit))
  {
    //nearest_pole_distance = std::abs(theFieldDimensions.yPosRightGoal - rightLimit);
    pole_distance_penalty = std::abs(rightLimit);
  }
  else
  {
    //nearest_pole_distance = std::abs(theFieldDimensions.yPosLeftGoal - leftLimit);
    pole_distance_penalty = std::abs(leftLimit);
  }

  pole_distance_penalty *= theOpponentGoalModel.utilityPolesWeight;
  //std::cout<<"pole_distance_penalty: "<<std::endl;
  //std::cout<<pole_distance_penalty<<std::endl;
  //std::cout<<"final_utility: "<<std::endl;
  //std::cout<<final_utility<<std::endl;

  //return final_utility-pole_distance_penalty;
  //Squared to avoid oscillations
  // [2022] cast to silence warning
  return static_cast<float> (pow(final_utility-pole_distance_penalty, 2));

  //Ideas that can be exploited in order to improve this function:
  /*
  1) First criteria: vicinity to jolly (or in alternative the second robot nearest to the ball, as the first one is the striker)
  2) Second criteria: number of opponent in intercept range
  3) Third criteria: vicinity of enemy goalie (is this possible?)
  4) Width of parent area
  5) Penalize the poles, but this can be done at an higher level of abstraction
  */
};

std::vector<FreeGoalTargetableArea> LibStrikerProvider::computeFreeAreas(float minimumDiscretizedAreaSize) const
{
  /*
  NOTICE:
  begin = leftLimit
  end = rightLimit
  */

  Pose2f myPose = Pose2f(theRobotPose.translation);

  float GOAL_TARGET_OBSTACLE_INFLATION = theOpponentGoalModel.goalTargetObstacleInflation;
  float GOAL_TARGET_AREA_DISCRETIZATION = theOpponentGoalModel.useAreaDiscretization;
  float GOAL_POST_RADIUS = theFieldDimensions.goalPostRadius;
  float GOAL_POLE_MINIMUM_TARGET_OFFSET = theOpponentGoalModel.goalPoleMinimumTargetOffset;

  float GOAL_LINE_LEFT_LIMIT = theFieldDimensions.yPosLeftGoal - GOAL_POST_RADIUS - GOAL_POLE_MINIMUM_TARGET_OFFSET;
  float GOAL_LINE_RIGHT_LIMIT = theFieldDimensions.yPosRightGoal + GOAL_POST_RADIUS + GOAL_POLE_MINIMUM_TARGET_OFFSET;

  //vector of opponents
  std::vector<Obstacle> opponents;
  //vector of poles
  std::vector<Obstacle> poles;

  for(auto obs : theTeamPlayersModel.obstacles){
    /*NOTICE: poles are added statically (a priori) by the vision system
      so the 4 goal poles will always be in the obstacle list*/
    switch(obs.type)
    {
      case Obstacle::Type::goalpost:
      {
        if(obs.type==Obstacle::Type::goalpost && obs.center.x()>0)
        {
          //std::cout<<"Found opponent goal post: (left:"<<obs.left.y()<<", right:"<<obs.right.y()<<")\n";
          poles.push_back(obs);
        }
        break;
      }
      default:
      {
        if(obs.center.x()>theRobotPose.translation.x() || obs.left.x()>theRobotPose.translation.x() || obs.right.x()>theRobotPose.translation.x())
        {
          opponents.push_back(obs);
        }
        break;
      }
    }
  }

  Pose2f leftPole, rightPole;

  if(poles.size()==0)
  {
    leftPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundLine;
    leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),730.);
    if(leftPole.translation.y()>GOAL_LINE_LEFT_LIMIT) leftPole.translation.y()=GOAL_LINE_LEFT_LIMIT;
    rightPole.translation.x() = (float)theFieldDimensions.xPosOpponentGroundLine;
    rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),-730.);
    if(rightPole.translation.y()<GOAL_LINE_RIGHT_LIMIT) leftPole.translation.y()=GOAL_LINE_RIGHT_LIMIT;
  }
  else
  {

    leftPole.translation.x() = poles.at(0).right.x();
    if(leftPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundLine) leftPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundLine;

    leftPole.translation.y() = projectPointOntoOpponentGroundline(leftPole.translation.x(),poles.at(0).right.y());
    if(leftPole.translation.y()>GOAL_LINE_LEFT_LIMIT) leftPole.translation.y()=GOAL_LINE_LEFT_LIMIT;

    rightPole.translation.x() = poles.at(1).left.x();
    if(rightPole.translation.x()>(float)theFieldDimensions.xPosOpponentGroundLine) rightPole.translation.x()=(float)theFieldDimensions.xPosOpponentGroundLine;

    rightPole.translation.y() = projectPointOntoOpponentGroundline(rightPole.translation.x(),poles.at(1).left.y());
    if(rightPole.translation.y()<GOAL_LINE_RIGHT_LIMIT) rightPole.translation.y()=GOAL_LINE_RIGHT_LIMIT;
  }

  //FREE AREAS COMPUTATION

  std::vector<float> leftPoints;
  std::vector<float> rightPoints;
  std::vector<FreeGoalTargetableArea> freeAreas;


  Obstacle swapper;

  /*1) Sort the opponents in vector based on the y coordinate of their left points
  (for each obstacle, the leftmost point I see)*/
  for(int i = 0; i < opponents.size(); i++){
    for(int k = 0; k < opponents.size(); k++){
      float firstLeft = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y());
      float secondLeft = projectPointOntoOpponentGroundline(opponents.at(k).left.x(),opponents.at(k).left.y());
      if(firstLeft > secondLeft ){
        swapper = opponents.at(k);
        opponents.at(k) = opponents.at(i);
        opponents.at(i) = swapper;
      }
    }
  }

  /*2) Find overlapping obstacles and merge them into a single one. For each obstacle (including the
    ones obtained by merging) save the projections on the goal line of its left and right point, in order
    to populate a list of obstacles from the player's point of view.

    NOTICE: the obstacles are ordered from left to right

    NOTICE: GOAL_TARGET_OBSTACLE_INFLATION is used as an "obstacle inflation" factor
    to enlarge obstacles (as the visualization system makes the opponent robots smaller than their
    feet width
  */

  if(opponents.size()>0)
  {
    float leftPointY = projectPointOntoOpponentGroundline(opponents.at(0).left.x(),opponents.at(0).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/(theRobotPose.translation-opponents.at(0).left).norm();
    float rightPointY = projectPointOntoOpponentGroundline(opponents.at(0).right.x(),opponents.at(0).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/(theRobotPose.translation-opponents.at(0).right).norm();
    if(opponents.size()==1)
    {
      //If the obstacle projection is at least partially inside the goal line add it
      if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
      {
        //std::cout<<"1 Adding single obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
        leftPoints.push_back(leftPointY);
        rightPoints.push_back(rightPointY);
      }
    }
    else if(opponents.size()>1)
    {
      int i=1;
      bool wereOverlapping;

      /*
        For each obstacle in the list, compare it against the next one to check for overlapping and,
        depending on the kind of overlapping, merge them accordingly or add them separately
      */
      while(i<opponents.size()+1)
      {
        float nextLeftPointY, nextRightPointY;
        if(i==opponents.size())
        {
          /*
            If leftPoint and rightPoint identify the last obstacle in the opponents list, use
            the right pole as the next obstacle to compare for overlapping
          */
          nextLeftPointY = projectPointOntoOpponentGroundline(poles.at(1).left.x(),poles.at(1).left.y());
          nextRightPointY = projectPointOntoOpponentGroundline(poles.at(1).right.x(),poles.at(1).right.y());
        }
        else
        {
          /*
            Else use the next obstacle in the list
          */
          nextLeftPointY = projectPointOntoOpponentGroundline(opponents.at(i).left.x(),opponents.at(i).left.y())+GOAL_TARGET_OBSTACLE_INFLATION*1000/(theRobotPose.translation-opponents.at(i).left).norm();
          nextRightPointY = projectPointOntoOpponentGroundline(opponents.at(i).right.x(),opponents.at(i).right.y())-GOAL_TARGET_OBSTACLE_INFLATION*1000/(theRobotPose.translation-opponents.at(i).right).norm();
        }

        /*
          Check for overlapping: there are three cases to manage
          1) One obstacle is inside the other: do nothing
          2) Obstacles overlap only partially: merge the obstacles
          3) No overlap: add the first obstacle and pass to the second one
        */
        if(areOverlappingSegmentsOnYAxis(leftPointY, rightPointY, nextLeftPointY, nextRightPointY))
        {

          if(leftPointY>nextLeftPointY && rightPointY<nextRightPointY)
          {
            //1) One obstacle is inside the other: do nothing

            //std::cout<<"CASE 1\n";
            //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
          }
          else
          {
            //2) Obstacles overlap only partially: merge the obstacles

            //std::cout<<"CASE 2\n";
            rightPointY = nextRightPointY;

            //std::cout<<"Current obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";

          }
          wereOverlapping=true;
        }
        else
        {
          //3) No overlap: add the first obstacle and pass to the second one
          if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
          {
            //Add the obstacle projection only if it falls (even only partially) inside the goal line
            //std::cout<<"2 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
            leftPoints.push_back(leftPointY);
            rightPoints.push_back(rightPointY);
          }
          leftPointY = nextLeftPointY;
          rightPointY = nextRightPointY;
          wereOverlapping=false;
        }

        i++;
      }

      //Add last remaining obstacle points (only in case where it overlaps right pole)
      if(wereOverlapping)
      {
        if(areOverlappingSegmentsOnYAxis(leftPointY,rightPointY,leftPole.translation.y(),rightPole.translation.y()))
        {
          //Add the obstacle projection only if it falls (even only partially) inside the goal line
          //std::cout<<"3 Adding obstacle: ("+std::to_string(leftPointY)+","+std::to_string(rightPointY)+")\n";
          leftPoints.push_back(leftPointY);
          rightPoints.push_back(rightPointY);
        }
      }
    }
  }

  //std::cout<<"End of overlap check\n\n";

  /*3) Now that we have left and right points of the obstacles
    3.1) We first determine if there is any free area (or if a single obstacle
          projection overlaps the whole goal line)
    3.2) We shrink the left and right limit (begin/end) of the goal line if there are
          obstacles overlapping the left/right poles
  */
  //determines if all obstacles are outside goal
  bool noneInside = true;

  //Consider my angle in inflating the pole
  float begin = leftPole.translation.y();
  float end = rightPole.translation.y();

  for(int i = 0; i < leftPoints.size(); i++){

    //3.1)
    //at least one point inside goal
    if(leftPoints.at(i)<leftPole.translation.y() && rightPoints.at(i)>rightPole.translation.y() ||
    leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i)<leftPole.translation.y() ||
    leftPoints.at(i)>rightPole.translation.y() && rightPoints.at(i)<rightPole.translation.y())
    {
      noneInside = false;
    }
    //WATCH OUT, TEMPORARY SOLUTION: If an obstacle projection cover the whole goal area I return an empty area list
    if(leftPoints.at(i)>leftPole.translation.y() && rightPoints.at(i) < rightPole.translation.y())
    {
      return freeAreas;
    }

    //3.2)
    //left obstacle border outside goal, right border inside
    if(leftPoints.at(i) > leftPole.translation.y() && rightPoints.at(i) < leftPole.translation.y()){
      begin = rightPoints.at(i);
    }
    //right obstacle border outside goal, left border inside
    if(leftPoints.at(i) > rightPole.translation.y() && rightPoints.at(i) < rightPole.translation.y()){
      end = leftPoints.at(i);
    }
  }

  /*4) Build the free targetable segments vector
    4.1) If there is no targetable segment, return the empty vector
    4.2) Else populate the freeAreas vector
  */

  //4.1)
  if(noneInside == true){
    freeAreas.push_back(FreeGoalTargetableArea(begin, end, 1));
    //std::cout<<"NONE INSIDE\n";
    return freeAreas;
  }
  std::vector<float> freeAreasPoints;
  freeAreasPoints.push_back(begin);

  //4.2)
  for(int i = 0; i < leftPoints.size(); i++){
    if(leftPoints.at(i) < begin && leftPoints.at(i) > end){
      freeAreasPoints.push_back(leftPoints.at(i));
    }
    if(rightPoints.at(i) < begin && rightPoints.at(i) > end ){
      freeAreasPoints.push_back(rightPoints.at(i));
    }
  }
  freeAreasPoints.push_back(end);

  sort(freeAreasPoints.begin(),freeAreasPoints.end());

  /*5A) Apply discretization to the targetable segments, by dividing targetable areas in smaller segments,
      in order to assign them a utility value
  */

  if(GOAL_TARGET_AREA_DISCRETIZATION)
  {
    for(int i = freeAreasPoints.size()-1; i-1>=0; i-=2)
    {
      float beginning = freeAreasPoints.at(i);
      float end = freeAreasPoints.at(i-1);
      float size = std::abs(end-beginning);
      if(size>minimumDiscretizedAreaSize)
      {
        int numberOfDiscretizedAreasInFreeArea = floor(size/minimumDiscretizedAreaSize);

        float discretizedAreaSize = size/numberOfDiscretizedAreasInFreeArea;

        float firstPoint = beginning;
        for(int i = numberOfDiscretizedAreasInFreeArea -1; i>0; i--)
        {
          float lastPoint = firstPoint-discretizedAreaSize;
          freeAreas.push_back(FreeGoalTargetableArea(firstPoint,lastPoint,areaValueHeuristic(firstPoint,lastPoint)));
          firstPoint = lastPoint;
        }
          freeAreas.push_back(FreeGoalTargetableArea(firstPoint,end,areaValueHeuristic(firstPoint,end)));
      }
      else
      {
        continue;
      }

    }

    /*
      6) Sort the result of the discretization on its utility value
    */

    //Sort the freeAreas vector in a decreasing order of utility values
    sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
  }
  else
  {
    /*
      5B) Do not discretize free areas: the whole area between opponent projections is used:
          might be imprecise and less effective and also will require longer times to align with the ball
    */
    for(int i = freeAreasPoints.size() -1; i-1 >= 0; i-=2)
    {
      freeAreas.push_back(FreeGoalTargetableArea(freeAreasPoints.at(i), freeAreasPoints.at(i-1), areaValueHeuristic(freeAreasPoints.at(i),freeAreasPoints.at(i-1))));
    }
    sort(freeAreas.begin(),freeAreas.end(), [](const FreeGoalTargetableArea &x, const FreeGoalTargetableArea &y){ return (x.value > y.value);});
  }

  return freeAreas;
}

Vector2f LibStrikerProvider::goalTarget(bool shootASAP, bool forceHeuristic) const {
  return goalTargetWithArea(shootASAP, forceHeuristic).first;
}

/*
@author Emanuele Musumeci (original goalTarget), Francesco Petri, Fabian Fonseca (adaptation to return the area as well)
Return a Vector2f containing the chosen target and the associated FreeGoalTargetableArea based on the selected mode. There are two modes:
1) the default mode (shootASAP=false) decides whether
    A) to shoot to the nearest possible target (this happens if the robot
    has a distance from the opponent goal less or equal than GOAL_TARGET_DISTANCE_THRESHOLD)
    B) to choose the best target based on its utility value
2) the shootASAP mode (shootASAP=true) instead forces shooting to the nearest possible target
*/
//TODO: Move constant parameters to CFG
std::pair<Vector2f, FreeGoalTargetableArea> LibStrikerProvider::goalTargetWithArea(bool shootASAP, bool forceHeuristic) const
{
  float GOAL_TARGET_AREA_MIN_SIZE = theOpponentGoalModel.goalTargetAreaMinSize;
  float GOAL_TARGET_MIN_OFFSET_FROM_SIDE = theOpponentGoalModel.goalTargetMinOffsetFromSide;
  float GOAL_TARGET_DISTANCE_THRESHOLD = theOpponentGoalModel.goalTargetDistanceThreshold;

  std::vector<FreeGoalTargetableArea> freeAreas = computeFreeAreas(GOAL_TARGET_AREA_MIN_SIZE);

  /*
    1) Filter free areas, ignoring the smaller ones
  */
  std::vector<FreeGoalTargetableArea> filteredFreeAreas;
  for(const auto& area : freeAreas)
  {
    if(area.interval>=GOAL_TARGET_AREA_MIN_SIZE)
    {
      filteredFreeAreas.push_back(area);
      //std::cout << ("[Robot #"+std::to_string(theRobotInfo.number)+"] FreeGoalTargetableArea: ("+std::to_string(area.begin)+","+std::to_string(area.end)+")") << "\n";
    }
  }

  std::pair<Vector2f, FreeGoalTargetableArea> targetAndArea;

  /*
    2) If there is no free area, return the median point on the opponent goal line
  */
  //WATCH OUT: SPECIAL RETURN AS AN AREA WITH 0 LENGTH AND NEGATIVE UTILITY, TO MANAGE IN STRIKER BEHAVIOR
  if(filteredFreeAreas.size()==0)
  {
    targetAndArea.first = Vector2f(theFieldDimensions.xPosOpponentGroundLine,0);
    targetAndArea.second = FreeGoalTargetableArea(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.xPosOpponentGroundLine, -1000);
    return targetAndArea;
  }

  //project my line of view onto the goal line
  float myGazeProjection = projectGazeOntoOpponentGroundline();

  float targetPoint = 0;
  FreeGoalTargetableArea areaAssociatedToTargetPoint;

  float minTargetableAreaDistance=INFINITY;

  //CASE 1: I'm near the goal post -> choose the nearest free area
  if(!forceHeuristic && 
      (shootASAP || (std::abs(theFieldDimensions.xPosOpponentGroundLine-theRobotPose.translation.x())<GOAL_TARGET_DISTANCE_THRESHOLD && filteredFreeAreas.size()!=0) || filteredFreeAreas.size()==1))
  {
    for(int i = 0; i < filteredFreeAreas.size(); i++)
    {
      FreeGoalTargetableArea currentArea = filteredFreeAreas.at(i);
      //CASE 1.1: Looking at free area directly
      if(myGazeProjection<currentArea.begin && myGazeProjection>currentArea.end)
      {
        if(myGazeProjection>currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
        else if(myGazeProjection<currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE) targetPoint = currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
        else targetPoint = myGazeProjection;
        areaAssociatedToTargetPoint = currentArea;
        break;
      }
      //CASE 1.2: Looking away from free area
      else
      {
        //if freeArea is on the left
        if(currentArea.begin<myGazeProjection)
        {
          float currentFreeAreaDistance=myGazeProjection-currentArea.begin;
          if(minTargetableAreaDistance>currentFreeAreaDistance)
          {
            minTargetableAreaDistance=currentFreeAreaDistance;
            targetPoint= currentArea.begin-GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
            areaAssociatedToTargetPoint = currentArea;
          }
        }
        //if freeArea is on the right
        else if(currentArea.end>myGazeProjection)
        {
          float currentFreeAreaDistance=currentArea.end-myGazeProjection;
          if(minTargetableAreaDistance>currentFreeAreaDistance)
          {
            minTargetableAreaDistance=currentFreeAreaDistance;
            targetPoint= currentArea.end+GOAL_TARGET_MIN_OFFSET_FROM_SIDE;
            areaAssociatedToTargetPoint = currentArea;
          }
        }
      }
    }
  }
  //CASE 2: I'm far away from the goal post -> find the area with the highest utility
  else
  {
    //The freeAreas vector is sorted in a decreasing order of utility values, so I select the first area    

    //Use whole area
    targetPoint = filteredFreeAreas.at(0).midpoint;
    areaAssociatedToTargetPoint = filteredFreeAreas.at(0);
  }

  //If I'm too near to the goal line, shoot inside the goal, else shoot on the goal line
  Vector2f target;
  
  //EMANUELE M. changed the threshold a bit, feel free to change back (was 600, is 1000) (also note this change was made prior to 2022)
  if (theRobotPose.translation.x() >= (theFieldDimensions.xPosOpponentGroundLine - 1000.f) &&
          std::abs(theRobotPose.translation.y()) < 500.f )
    target = Vector2f(theFieldDimensions.xPosOpponentGroundLine + 1000.f, targetPoint);
  else
    target = Vector2f(theFieldDimensions.xPosOpponentGroundLine, targetPoint);
  
  targetAndArea.first = target;
  targetAndArea.second = areaAssociatedToTargetPoint;
  return targetAndArea;
}

GlobalVector2f LibStrikerProvider::getStrikerDribblePoint(){

  // Represents the effective range of a dribbling action, influencing distance calculations and decision-making during dribbling maneuvers.
  float steplen = 1000.f;

  //* Center of dribbling: the ball position
  GlobalVector2f center = theFieldBall.recentBallPositionOnField();
  Geometry::Circle circle = Geometry::Circle(center, steplen);
  GlobalVector2f opponent_goal = GlobalVector2f(theFieldDimensions.xPosOpponentGoalArea, 0.f);

  //* Compute the goal intersection with the circle (centered in "center" and with radius "steplen")
  // There are always two intersections with the circle (so it is not necessary to check the number of intersections returned)
  Geometry::Line to_goal = Geometry::Line(center, opponent_goal-center);
  
  GlobalVector2f goal_intersection_1;
  GlobalVector2f goal_intersection_2;
  int num_of_interesections = Geometry::getIntersectionOfLineAndCircle(to_goal, circle, goal_intersection_1, goal_intersection_2);

  //* Choose the intersection closest to the opponent goal
  GlobalVector2f goal_intersection = (goal_intersection_1-opponent_goal).norm() < (goal_intersection_2-opponent_goal).norm() ? goal_intersection_1 : goal_intersection_2;
  
  //* Goal weight: inverse of the distance from the goal: the closer the goal, the higher the weight
  float goal_weight = 1/(center - opponent_goal).norm(); 
  
  //* The target is set to the goal intersection
  GlobalVector2f target = goal_intersection;


  //* Compute the limit intersections: the points where the circle intersects the line perpendicular to the line from the center to the goal
  //* The dribbling area is limited on the semi-circle towards the goal (i.e. it is not possible to dribble towards the own goal)
  // There are always two intersections with the circle (so it is not necessary to check the number of intersections returned)
  Geometry::Line limit_line = Geometry::Line(center, Vector2f(-to_goal.direction.y(), to_goal.direction.x()));
  GlobalVector2f limit_intersection_1;
  GlobalVector2f limit_intersection_2;
  num_of_interesections = Geometry::getIntersectionOfLineAndCircle(limit_line, circle, limit_intersection_1, limit_intersection_2);
  GlobalVector2f left_limit = limit_intersection_1.y() > limit_intersection_2.y() ? limit_intersection_1 : limit_intersection_2;
  GlobalVector2f right_limit = limit_intersection_1.y() < limit_intersection_2.y() ? limit_intersection_1 : limit_intersection_2;


  //* Compute the intersection with the field borders 
  //* The order is relevant: right, left, up (opponent), down (own) limit
  //* There are three vectors:
  //* 1) field_border_limits_bool: contains the boolean values of the limits (if the limit is considered or not)
  //* 2) field_border_limits: contains the limits of the field borders, or (0,0) if the limit is not considered
  //* 3) field_border_intersections: contains the intersections of the field borders with the circle, or (0,0) if the limit is not considered
  GlobalVector2f right_opponent_corner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline);
  GlobalVector2f left_opponent_corner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
  GlobalVector2f right_own_corner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline);
  GlobalVector2f left_own_corner = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline);

  Geometry::Line right_field_border = Geometry::Line(right_opponent_corner, right_own_corner-right_opponent_corner);
  Geometry::Line left_field_border = Geometry::Line(left_opponent_corner, left_own_corner-left_opponent_corner);
  Geometry::Line opponent_goal_line = Geometry::Line(right_opponent_corner, left_opponent_corner-right_opponent_corner);
  Geometry::Line own_goal_line = Geometry::Line(right_own_corner, left_own_corner-right_own_corner);

  //* Lines passing through the center: horizontal and vertical lines wrt the field
  Geometry::Line center_horizontal_line = Geometry::Line(center, Vector2f(0,1));
  Geometry::Line center_vertical_line = Geometry::Line(center, Vector2f(1,0));

  GlobalVector2f right_field_border_limit;
  bool right_border_limit = Geometry::getIntersectionOfLines(center_horizontal_line, right_field_border, right_field_border_limit);

  GlobalVector2f left_field_border_limit;
  bool left_border_limit = Geometry::getIntersectionOfLines(center_horizontal_line, left_field_border, left_field_border_limit);

  GlobalVector2f opponent_goal_line_limit;
  bool up_line_limit = Geometry::getIntersectionOfLines(center_vertical_line, opponent_goal_line, opponent_goal_line_limit);

  GlobalVector2f own_goal_line_limit;
  bool down_line_limit = Geometry::getIntersectionOfLines(center_vertical_line, own_goal_line, own_goal_line_limit);
  
  std::vector<GlobalVector2f> field_border_limits = { GlobalVector2f(0.f,0.f), GlobalVector2f(0.f,0.f), GlobalVector2f(0.f,0.f), GlobalVector2f(0.f,0.f) }; // Only for debug drawing
  std::vector<GlobalVector2f> field_border_intersections = { GlobalVector2f(0.f,0.f), GlobalVector2f(0.f,0.f), GlobalVector2f(0.f,0.f), GlobalVector2f(0.f,0.f) };
  std::vector<bool> field_border_intersections_bool = { false, false, false, false }; 

  //* Starts to consider the field borders if the distance from the border is less than the steplen
  if(right_border_limit){
    field_border_limits.at(0) = right_field_border_limit; // Only for debug drawing
    if((right_field_border_limit-center).norm() < steplen){
      field_border_intersections.at(0) = goal_intersection;
      field_border_intersections_bool.at(0) = true;
    }
  }
  if(left_border_limit){
    field_border_limits.at(1) = left_field_border_limit; // Only for debug drawing
    if((left_field_border_limit-center).norm() < steplen){
      field_border_intersections.at(1) = goal_intersection;
      field_border_intersections_bool.at(1) = true;
    }
  }
  if(up_line_limit && (opponent_goal_line_limit.y()>theFieldDimensions.yPosLeftGoal || opponent_goal_line_limit.y()<theFieldDimensions.yPosRightGoal)) {
    field_border_limits.at(2) = opponent_goal_line_limit; // Only for debug drawing
    if((opponent_goal_line_limit-center).norm() < steplen){
      field_border_intersections.at(2) = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosCenterGoal);;
      field_border_intersections_bool.at(2) = true;
    }
  }
  if(down_line_limit){
    field_border_limits.at(3) = own_goal_line_limit; // Only for debug drawing
    if((own_goal_line_limit-center).norm() < steplen){
      field_border_intersections.at(3) = goal_intersection;
      field_border_intersections_bool.at(3) = true;
    }
  }
  
  //* COMPUTE the opponents intersections
  std::vector<Geometry::Circle> opponent_circles; // Only for debug drawing
  std::vector<std::tuple<GlobalVector2f, GlobalVector2f>> opponents_interesections;
  std::vector<std::tuple<GlobalVector2f, GlobalVector2f, GlobalVector2f>> opponents_sectors; 
  for(auto obstacle:theObstacleModel.obstacles){
    if((obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::fallenOpponent) && 
       (center - theLibMisc.rel2Glob(obstacle.center.x(), obstacle.center.y()).translation).norm() < 2*steplen){
      
      GlobalVector2f opponent = theLibMisc.rel2Glob(obstacle.center.x(), obstacle.center.y()).translation; 

      float opponent_circle_radius;
      if((center-opponent).norm()>0.5*steplen) opponent_circle_radius = steplen + steplen*((center-opponent).norm()-0.5*steplen)/(1.5*steplen);
      else                                     opponent_circle_radius = steplen;
      
      Geometry::Circle opponent_circle = Geometry::Circle(opponent, opponent_circle_radius);
      opponent_circles.push_back(opponent_circle); // Only for debug drawing

      //* Compute the sector of the opponent (left and right bounds, i.e. how much space the opponent occupies in the field in front of me)
      //* The sector depends on the distance of the opponent from me: the closer the opponent, the larger the sector (from 0 to 90 degrees) 
      float angle_of_opponent = atan2(opponent.y()-center.y(), opponent.x()-center.x());
      float sector_angle = (1-(center-opponent).norm()/(2*steplen)) * 45*M_PI/180;   
      float distance_of_opponent = (opponent-center).norm();

      float right_angle = theLibMisc.radiansToDegree(angle_of_opponent-sector_angle);
      float left_angle = theLibMisc.radiansToDegree(angle_of_opponent+sector_angle);

      GlobalVector2f left_bound = GlobalVector2f(center.x() + distance_of_opponent*cos(angle_of_opponent+sector_angle), center.y() + distance_of_opponent*sin(angle_of_opponent+sector_angle));
      GlobalVector2f right_bound = GlobalVector2f(center.x() + distance_of_opponent*cos(angle_of_opponent-sector_angle), center.y() + distance_of_opponent*sin(angle_of_opponent-sector_angle));
      opponents_sectors.push_back(std::make_tuple(opponent, left_bound, right_bound));

      //* Check if the goal intersection is inside the opponent circle. If it is not, the opponent is ignored
      if((goal_intersection-opponent).norm() > opponent_circle_radius) continue;

      GlobalVector2f opponent_intersection_1;
      GlobalVector2f opponent_intersection_2;
      GlobalVector2f opponent_intersection;
      int num_of_interesections = Geometry::getIntersectionOfCircles(circle, opponent_circle, opponent_intersection_1, opponent_intersection_2);

      if(num_of_interesections == 2){

        //* If the opponent is near to the goal line (i.e. is centered in front of me) go right by default
        //* Note: Questo caso è gestito così per evitare balletti davanti alla palla in situazioni di simmetria (calcio di inizio in particolare)   
        if(std::abs(Geometry::getDistanceToLine(to_goal, opponent)) < 200){
          opponent_intersection = opponent_intersection_1.y() < opponent_intersection_2.y() ? opponent_intersection_1 : opponent_intersection_2; //* go right
        }

        //* Else choose the intersection that is closer to the goal
        else{
          float angle_goalInt_circleInt1 = std::abs(theLibMisc.radiansToDegree(theLibMisc.angleBetweenGlobalVectors(center, goal_intersection, opponent_intersection_1)));
          float angle_goalInt_circleInt2 = std::abs(theLibMisc.radiansToDegree(theLibMisc.angleBetweenGlobalVectors(center, goal_intersection, opponent_intersection_2)));
          opponent_intersection = angle_goalInt_circleInt1 < angle_goalInt_circleInt2 ? opponent_intersection_1 : opponent_intersection_2;
        }
      }

      else if (num_of_interesections == 1) opponent_intersection = opponent_intersection_1;
      
      //* There are no intersections. This should never happen, but if it does, the opponent is ignored
      else continue;

      opponents_interesections.push_back(std::make_tuple(opponent, opponent_intersection));
    }
  }

  //* Compute the opponent weight and target
  //* Opponent weight: mean of the inverse of the distance from the opponents
  //* Opponents target: mean of all opponents interesections   
  float total_opponents_weight = 0;
  GlobalVector2f opponents_target = GlobalVector2f(0.f, 0.f);
  if(opponents_interesections.size()>0){
    for(auto opponent_intersection:opponents_interesections){
      float opponent_weight = 1/(center - std::get<0>(opponent_intersection)).norm();
      total_opponents_weight += opponent_weight;
      opponents_target += std::get<1>(opponent_intersection)*opponent_weight;
    }
  }

  //* Compute the field limit weight and target
  //* Field limit weight: mean of the inverse of the distance from the field limits
  //* Field limit target: mean of all field border intersections
  float field_limit_weight = 0;
  GlobalVector2f field_limit_target = GlobalVector2f(0.f, 0.f);
  int num_of_field_limits = 0;
  for(int i=0; i<field_border_intersections_bool.size(); i++){
    if(field_border_intersections_bool.at(i)){

      //* target_temp: weighted mean of goal intersection and opponents target (i.e. considering only goal and opponents)
      GlobalVector2f target_temp = (goal_weight*goal_intersection + opponents_target)/(goal_weight+total_opponents_weight);

      GlobalVector2f intersection1;
      GlobalVector2f intersection2;
      GlobalVector2f intersection;
      int num_of_interesections;
      if(i == 0) num_of_interesections = Geometry::getIntersectionOfLineAndCircle(right_field_border, circle, intersection1, intersection2);
      if(i == 1) num_of_interesections = Geometry::getIntersectionOfLineAndCircle(left_field_border, circle, intersection1, intersection2);
      if(i == 2) num_of_interesections = Geometry::getIntersectionOfLineAndCircle(opponent_goal_line, circle, intersection1, intersection2);
      if(i == 3) num_of_interesections = Geometry::getIntersectionOfLineAndCircle(own_goal_line, circle, intersection1, intersection2);
      
      //* Choose the intersection closest to the goal
      if(num_of_interesections == 2)      intersection = (goal_intersection-intersection1).norm() < (goal_intersection-intersection2).norm() ? intersection1 : intersection2;
      else if(num_of_interesections == 1) intersection = intersection1;
      else                                intersection = goal_intersection;
      
      //* angle between the field border intersection and the goal intersection
      float angle_of_intersection = theLibMisc.angleBetweenGlobalVectors(center, goal_intersection, intersection);
      if(angle_of_intersection<0.05) angle_of_intersection = 0;
      
      //* angle between the field border intersection and the target_temp
      float angle_of_temp_target = theLibMisc.angleBetweenGlobalVectors(center, goal_intersection, target_temp);
      if(angle_of_temp_target<0.05) angle_of_temp_target = 0;

      //* ratio: it is a measure of how much I want to go straight to the goal intersection:
      //* if the angle of the intersection is 0, the ratio is 0 (no influence of the field limit)
      //* if the angle of temp_target is 0, the ratio is 0 (no influence of the opponents on the target, so there is no need to consider the field limit (already to the goal))
      //* otherwise the closer is the center to the field limit, the higher the ratio, and
      //* the closer is the opponent to the goal intersection (i.e. the larger is the angle of the temp target), the higher the ratio
      float ratio = 0;
      if(angle_of_intersection!=0) ratio = (angle_of_temp_target/angle_of_intersection)*8;
      
      field_limit_weight += (1/(center - field_border_limits.at(i)).norm()) * ratio;
      if(std::isinf(field_limit_weight)) field_limit_weight = 10000;
      field_limit_target += field_border_intersections.at(i);
      num_of_field_limits++;
    }
  }
  if(num_of_field_limits>0) field_limit_target /= num_of_field_limits;
  field_limit_target = field_limit_target*field_limit_weight;

  //** Target: weighted mean of goal intersection and opponents target
  target = (goal_weight*goal_intersection + opponents_target + field_limit_target)/(goal_weight+total_opponents_weight+field_limit_weight);

  //** Clip the target to the circle
  Geometry::Line to_target = Geometry::Line(center, target-center);
  GlobalVector2f target_intersection_1;
  GlobalVector2f target_intersection_2;
  num_of_interesections = Geometry::getIntersectionOfLineAndCircle(to_target, circle, target_intersection_1, target_intersection_2);
  if(num_of_interesections == 2){
    target = (target_intersection_1 - target).norm() < (target_intersection_2 - target).norm() ? target_intersection_1 : target_intersection_2;
  }
  else if (num_of_interesections == 1){
    target = target_intersection_1;
  }
  else{
    target = goal_intersection;
  }

  //** Clip the target to the limit line (perpendicular to the line to the line that connects the ball and the opponent goal)
  // Using theLibMisc.angleBetweenGlobalVectors()
  // c+ s+ | c+ s+  
  // c- s+ | c- s+
  float cos_angle = cos(theLibMisc.angleBetweenGlobalVectors(center, goal_intersection, target));
  if(cos_angle<0){
    if(limit_intersection_1.x() > limit_intersection_2.x()) 
      target = limit_intersection_1;
    else 
      target = limit_intersection_2;
  }
  

  //* Compute the target sector and check if it is free (i.e. there are no opponents inside the sector)
  float angle_of_target = atan2(target.y()-center.y(), target.x()-center.x());
  float sector_angle = 15*M_PI/180; 
  GlobalVector2f left_target_bound = GlobalVector2f(center.x() + 2*steplen*cos(angle_of_target+sector_angle), center.y() + 2*steplen*sin(angle_of_target+sector_angle));
  GlobalVector2f right_target_bound = GlobalVector2f(center.x() + 2*steplen*cos(angle_of_target-sector_angle), center.y() + 2*steplen*sin(angle_of_target-sector_angle));

  bool is_target_sector_free = true;
  for(auto opp_sector:opponents_sectors){
    GlobalVector2f opponent = std::get<0>(opp_sector);
    GlobalVector2f left_opp_bound = std::get<1>(opp_sector);
    GlobalVector2f right_opp_bound = std::get<2>(opp_sector);
    if(theLibMisc.isInsideGlobalSector(center, left_target_bound, right_target_bound, 0.0, 2*steplen, left_opp_bound) ||
       theLibMisc.isInsideGlobalSector(center, left_target_bound, right_target_bound, 0.0, 2*steplen, right_opp_bound) || 
       theLibMisc.isInsideGlobalSector(center, left_target_bound, right_target_bound, 0.0, 2*steplen, opponent)){
      is_target_sector_free = false;
      break;
    }
  }

  //* If the target sector is not free, scan the semi-circle in front of me to find a free target point
  if(!is_target_sector_free && !field_border_intersections.size()>0){
    
    float right_angle = theLibMisc.radiansToDegree(theLibMisc.angleBetweenGlobalVectors(center, target, right_limit));
    float left_angle = theLibMisc.radiansToDegree(theLibMisc.angleBetweenGlobalVectors(center, target, left_limit));
    float target_angle = theLibMisc.radiansToDegree(atan2(target.y()-center.y(), target.x()-center.x()));

    //* Scan left first or right first based on the opponents distribution 
    bool scan_left_first = true; 
    float left_side_weight = 0.f;
    float right_side_weight = 0.f;
    int num_opponents_on_left = 0;
    int num_opponents_on_right = 0;
    for(auto opp_sector:opponents_sectors){
      GlobalVector2f opponent = std::get<0>(opp_sector);
      GlobalVector2f left_opp_bound = std::get<1>(opp_sector);
      GlobalVector2f right_opp_bound = std::get<2>(opp_sector);
      Geometry::Line to_goal = Geometry::Line(center, goal_intersection-center);
      if(Geometry::isPointLeftOfLine(center, goal_intersection, opponent)){
        left_side_weight += 1/std::abs(Geometry::getDistanceToLine(to_goal, opponent)) * 1/(center-opponent).norm();
        num_opponents_on_left++;
      }
      else{
        right_side_weight += 1/std::abs(Geometry::getDistanceToLine(to_goal, opponent)) * 1/(center-opponent).norm();
        num_opponents_on_right++;
      }
    }
    if(left_side_weight < right_side_weight) scan_left_first = true;
    else scan_left_first = false;
    
    float first_angle = scan_left_first ? left_angle : right_angle;
    float second_angle = scan_left_first ? right_angle : left_angle;
     
    bool found_target = false;

    for(int i=0; i<int(first_angle) && !found_target; i+=5){
      float point_angle = scan_left_first ? target_angle*M_PI/180 + i*M_PI/180 : target_angle*M_PI/180 - i*M_PI/180;
      GlobalVector2f point = GlobalVector2f(center.x() + steplen*cos(point_angle), center.y() + steplen*sin(point_angle));
      GlobalVector2f left_point = GlobalVector2f(center.x() + steplen*cos(point_angle+sector_angle), center.y() + steplen*sin(point_angle+sector_angle));
      GlobalVector2f right_point = GlobalVector2f(center.x() + steplen*cos(point_angle-sector_angle), center.y() + steplen*sin(point_angle-sector_angle));

      bool is_valid = true;
      for(auto opp_sector:opponents_sectors){
        GlobalVector2f opponent = std::get<0>(opp_sector);
        GlobalVector2f left_opp_bound = std::get<1>(opp_sector);
        GlobalVector2f right_opp_bound = std::get<2>(opp_sector);
        if(theLibMisc.isInsideGlobalSector(center, left_opp_bound, right_opp_bound, 0.0, 2*steplen, left_point) ||
           theLibMisc.isInsideGlobalSector(center, left_opp_bound, right_opp_bound, 0.0, 2*steplen, right_point) ||
           theLibMisc.isInsideGlobalSector(center, left_opp_bound, right_opp_bound, 0.0, 2*steplen, point)){
          is_valid = false;
          break;
        }
      }
      if(is_valid){
        target = point;
        found_target = true;
        break;
      }
    }

    for(int i=0; i<int(second_angle) && !found_target; i+=5){
      float point_angle = scan_left_first ? target_angle*M_PI/180 - i*M_PI/180 : target_angle*M_PI/180 + i*M_PI/180;
      GlobalVector2f point = GlobalVector2f(center.x() + steplen*cos(point_angle), center.y() + steplen*sin(point_angle));
      GlobalVector2f left_point = GlobalVector2f(center.x() + steplen*cos(point_angle+sector_angle), center.y() + steplen*sin(point_angle+sector_angle));
      GlobalVector2f right_point = GlobalVector2f(center.x() + steplen*cos(point_angle-sector_angle), center.y() + steplen*sin(point_angle-sector_angle));

      bool is_valid = true;
      for(auto opp_sector:opponents_sectors){
        GlobalVector2f opponent = std::get<0>(opp_sector);
        GlobalVector2f left_opp_bound = std::get<1>(opp_sector);
        GlobalVector2f right_opp_bound = std::get<2>(opp_sector);
        if(theLibMisc.isInsideGlobalSector(center, left_opp_bound, right_opp_bound, 0.0, 2*steplen, left_point) ||
           theLibMisc.isInsideGlobalSector(center, left_opp_bound, right_opp_bound, 0.0, 2*steplen, right_point) ||
           theLibMisc.isInsideGlobalSector(center, left_opp_bound, right_opp_bound, 0.0, 2*steplen, point)){
          is_valid = false;
          break;
        }
      }
      if(is_valid){
        target = point;
        found_target = true;
        break;
      }
    }
  }

  //* Clip the target to the field borders
  bool clip_to_field = Geometry::clipPointInsideRectangle(Vector2i(right_own_corner.x(), right_own_corner.y()), Vector2i(left_opponent_corner.x(), left_opponent_corner.y()), target);
  
  // Only for debug drawing
  angle_of_target = atan2(target.y()-center.y(), target.x()-center.x()); 
  left_target_bound = GlobalVector2f(center.x() + 2*steplen*cos(angle_of_target+sector_angle), center.y() + 2*steplen*sin(angle_of_target+sector_angle));
  right_target_bound = GlobalVector2f(center.x() + 2*steplen*cos(angle_of_target-sector_angle), center.y() + 2*steplen*sin(angle_of_target-sector_angle));


  //** BASIC DEBUG DRAWING
  //* Circle of raduis steplen
  CIRCLE3D("module:LibStrikerProvider:strikerDribblePoint", circle.center.x(), circle.center.y(), 0.f, circle.radius, 5.f, ColorRGBA::gray);
  
  //* Limit line
  LINE3D("module:LibStrikerProvider:strikerDribblePoint", limit_intersection_1.x(), limit_intersection_1.y(), 0.f, limit_intersection_2.x(), limit_intersection_2.y(), 0.f, 2.5, ColorRGBA::gray);
  
  //* Sector
  LINE3D("module:LibStrikerProvider:strikerDribblePoint", center.x(), center.y(), 0.f, left_target_bound.x(), left_target_bound.y(), 0.f, 2.5, ColorRGBA::gray);
  LINE3D("module:LibStrikerProvider:strikerDribblePoint", center.x(), center.y(), 0.f, right_target_bound.x(), right_target_bound.y(), 0.f, 2.5, ColorRGBA::gray);
  
  //* Target
  CYLINDER3D("module:LibStrikerProvider:strikerDribblePoint", target.x(), target.y(), 0.f, 0.f, 0.f, 0.f, 30, 1, ColorRGBA::yellow);
  LINE3D("module:LibStrikerProvider:strikerDribblePoint", center.x(), center.y(), 0.f, target.x(), target.y(), 0.f, 2.5, ColorRGBA::yellow);
  
  //** VERY VERBOSE DEBUG DRAWING
  //* Line to goal and goal intersection
  CYLINDER3D("module:LibStrikerProvider:strikerDribblePointVerbose", goal_intersection.x(), goal_intersection.y(), 0.f, 0.f, 0.f, 0.f, 15, 1, ColorRGBA::green);
  LINE3D("module:LibStrikerProvider:strikerDribblePointVerbose", center.x(), center.y(), 0.f, opponent_goal.x(), opponent_goal.y(), 0.f, 2.5, ColorRGBA::green);

  //* Lines to field borders
  for(int i = 0; i < field_border_limits.size(); i++){
    if(field_border_intersections_bool.at(i)){
      GlobalVector2f limit = field_border_limits.at(i);
      CYLINDER3D("module:LibStrikerProvider:strikerDribblePointVerbose", limit.x(), limit.y(), 0.f, 0.f, 0.f, 0.f, 30, 1, ColorRGBA::brown);
      LINE3D("module:LibStrikerProvider:strikerDribblePointVerbose", center.x(), center.y(), 0.f, limit.x(), limit.y(), 0.f, 2.5, ColorRGBA::brown);
    }
  }
  //* Lines to field borders intersections
  for(int i = 0; i < field_border_intersections.size(); i++){
    if(field_border_intersections_bool.at(i)){
      GlobalVector2f intersection = field_border_intersections.at(i);
      CYLINDER3D("module:LibStrikerProvider:strikerDribblePointVerbose", intersection.x(), intersection.y(), 0.f, 0.f, 0.f, 0.f, 30, 1, ColorRGBA::violet);
      LINE3D("module:LibStrikerProvider:strikerDribblePointVerbose", center.x(), center.y(), 0.f, intersection.x(), intersection.y(), 0.f, 2.5, ColorRGBA::violet);
    }
  }
  
  //* Lines to opponents and their intersections
  for(auto opponent_sector:opponents_sectors){
    GlobalVector2f opponent = std::get<0>(opponent_sector);
    GlobalVector2f intersection_1 = std::get<1>(opponent_sector);
    GlobalVector2f intersection_2 = std::get<2>(opponent_sector);
    CYLINDER3D("module:LibStrikerProvider:strikerDribblePointVerbose", opponent.x(), opponent.y(), 0.f, 0.f, 0.f, 0.f, 15, 1, ColorRGBA::red);
    CYLINDER3D("module:LibStrikerProvider:strikerDribblePointVerbose", intersection_1.x(), intersection_1.y(), 0.f, 0.f, 0.f, 0.f, 5, 1, ColorRGBA::red);
    CYLINDER3D("module:LibStrikerProvider:strikerDribblePointVerbose", intersection_2.x(), intersection_2.y(), 0.f, 0.f, 0.f, 0.f, 5, 1, ColorRGBA::red);
    LINE3D("module:LibStrikerProvider:strikerDribblePointVerbose", center.x(), center.y(), 0.f, intersection_1.x(), intersection_1.y(), 0.f, 2.5, ColorRGBA::red);
    LINE3D("module:LibStrikerProvider:strikerDribblePointVerbose", center.x(), center.y(), 0.f, intersection_2.x(), intersection_2.y(), 0.f, 2.5, ColorRGBA::red);
  }
  for(auto circle:opponent_circles){
    CIRCLE3D("module:LibStrikerProvider:strikerDribblePointVerbose", circle.center.x(), circle.center.y(), 0.f, circle.radius, 2.5, ColorRGBA::red);
  }
  for(auto opponents_intersections:opponents_interesections){
    GlobalVector2f intersection = std::get<1>(opponents_intersections);
    CYLINDER3D("module:LibStrikerProvider:strikerDribblePointVerbose", intersection.x(), intersection.y(), 0.f, 0.f, 0.f, 0.f, 15, 1, ColorRGBA::blue);
    LINE3D("module:LibStrikerProvider:strikerDribblePointVerbose", center.x(), center.y(), 0.f, intersection.x(), intersection.y(), 0.f, 2.5, ColorRGBA::blue);
  }

  return target.hasNaN() ? goal_intersection : target;
}

bool LibStrikerProvider::shouldKick(bool currentlyKicking, float kickIntervalThreshold) const {
  // currentlyKicking: true if the robot is in GoToBallAndKick state, false if in GoToBallAndDribble state
  // is used to avoid stop kicking when the robot is already kicking

  // Comportamento intorno ai pali
  Vector2f ball_position = theFieldBall.recentBallPositionOnField();
  Vector2f left_rect_bottom_left_corner = Vector2f(theFieldDimensions.xPosOpponentGroundLine - 120, theFieldDimensions.yPosLeftSideline);
  Vector2f left_rec_top_right_corner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal+100);
  Vector2f right_rect_bottom_left_corner = Vector2f(theFieldDimensions.xPosOpponentGroundLine - 120, theFieldDimensions.yPosRightGoal-100);
  Vector2f right_rec_top_right_corner = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline);
  if(Geometry::isPointInsideRectangle(left_rect_bottom_left_corner, left_rec_top_right_corner, ball_position) ||
     Geometry::isPointInsideRectangle(right_rect_bottom_left_corner, right_rec_top_right_corner, ball_position)) return false;
  
  int numberOfOpponentsInFrontOfMe = 0;
  for(auto obs:theObstacleModel.obstacles){
    if(obs.type == Obstacle::opponent){
      if(obs.center.x() > -150 && obs.center.norm() < 2000)
        numberOfOpponentsInFrontOfMe++;
    }
  }
  int numberOfOpponentsInOurHalf = 0;
  for(auto obs:theTeamPlayersModel.obstacles){
    if(obs.type == Obstacle::opponent){
      if(obs.center.x() < 0)
        numberOfOpponentsInOurHalf++;
    }
  }

  // interval: length of the free area (from FreeGoalTargetableArea.h)
  float interval = goalTargetWithArea(false, false).second.interval;

  // always kick if the robot is near to opponent goal and there are too many obstacles to strategize
  if (theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyArea - (currentlyKicking ? 800.f : 0.f) &&
      numberOfOpponentsInFrontOfMe > (currentlyKicking ? 2 : 3))
    return true;

  // kick if the robot has a good angle to the goal and the ball is seen enough in the last 60 frames (seenPercentage), 
  // and the robot is not too far from the goal (max 4 meters)
  else if (interval                     > (currentlyKicking ? -1 : kickIntervalThreshold) &&
           theRobotPose.translation.x() > (currentlyKicking ? 0.f : 800.f) && // TODO (must be tuned with the real robots) considering that the penalty area starts at 2850
           theBallModel.seenPercentage  > (currentlyKicking ? 0 : 40)) 
    return true; 

  // kick if the robot is near to our goal and there are too many opponents in our half (spazzata dalla difesa)
  else if (theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundLine / 2 + (currentlyKicking ? 400.f : 0.f) &&
            numberOfOpponentsInOurHalf   > 1)
    return true;

  else
    return false;
}

KickInfo::KickType LibStrikerProvider::getKick(bool kickAsap, bool kickRight) const{
  //Rangef 
  std::array<Rangef, 3> rgs = {Rangef(0.f, 1000.f), Rangef(1000.f, INFINITY), Rangef(INFINITY, INFINITY)};

  // The first element of the pair is the case of kickAsap false, the second one is kickAsap true
  std::array<std::pair<KickInfo::KickType,KickInfo::KickType>, 3> kicksRight {std::make_pair(KickInfo::forwardFastRight, KickInfo::walkForwardsRight), std::make_pair(KickInfo::forwardFastRightLong, KickInfo::walkForwardsRightLong), std::make_pair(KickInfo::ottoMetriRightKick,KickInfo::ottoMetriLooseRightKick)};
  std::array<std::pair<KickInfo::KickType,KickInfo::KickType>, 3> kicksLeft {std::make_pair(KickInfo::forwardFastLeft, KickInfo::walkForwardsLeft), std::make_pair(KickInfo::forwardFastLeftLong, KickInfo::walkForwardsLeftLong), std::make_pair(KickInfo::ottoMetriLeftKick,KickInfo::ottoMetriLooseLeftKick)};

  ASSERT(rgs.size() == kicksRight.size());
  ASSERT(kicksLeft.size() == kicksRight.size());

  std::array<std::pair<KickInfo::KickType,KickInfo::KickType>, 3> kicks = kickRight ? kicksRight : kicksLeft;
  Vector2f distVector = theRobotPose.translation - Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0);
  float dist = distVector.norm();

  for(int i = 0; i < rgs.size(); ++i)
    if(rgs[i].isInside(dist))
      return kickAsap ? kicks[i].second : kicks[i].first;
  
  OUTPUT_ERROR("getKick function failed, no suitable kick found");
}


KickInfo::KickType LibStrikerProvider::getWalkKick(GlobalVector2f target) const {
  
  /**  KICKTYPES:
   * * Walk Kick Types:
   * 
   * * KickType                                     | rotation [deg] | range [mm] | execution time [ms] | real range [mm] (2024)
   *   --------------------------------------------------------------------------------------------------------------------------
   *   walkForwardsRight/Left                       | 0              | 400-1000   | 600                 |
   *   walkForwardsRightLong/LeftLong               | 0              | 2500-3500  | 350                 |
   *   walkSidewardsRightFootToRight/LeftFootToLeft | 90/-90         | 2000       | 400                 |
   *   walkTurnRightFootToLeft/LeftFootToRight      | -45/45 deg     | 400-2100   | 600                 |
   *   walkForwardStealBallRight/Left               | -90/90 deg     | 200        | 0                   |
   *   walkForwardsRightAlternative/LeftAlternative | 3/-3 deg       | 2100       | 600                 |
  */

  float distance = (theRobotPose.inversePose * target).norm();
  float angle = std::abs(theLibMisc.radiansToDegree((theRobotPose.inversePose * target).angle()));  
  LocalVector2f ball_relative = theFieldBall.recentBallPositionRelative();

  // compute left or right foot
  bool left = false;
  if(ball_relative.y() > 0) left = true;

  // LONG_DISTANCES: 2000-3500
  // NOTE: walkForwardsLeftLong/RightLong have a range of 2500-3500, so this condition is a bit forced
  if (distance>=2000 && distance<3500) {
    if (left) return KickInfo::KickType::walkForwardsLeftLong;
    else return KickInfo::KickType::walkForwardsRightLong;
  }

  // SHORT_DISTANCES: 400-2000
  // TODO: this should be splitted into 400-1000 (short) and 1000-2000 (mid), but for now we don't have a walkkick with range 1000-2000
  else{

    // Forward: 400-1000
    if (angle<=50){
      if (left) return KickInfo::KickType::walkForwardsLeft;
      else return KickInfo::KickType::walkForwardsRight;
    }

    // Forward-Turn: 400-2100
    else if (angle>50 && angle<=80 && ball_relative.norm() < 500){
      if (left) return KickInfo::KickType::walkTurnRightFootToLeft;
      else return KickInfo::KickType::walkTurnLeftFootToRight;
    }

    // Sideward: 2000
    else if (angle>80 && angle<=135 && ball_relative.norm() < 500){
      if (left) return KickInfo::KickType::walkSidewardsLeftFootToLeft;
      else return KickInfo::KickType::walkSidewardsRightFootToRight;
    }

    // Backward
    else {
      // TODO: Colpo di tacco !! :D
      if (left) return KickInfo::KickType::walkForwardsLeft;
      else return KickInfo::KickType::walkForwardsRight;
    }
  }
  
  // default
  if (left) return KickInfo::KickType::walkForwardsLeft;
  else return KickInfo::KickType::walkForwardsLeft;
};

GlobalVector2f LibStrikerProvider::getStrikerPositionSpecial() const{
  GlobalVector2f target = Vector2f(0.f, 0.f);
  GlobalVector2f ball = theFieldBall.recentBallPositionOnField();
  switch(theGameState.state){
    
    // Own free kicks
    case GameState::State::ownCorner:{
      target = ball;
      break;
    }
    case GameState::State::ownKickIn:{
      target = ball;
      break;
    }
    case GameState::State::ownPenaltyKick:{
      target = ball;
      break;
    }
    case GameState::State::ownPushingKick:{
      target = ball;
      break;
    }
    case GameState::State::ownGoalKick:{
      target = (theLibSpec.isGoaliePlaying()) ? Vector2f(-150.f, 0.f) : ball;
      break;
    }
    
    // Opponents free kicks
    case GameState::State::opponentCorner:{
      target = (ball.y() > 0) ? //left corner
                GlobalVector2f( (theFieldDimensions.xPosOwnGoalArea+theFieldDimensions.xPosOwnPenaltyMark)/2 , 1300) :
                GlobalVector2f( (theFieldDimensions.xPosOwnGoalArea+theFieldDimensions.xPosOwnPenaltyMark)/2 , -1300);
      break;
    }
    case GameState::State::opponentKickIn:{
      if(ball.y() > 0) target = GlobalVector2f(ball.x() - 900.f, ball.y() - 300.f);
      else target = GlobalVector2f(ball.x() - 900.f, ball.y() + 300.f);
      if(target.x() < - 4000.f) target = GlobalVector2f(-4100.f, ball.y() > 0 ? 2100 : -2100); 
      break;
    }
    case GameState::State::opponentPushingKick:{
      Line2 ball_goal = Line2::Through(ball, GlobalVector2f(theFieldDimensions.xPosOwnGroundLine, 0.f));
      Line2 parallel = Line2::Through(ball - GlobalVector2f(800.f, 0.f), ball - GlobalVector2f(800.f, 300.f));
      target = ball_goal.intersection(parallel);
      break;
    }
    case GameState::State::opponentGoalKick:{
      target = ball - GlobalVector2f(1000.f, 0.f);
      break;
    }
    
    // TODO: add the case of the penalty kick
    case GameState::State::opponentPenaltyKick:
      break;
  }
  return target.hasNaN() ? GlobalVector2f(0.f,0.f) : target;

}

GlobalVector2f LibStrikerProvider::getStrikerPosition() const{
  GlobalVector2f target;
  GlobalVector2f ball = theFieldBall.recentBallPositionOnField();

  // Free kick
  if(theGameState.state == GameState::State::ownCorner          ||
     theGameState.state == GameState::State::ownGoalKick        ||
     theGameState.state == GameState::State::ownKickIn          ||
     theGameState.state == GameState::State::ownPushingKick     ||
     theGameState.state == GameState::State::ownPenaltyKick     ||
     
     theGameState.state == GameState::State::opponentCorner     ||
     theGameState.state == GameState::State::opponentGoalKick   ||
     theGameState.state == GameState::State::opponentKickIn     ||
     theGameState.state == GameState::State::opponentPushingKick){
    target = getStrikerPositionSpecial();
  }
  
  // Normal play
  else{
    target = ball;
  }

  return target.hasNaN() ? GlobalVector2f(0.f,0.f) : target;
}
