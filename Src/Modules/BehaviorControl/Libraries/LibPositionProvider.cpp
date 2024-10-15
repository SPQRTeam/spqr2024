/**
 * @file LibPositionProvider.cpp
 * @author Martin Kroker
 * @author Andreas Stolpmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "LibPositionProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <iostream>

MAKE_MODULE(LibPositionProvider, behaviorControl);

// [torch, 2022] I like this style: use duplicate internal functions for everything and keep the update simple.
// Let's do more of that!

void LibPositionProvider::update(LibPosition& libPosition)
{
  DECLARE_DEBUG_DRAWING("behavior:LibPositionProvider:obstacleAtMyPosition", "drawingOnField");

  libPosition.distanceToOwnGoalGreaterThan = [this](float distance) -> bool
  {
    return distanceToOwnGoalGreaterThan(distance);
  };
  libPosition.isInOwnGoalArea = [this](const Vector2f& position)
  {
    return isInOwnGoalArea(position);
  };
  libPosition.isNearOwnGoalArea = [this](const Vector2f& position, float toleranceX, float toleranceY) -> bool
  {
    return isNearOwnGoalArea(position, toleranceX, toleranceY);
  };
  libPosition.isInOwnPenaltyArea = [this](const Vector2f& position) -> bool
  {
    return isInOwnPenaltyArea(position);
  };
  libPosition.isNearOwnPenaltyArea = [this](const Vector2f& position, float toleranceX, float toleranceY) -> bool
  {
    return isNearOwnPenaltyArea(position, toleranceX, toleranceY);
  };
  libPosition.isInOpponentPenaltyArea = [this](const Vector2f& position) -> bool
  {
    return isInOpponentPenaltyArea(position);
  };
  libPosition.isNearOpponentPenaltyArea = [this](const Vector2f& position, float toleranceX, float toleranceY) -> bool
  {
    return isNearOpponentPenaltyArea(position, toleranceX, toleranceY);
  };
  libPosition.isOutSideGoalFrame = [this](const Vector2f& position, const float offset) -> bool
  {
    return isOutSideGoalFrame(position, offset);
  };
  libPosition.getObstacleAtMyPositionCircle = [this](const Vector2f& pos) -> Geometry::Circle
  {
    return getObstacleAtMyPositionCircle(pos);
  };

  libPosition.myReadyPosition = [this]() -> Pose2f {
    return myReadyPosition();
  };
  libPosition.isPositionInsideRectangle = [this](const Vector2f& position, const Vector2f& corner1, const Vector2f& corner2) -> bool {
    return isPositionInsideRectangle(position, corner1, corner2);
  };
}

bool LibPositionProvider::distanceToOwnGoalGreaterThan(float distance) const
{
  const Vector2f midPoint(theFieldDimensions.xPosOwnGroundLine, 0.f);
  float distanceToGoal = (theRobotPose.translation - midPoint).norm();
  return distanceToGoal > distance;
}

bool LibPositionProvider::isNearOwnGoalArea(const Vector2f& position, float toleranceX, float toleranceY) const
{
  return std::abs(position.y()) <= theFieldDimensions.yPosLeftGoalArea + toleranceY &&
         position.x() <= theFieldDimensions.xPosOwnGoalArea + toleranceX;
}

bool LibPositionProvider::isInOwnGoalArea(const Vector2f& position) const
{
  return isNearOwnGoalArea(position, 0.f, 0.f);
}

bool LibPositionProvider::isNearOwnPenaltyArea(const Vector2f& position, float toleranceX, float toleranceY) const
{
  return std::abs(position.y()) <= theFieldDimensions.yPosLeftPenaltyArea + toleranceY &&
         position.x() <= theFieldDimensions.xPosOwnPenaltyArea + toleranceX;
}

bool LibPositionProvider::isInOwnPenaltyArea(const Vector2f& position) const
{
  return isNearOwnPenaltyArea(position, 0.f, 0.f);
}

bool LibPositionProvider::isNearOpponentPenaltyArea(const Vector2f& position, float toleranceX, float toleranceY) const
{
  return std::abs(position.y()) <= theFieldDimensions.yPosLeftPenaltyArea + toleranceY &&
         position.x() >= theFieldDimensions.xPosOpponentPenaltyArea - toleranceX;
}

bool LibPositionProvider::isInOpponentPenaltyArea(const Vector2f& position) const
{
  return isNearOpponentPenaltyArea(position, 0.f, 0.f);
}

bool LibPositionProvider::isOutSideGoalFrame(const Vector2f& position, const float offset) const
{
  return position.x() - theFieldDimensions.xPosOwnGroundLine > offset || std::abs(position.y()) - theFieldDimensions.yPosLeftGoal > offset;
}

Geometry::Circle LibPositionProvider::getObstacleAtMyPositionCircle(const Vector2f& position)
{
  CIRCLE("behavior:LibPositionProvider:obstacleAtMyPosition", lastCircle.center.x(), lastCircle.center.y(), lastCircle.radius, 20, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA::black);
  const Vector2f posRel(theRobotPose.inversePose * position);
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
    if((obstacle.center - posRel).squaredNorm() < sqr(positionOffsetIfOccupied))
      return lastCircle = Geometry::Circle(theRobotPose * obstacle.center, positionOffsetIfOccupied);

  if((lastCircle.center - position).squaredNorm() < sqr(lastCircle.radius) &&
     std::abs((theRobotPose.inversePose * lastCircle.center).angle()) > deleteObstacleCircleRange)
    return lastCircle;

  lastCircle.radius = 0.f;
  return lastCircle;
}


Pose2f LibPositionProvider::myReadyPosition() const {

  Pose2f strikerPose{}; //4
  Pose2f goaliePose = Pose2f(0.f, -4250.f, 0.f); //1
  Pose2f defenderonePose = Pose2f(0.f, theFieldDimensions.xPosOwnPenaltyMark, 400.f); //2
  Pose2f defendertwoPose = Pose2f(0.f, theFieldDimensions.xPosOwnPenaltyMark, -400.f); //5
  Pose2f supporterPose{}; //6
  Pose2f receiverPose{}; //3
  Pose2f liberoPose{}; //7
  
  if(theGameInfo.setPlay == SET_PLAY_PENALTY_KICK && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){ // penalty for us
    strikerPose = Pose2f(0.f, theFieldDimensions.xPosOpponentPenaltyMark - 500.f, 0.f);
    supporterPose = Pose2f(0.f, 0.f, 0.f);
    receiverPose = Pose2f(0.f, 2000.f, -1000.f);
    liberoPose = Pose2f(0.f, 2000.f, 1000.f);
  }
  // TODO: change these positions for opponent penalty kick
  else if(theGameInfo.setPlay == SET_PLAY_PENALTY_KICK && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber){ // penalty for opponent
    strikerPose = Pose2f(0.f, theFieldDimensions.xPosOwnPenaltyArea + 400.f, -2000.f);
    defenderonePose = Pose2f(0.f, theFieldDimensions.xPosOwnPenaltyArea + 800.f, 600.f); //2
    defendertwoPose = Pose2f(0.f, theFieldDimensions.xPosOwnPenaltyArea + 800.f, -600.f); //5

    supporterPose = Pose2f(0.f, -1000.f, 1000.f);
    receiverPose = Pose2f(0.f, 0.f, 1500.f);
    liberoPose = Pose2f(0.f, 0.f, -1500.f);
  }

  else{ // kickoff 
    strikerPose = Pose2f(0.f, -1000.f, 0.f);
    supporterPose = Pose2f(0.f, -1500.f, 0.f);
    receiverPose = Pose2f(0.f, -500.f, -2000.f);
    liberoPose = Pose2f(0.f, -500.f, 2000.f);   
  }
  
  /*
  else{ // kickoff for opponent
    strikerPose = Pose2f(0.f, -1000.f, -250.f);
    supporterPose = Pose2f(0.f, -2000.f, 1500.f);
    receiverPose = Pose2f(0.f, -1500.f, -1500.f);
    liberoPose = Pose2f(0.f, -2000.f, 250.f);
  }
  */


  Pose2f target{};

  if(theRobotInfo.number == 1)
    target = goaliePose;
  else
  {
    //ordered ready poses and numbers
    std::vector<Pose2f> readyPoses{strikerPose, defenderonePose, defendertwoPose, supporterPose, receiverPose, liberoPose};
    std::vector<int> readyPlayerNums{4, 2, 5, 6, 7, 3}; //ordered by priority

    //ap = [2,3,7,6]
    //ip = [4,5]

    //active and inactive player number arrays
    std::vector<int> activePlayers{};
    std::vector<int> inactivePlayers = {};
    for(auto i : readyPlayerNums){
      bool isPenalized{};
      for(auto const& mate : theTeamData.teammates){
        if(mate.number == i){
          isPenalized = mate.isPenalized;
          break;
        } 
      }
      if(i == theRobotInfo.number || !isPenalized || theGameInfo.state == STATE_STANDBY)
        activePlayers.push_back(i);
      else{
        inactivePlayers.push_back(i); //a non existing robot is given as penalized
        //OUTPUT_TEXT("I'm " << theRobotInfo.number << " and " << i << " is inactive");
      }
    }
    const int numActivePlayers = activePlayers.size();
    
    //Role i have to take on
    auto it = std::find(readyPlayerNums.begin(), readyPlayerNums.end(), theRobotInfo.number);
    int index = it - readyPlayerNums.begin();

    //This means there are penalized players and i have to substitute a more importante role than mine
    int numberToSub{}; //number whose role i take on
    if (index >= numActivePlayers){
      int inactiveIndex = 0;
      for(int activeIndex = activePlayers.size()-1; activeIndex >= 0; activeIndex--){
        if(activePlayers[activeIndex] == theRobotInfo.number){
          numberToSub = inactivePlayers[inactiveIndex]; break;
        }
        inactiveIndex++;
      }
      //OUTPUT_TEXT("I'm " << theRobotInfo.number << " and I substitute " << numberToSub);
      it = std::find(readyPlayerNums.begin(), readyPlayerNums.end(), numberToSub);
      index = it - readyPlayerNums.begin();
    }

    target =  readyPoses[index];
  }
  return target; 

}

bool LibPositionProvider::isPositionInsideRectangle(const Vector2f& position, const Vector2f& corner1, const Vector2f& corner2) const {
  Vector2f min_point = Vector2f(std::min(corner1.x(), corner2.x()), std::min(corner1.y(), corner2.y()));
  Vector2f max_point = Vector2f(std::max(corner1.x(), corner2.x()), std::max(corner1.y(), corner2.y()));

  return position.x() > min_point.x() && position.x() < max_point.x() && position.y() > min_point.y() && position.y() < max_point.y();
}
