/**
 * @file LibTeamProvider.cpp
 * @author Lukas Malte Monerjahn
 */

#include "LibTeammatesProvider.h"

MAKE_MODULE(LibTeammatesProvider, behaviorControl);

void LibTeammatesProvider::update(LibTeammates& libTeammates)
{
  libTeammates.nonKeeperTeammatesInOwnPenaltyArea = nonKeeperTeammatesInOwnPenaltyArea();
  libTeammates.teammatesInOpponentPenaltyArea = teammatesInOpponentPenaltyArea();
  libTeammates.nearestTeammate = [this]() -> Pose2f {
    return nearestTeammate();
  };
}

int LibTeammatesProvider::nonKeeperTeammatesInOwnPenaltyArea() const
{
  float distanceThreshold = outsideDistanceThreshold;
  bool isNear = false;
  if(theLibPosition.isNearOwnPenaltyArea(theRobotPose.translation, -theRobotPose.getXAxisStandardDeviation(), -theRobotPose.getYAxisStandardDeviation()))
    distanceThreshold = insideDistanceThreshold;
  else if(theLibPosition.isNearOwnPenaltyArea(theRobotPose.translation, outsideDistanceThreshold, outsideDistanceThreshold))
    isNear = true;
  int teammatesInPenaltyArea = 0;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED
       && !teammate.isGoalkeeper
       && theLibPosition.isNearOwnPenaltyArea(teammate.theRobotPose.translation, distanceThreshold, distanceThreshold))
    {
      if(isNear && !theLibPosition.isNearOwnPenaltyArea(teammate.theRobotPose.translation, insideDistanceThreshold, insideDistanceThreshold))
      {
        /// breaks ties between two robots attempting to enter simultaneously
        if(teammate.rolePlaysTheBall())
          ++teammatesInPenaltyArea;
      }
      else
        ++teammatesInPenaltyArea;
    }
  }
  return teammatesInPenaltyArea;
}

int LibTeammatesProvider::teammatesInOpponentPenaltyArea() const
{
  float distanceThreshold = outsideDistanceThreshold;
  if(theLibPosition.isNearOpponentPenaltyArea(theRobotPose.translation, -theRobotPose.getXAxisStandardDeviation(), -theRobotPose.getYAxisStandardDeviation()))
    distanceThreshold = insideDistanceThreshold;
  int teammatesInPenaltyArea = 0;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED
       && theLibPosition.isNearOpponentPenaltyArea(teammate.theRobotPose.translation, distanceThreshold, distanceThreshold))
    {
      ++teammatesInPenaltyArea;
    }
  }
  return teammatesInPenaltyArea;
}

Pose2f LibTeammatesProvider::nearestTeammate() const
{
  Pose2f nearest = Pose2f(theRobotPose.translation + Vector2f(10000.f,10000.f));
  for(const auto& mate : theTeamData.teammates){
    if( (mate.theRobotPose.translation - theRobotPose.translation).norm() < (nearest.translation - theRobotPose.translation).norm() ) {
      nearest = mate.theRobotPose;
    }
  }
  return nearest;
};
