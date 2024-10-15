/**
 * @file LibObstaclesProvider.cpp
 * 
 * See LibObstacles
 *
 * @author Francesco Petri
 */

#include "LibObstaclesProvider.h"

MAKE_MODULE(LibObstaclesProvider, behaviorControl);

void LibObstaclesProvider::update(LibObstacles& libObstacles)
{
  libObstacles.nearestOpponent = [this]() -> Pose2f {
    return nearestOpponent();
  };
  /*
  libObstacles.nearestOpponentWithFilter = [this](std::function<bool(Obstacle)> filterPredicate) -> Pose2f {
    return nearestOpponentWithFilter(filterPredicate);
  };
  */
  libObstacles.areThereOpponentsNearby = [this](const Vector2f& point, float radius) -> bool {
    return areThereOpponentsNearby(point, radius);
  };
  libObstacles.areThereTeammatesNearby = [this](const Vector2f& point, float radius) -> bool {
    return areThereTeammatesNearby(point, radius);
  };
  libObstacles.opponentOnOurField = [this]() -> bool {
    return opponentOnOurField();
  };
  libObstacles.obstacleExistsAroundPoint = [this](const Vector2f& point) -> bool {
    return obstacleExistsAroundPoint(point);
  };
}


Pose2f LibObstaclesProvider::nearestOpponent() const {
  Pose2f nearest = Pose2f(9000.f,6000.f);
  for(const auto& obs : theObstacleModel.obstacles){
    if( (obs.center.norm() < nearest.translation.norm()) && obs.type == Obstacle::opponent) {
      nearest = obs.center;
    }
  }
  return nearest;
}

Pose2f LibObstaclesProvider::nearestOpponentWithFilter(std::function<bool(Obstacle)> filterPredicate) const {
  Pose2f nearest = Pose2f(9000.f,6000.f);
  for(const auto& obs : theObstacleModel.obstacles){
    if(filterPredicate(obs) && (obs.center.norm() < nearest.translation.norm()) && obs.type == Obstacle::opponent) {
      nearest = obs.center;
    }
  }
  return nearest;
}

bool LibObstaclesProvider::areThereOpponentsNearby(const Vector2f& point, float radius) const {
  bool dangerFlag = false;
  for(const auto& obs : theObstacleModel.obstacles)
  {
    if(obs.type == Obstacle::Type::opponent && theLibMisc.distance(point, obs.center)<=radius)
    {
      dangerFlag = true;
      break;
    }
  }
  return dangerFlag;
}

bool LibObstaclesProvider::areThereTeammatesNearby(const Vector2f& point, float radius) const {
  bool flag = false;
  for(const auto& obs : theObstacleModel.obstacles)
  {
    if(obs.type == Obstacle::Type::teammate && theLibMisc.distance(point, obs.center)<=radius)
    {
      flag = true;
      break;
    }
  }
  return flag;
}

bool LibObstaclesProvider::opponentOnOurField() const {
  for(auto const& obs : theTeamPlayersModel.obstacles){
    if(obs.isOpponent() && obs.center.x() < 0.f){
      return true;
    }
  }
  return false;
}

bool LibObstaclesProvider::obstacleExistsAroundPoint(const Vector2f& point) const {
  for(const auto& obstacle : theObstacleModel.obstacles){
    if((obstacle.center - point).norm() < 400.f){
        return true;
    }
  }
  return false;
}

