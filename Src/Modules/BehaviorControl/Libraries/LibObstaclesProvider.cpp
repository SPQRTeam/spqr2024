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
  libObstacles.nearestOpponentToPoint = [this](const GlobalVector2f& point) -> GlobalPose2f {
    return nearestOpponentToPoint(point);
  };
}

GlobalPose2f LibObstaclesProvider::nearestOpponentToPoint(const GlobalVector2f& p) const {
  LocalVector2f lPoint(theLibMisc.glob2Rel(p.x(), p.y()).translation);

  auto criterion = [](const Obstacle& a, const Obstacle& b) {
    return a.center.norm() < b.center.norm() && a.type == Obstacle::opponent && b.type == Obstacle::opponent;
  };

  auto min = std::min_element(theObstacleModel.obstacles.begin(), theObstacleModel.obstacles.end(), criterion);

  if(min != theObstacleModel.obstacles.end())
    return theLibMisc.rel2Glob_v(min->center);
  else
    return InvalidGlobalPose2f;
}


Pose2f LibObstaclesProvider::nearestOpponent() const {
  Pose2f nearest = InvalidGlobalPose2f;
  for(const auto& obs : theObstacleModel.obstacles){
    if( (obs.center.norm() < nearest.translation.norm()) && obs.type == Obstacle::opponent) {
      nearest = obs.center;
    }
  }
  return nearest;
}

Pose2f LibObstaclesProvider::nearestOpponentWithFilter(std::function<bool(Obstacle)> filterPredicate) const {
  Pose2f nearest = InvalidGlobalPose2f;
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
    if(obs.type == Obstacle::Type::opponent && (point-obs.center).norm()<=radius)
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
    if(obs.type == Obstacle::Type::teammate && (point-obs.center).norm()<=radius)
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

