/**
 * @file LibMiscProvider.cpp
 * 
 * See LibMisc
 *
 * @author Francesco Petri
 */

#include "LibMiscProvider.h"

MAKE_MODULE(LibMiscProvider, behaviorControl);

void LibMiscProvider::update(LibMisc& libMisc)
{

  DECLARE_DEBUG_DRAWING3D("module:LibMiscProvider:clipTargetOutsideObstacles", "field");

  libMisc.localDirectionToField = [this](const Vector2f& localDirection) {
    return localDirectionToField(localDirection);
  };
  libMisc.distance = [this](const Pose2f& p1, const Pose2f& p2) {
    return distance(p1, p2);
  };
  libMisc.distanceVec = [this](const Vector2f& p1, const Vector2f& p2) {
    return distanceVec(p1, p2);
  };
  libMisc.mapToInterval = [this](float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) -> float {
    return mapToInterval(value, fromIntervalMin, fromIntervalMax, toIntervalMin, toIntervalMax);
  };
  libMisc.isValueBalanced = [this](float currentValue, float target, float bound) -> bool {
    return isValueBalanced(currentValue, target, bound);
  };
  libMisc.angleToTarget = [this](float x, float y) -> float {
    return angleToTarget(x, y);
  };
  libMisc.glob2Rel = [this](float x, float y) -> Pose2f {
    return glob2Rel(x, y);
  };
  libMisc.rel2Glob = [this](float x, float y) -> Pose2f {
    return rel2Glob(x, y);
  };
  libMisc.norm = [this](float x, float y) -> float {
    return norm(x, y);
  };
  libMisc.radiansToDegree = [this](float x) -> float {
    return radiansToDegree(x);
  };
  libMisc.angleBetweenPoints = [this](const Vector2f& p1, const Vector2f& p2) -> float {
    return angleBetweenPoints(p1, p2);
  };
  libMisc.angleBetweenGlobalVectors = [this](const Vector2f& start, const Vector2f& end1, const Vector2f& end2) -> float {
    return angleBetweenGlobalVectors(start, end1, end2);
  };
  libMisc.isInsideGlobalSector = [this](const Vector2f startBounds, const Vector2f endBound1, const Vector2f endBound2, float startRadius, float endRadius, const Vector2f point) -> bool {
    return isInsideGlobalSector(startBounds, endBound1, endBound2, startRadius, endRadius, point);
  };
  libMisc.isInsideGlobalRectangle = [this](const Vector2f point, const Vector2f bottom, const Vector2f left, const Vector2f right, const Vector2f up) -> bool {
    return isInsideGlobalRectangle(point, bottom, left, right, up);
  };
  libMisc.clipTargetOutsideObstacles = [this](const Vector2f& target, const float& radius, const bool& consider_teammate_as_obstacles) -> Vector2f {
    return clipTargetOutsideObstacles(target, radius, consider_teammate_as_obstacles);
  };
  libMisc.calcAngleToTarget = [this](const Vector2f& target) -> Angle {
    return calcAngleToTarget(target);
  };
  libMisc.getMirrorAngle = [this](Vector2f A, Vector2f B, Vector2f C) -> Angle {
    return getMirrorAngle(A, B, C);
  };

  libMisc.angleToGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f)).angle();
  libMisc.angleToBall = (theRobotPose.inversePose * theFieldBall.positionOnField).angle();
}


int LibMiscProvider::localDirectionToField(const Vector2f& localDirection) const {
  Vector2f globalDirection = localDirection.rotated(theRobotPose.rotation);
  if (globalDirection.x() >= 0) return 1;
  else return -1;
}

float LibMiscProvider::distance(const Pose2f& p1, const Pose2f& p2) const {
  return static_cast<float>( std::sqrt( std::pow(p2.translation.x() - p1.translation.x(), 2) +
    std::pow(p2.translation.y() - p1.translation.y(), 2) ) );
}

float LibMiscProvider::distanceVec(const Vector2f& p1, const Vector2f& p2) const {     //TODO: Flavio&Valerio
  return static_cast<float>( std::sqrt( std::pow(p2.x() - p1.x(), 2) +
    std::pow(p2.y() - p1.y(), 2) ) );
}

float LibMiscProvider::mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) const {
  float fromIntervalSize = fromIntervalMax - fromIntervalMin;
  float toIntervalSize = toIntervalMax - toIntervalMin;
  if(value > fromIntervalMax) return toIntervalMax;
  else if (value<fromIntervalMin) return toIntervalMin;
  else return toIntervalMin + (value - fromIntervalMin) * toIntervalSize / fromIntervalSize;
}

bool LibMiscProvider::isValueBalanced(float currentValue, float target, float bound) const {
  float minErr = currentValue - (target - bound);
  float maxErr = currentValue - (target + bound);

  if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
    return true;
  else
    return false;
}

float LibMiscProvider::angleToTarget(float x, float y) const
{
  Pose2f relativePosition = glob2Rel(x,y);
  return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
}

Pose2f LibMiscProvider::glob2Rel(float x, float y) const
{
  Vector2f result;
  float theta = 0;
  float tempX = x - theRobotPose.translation.x();
  float tempY = y - theRobotPose.translation.y();

  result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
  result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

  return Pose2f(theta /*deg*/, result.x(),result.y());
}

Pose2f LibMiscProvider::rel2Glob(float x, float y) const
{
  Vector2f result;
  float rho = (float)(sqrt((x * x) + (y * y)));

  result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
  result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

  return Pose2f(result.x(),result.y());
}

float LibMiscProvider::norm(float x, float y) const
{
  return sqrtf((x*x) + (y*y));
};

float LibMiscProvider::radiansToDegree(float x) const
{
  return (float)((x*180)/3.14159265358979323846);
}

// [torch, 2022] I do not understand why deltaY is noy p2-p1 like it *feels* like it should be.
//               Am I missing something obvious?
//               Did the author forcefully enforce some nonstandard convention?
//               Anyway, this is used in a couple modules, so I can't change this at the moment.
//               Future users, make sure this is really what you want.
float LibMiscProvider::angleBetweenPoints(const Vector2f& p1, const Vector2f& p2) const {
  float deltaY = p1.y() - p2.y();
  float deltaX = p2.x() - p1.x();
  return atan2f(deltaY, deltaX);
}

float LibMiscProvider::angleBetweenVectors(const Vector2f& v1, const Vector2f& v2) const {
  return atan2f( v2.y()*v1.x() - v2.x()*v1.y(), v2.x()*v1.x() + v2.y()*v1.y());
}

float LibMiscProvider::angleBetweenGlobalVectors(const Vector2f& start, const Vector2f& end1, const Vector2f& end2) const {
  Vector2f v1 = Vector2f(end1.x() - start.x(), end1.y()-start.y());
  Vector2f v2 = Vector2f(end2.x() - start.x(), end2.y()-start.y());
  float cos_theta = ((v1.x()*v2.x())+(v1.y()*v2.y()))/(v1.norm()*v2.norm());
  float theta = acos(cos_theta);
  if(std::isnan(theta)) {
    return 0;
  }
  return theta;
}

bool LibMiscProvider::isInsideGlobalSector(const Vector2f startBounds, const Vector2f endBound1, const Vector2f endBound2, float startRadius, float endRadius, const Vector2f point) const{
  float radius_point = distanceVec(startBounds, point);
  if(radius_point < startRadius || radius_point > endRadius)
    return false;

  float angle_bound_1 = atan2f(endBound1.y() - startBounds.y(), endBound1.x() - startBounds.x());
  float angle_bound_2 = atan2f(endBound2.y() - startBounds.y(), endBound2.x() - startBounds.x());
  float angle_point = atan2f(point.y() - startBounds.y(), point.x() - startBounds.x());
  if(angle_bound_1 <= angle_bound_2){
    if(angle_point > angle_bound_1 && angle_point < angle_bound_2)
      return true;
    else 
      return false;
  } 
  else{
    if(angle_point > angle_bound_2 && angle_point < angle_bound_1)
      return true;
    else
      return false;
  }
}  

bool LibMiscProvider::isInsideGlobalRectangle(const Vector2f point, const Vector2f bottom, const Vector2f left, const Vector2f right, const Vector2f up) const{
  if(point.x() >= bottom.x() && point.x()<=up.x() && point.y() <= left.y() && point.y() >= right.y()){
    return true;
  }
  return false;
}

Vector2f LibMiscProvider::clipTargetOutsideObstacles(const Vector2f& target, const float& radius, const bool& consider_teammate_as_obstacles) const{  

  float cell_size = 200.f;
  std::vector<std::vector<std::pair<Vector2f, bool>>> matrix;
  for (int i = 0; i <= 8; i++){
    std::vector<std::pair<Vector2f, bool>> row;
    for (int j = 0; j <= 8; j++){
      Vector2f new_vector = Vector2f(target.x() + (4-i)*cell_size, target.y() + (4-j)*cell_size);
      row.push_back(std::make_pair(new_vector, true));
    }
    matrix.push_back(row);
  }

  for(int i=0; i<8; i++){
    for(int j=0; j<8; j++){
      for(auto obs:theTeamPlayersModel.obstacles){
        if(obs.type == Obstacle::Type::opponent || obs.type == Obstacle::Type::fallenOpponent || 
           (consider_teammate_as_obstacles && (obs.type == Obstacle::Type::teammate || obs.type == Obstacle::Type::fallenTeammate))) {  
          
          Vector2f obstacle = obs.center;
          Vector2f upper_left = matrix[i][j].first;
          Vector2f lower_right = matrix[i+1][j+1].first;

          if(obstacle.x() < upper_left.x() && obstacle.x() > lower_right.x() && obstacle.y() < upper_left.y() && obstacle.y() > lower_right.y()){
            matrix[i][j].second = false;
            for (int k = i-1; k <= i+1; k++){
              for (int l = j-1; l <= j+1; l++){
                if(k>=0 && k<=8 && l>=0 && l<=8){
                  matrix[k][l].second = false;
                }
              }
            }
          }

        }
      }
    }
  }

  if(matrix[3][3].second && matrix[3][4].second && matrix[4][3].second && matrix[4][4].second) return target;
  
  else{
    int i = 0;
    int j = 0;
    float min_distance = (matrix[0][0].first - target).norm();
    for(int k=0; k<8; k++){
      for(int l=0; l<8; l++){
        bool is_free = true;
        if(matrix[k][l].second == false || 
          (k>0 && l>0 && matrix[k-1][l-1].second == false) ||
          (k>0 && matrix[k-1][l].second == false) ||
          (l>0 && matrix[k][l-1].second == false)) is_free = false;

      
        if(is_free){
          float distance = (matrix[k][l].first - target).norm();
          if(distance < min_distance){
            min_distance = distance;
            i = k;
            j = l;
          }
        }
      }
    }

    return matrix[i][j].first;
  }

  return target;
}
Angle LibMiscProvider::calcAngleToTarget(Vector2f target) const
{
  return (theRobotPose.inversePose * target).angle();
}

Angle LibMiscProvider::getMirrorAngle(Vector2f A, Vector2f B, Vector2f C){

  Vector2f BA = A - B;
  Vector2f BC = C - B;
  // Normalize the direction vectors
  Vector2f BA_norm = (BA)/BA.norm();
  Vector2f BC_norm = (BC)/BC.norm();
  
  // Calculate the bisector vector
  Vector2f bisector = BA_norm + BC_norm;
  bisector = bisector / bisector.norm();
  
  // Calculate the point D at the desired distance
  Vector2f D = B + bisector;
  POINT3D("punto", D[0], D[1], 2.f, 15.f, ColorRGBA::red);

  Angle angle = calcAngleToTarget(D);

  return angle;
}