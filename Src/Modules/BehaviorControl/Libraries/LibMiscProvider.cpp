/**
 * @file LibMiscProvider.cpp
 * 
 * This file implements a module that holds some generic utility functions.
 */

#include "LibMiscProvider.h"

MAKE_MODULE(LibMiscProvider, behaviorControl);

void LibMiscProvider::update(LibMisc& libMisc)
{
  libMisc.isValueBalanced = [this](float currentValue, float target, float bound) -> bool {
    return isValueBalanced(currentValue, target, bound);
  };
  libMisc.glob2Rel = [this](float x, float y) -> LocalPose2f {
    return glob2Rel(x, y);
  };
  libMisc.glob2Rel_v = [this](GlobalVector2f v) -> LocalPose2f{
    return glob2Rel(v.x(), v.y());
  };
  libMisc.rel2Glob = [this](float x, float y) -> GlobalPose2f {
    return rel2Glob(x, y);
  };
  libMisc.rel2Glob_v = [this](LocalVector2f v) -> GlobalPose2f {
    return rel2Glob(v.x(), v.y());
  };
  libMisc.radiansToDegree = [this](RadAngle x) -> DegAngle {
    return radiansToDegree(x);
  };
  libMisc.angleBetweenVectors = [this](const Vector2f& v1, const Vector2f& v2) -> RadAngle {
    return angleBetweenVectors(v1, v2);
  };
  libMisc.angleBetweenGlobalVectors = [this](const GlobalVector2f& start, const GlobalVector2f& end1, const GlobalVector2f& end2) -> RadAngle {
    return angleBetweenGlobalVectors(start, end1, end2);
  };
  libMisc.isInsideGlobalSector = [this](const GlobalVector2f startBounds, const GlobalVector2f endBound1, const GlobalVector2f endBound2, float startRadius, float endRadius, const GlobalVector2f point) -> bool {
    return isInsideGlobalSector(startBounds, endBound1, endBound2, startRadius, endRadius, point);
  };
  libMisc.getMirrorAngle = [this](Vector2f A, Vector2f B, Vector2f C) -> RadAngle {
    return getMirrorAngle(A, B, C);
  };
  libMisc.angleToTarget = [this](const Vector2f& target) -> RadAngle {
    return angleToTarget(target);
  };
  
  libMisc.angleToGoal = angleToTarget(Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f));
  libMisc.angleToBall = angleToTarget(theFieldBall.positionOnField);
}

bool LibMiscProvider::isValueBalanced(float currentValue, float target, float bound) const 
{
  float minErr = currentValue - (target - bound);
  float maxErr = currentValue - (target + bound);

  if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
    return true;
  else
    return false;
}

LocalPose2f LibMiscProvider::glob2Rel(float x, float y) const
{
  Vector2f result;
  float theta = 0;
  float tempX = x - theRobotPose.translation.x();
  float tempY = y - theRobotPose.translation.y();

  result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
  result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

  return LocalPose2f(theta /*deg*/, result.x(),result.y());
}

GlobalPose2f LibMiscProvider::rel2Glob(float x, float y) const
{
  Vector2f result;
  float rho = (float)(sqrt((x * x) + (y * y)));

  result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
  result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

  return GlobalPose2f(result.x(),result.y());
}

DegAngle LibMiscProvider::radiansToDegree(RadAngle x) const
{
  return (float)((x*180)/3.14159265358979323846);
}

RadAngle LibMiscProvider::angleBetweenVectors(const Vector2f& v1, const Vector2f& v2) const 
{
  return atan2f( v2.y()*v1.x() - v2.x()*v1.y(), v2.x()*v1.x() + v2.y()*v1.y());
}

RadAngle LibMiscProvider::angleBetweenGlobalVectors(const GlobalVector2f& start, const GlobalVector2f& end1, const GlobalVector2f& end2) const 
{
  GlobalVector2f v1 = GlobalVector2f(end1.x() - start.x(), end1.y()-start.y());
  GlobalVector2f v2 = GlobalVector2f(end2.x() - start.x(), end2.y()-start.y());
  float cos_theta = ((v1.x()*v2.x())+(v1.y()*v2.y()))/(v1.norm()*v2.norm());
  RadAngle theta = acos(cos_theta);
  if(std::isnan(theta)) return 0;
  return theta;
}

bool LibMiscProvider::isInsideGlobalSector(const GlobalVector2f startBounds, const GlobalVector2f endBound1, const GlobalVector2f endBound2, float startRadius, float endRadius, const GlobalVector2f point) const{
  float radius_point = (startBounds-point).norm();
  if(radius_point < startRadius || radius_point > endRadius) return false;

  float angle_bound_1 = atan2f(endBound1.y() - startBounds.y(), endBound1.x() - startBounds.x());
  float angle_bound_2 = atan2f(endBound2.y() - startBounds.y(), endBound2.x() - startBounds.x());
  float angle_point = atan2f(point.y() - startBounds.y(), point.x() - startBounds.x());
  if(angle_bound_1 <= angle_bound_2){
    if(angle_point > angle_bound_1 && angle_point < angle_bound_2) return true;
    else return false;
  } 
  else{
    if(angle_point > angle_bound_2 && angle_point < angle_bound_1) return true;
    else return false;
  }
}  

RadAngle LibMiscProvider::getMirrorAngle(Vector2f A, Vector2f B, Vector2f C){
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

  Angle angle = angleToTarget(D);

  return angle;
}

RadAngle LibMiscProvider::angleToTarget(Vector2f target) const
{
  return (theRobotPose.inversePose * target).angle();
}