/**
 * @file LibSupporterProvider.cpp
 * 
 * See LibSupporter
 *
 * @author Francesco Petri
 */

#include "LibSupporterProvider.h"
#include <iostream>


MAKE_MODULE(LibSupporterProvider, behaviorControl);

void LibSupporterProvider::update(LibSupporter& libSupporter)
{
  DECLARE_DEBUG_DRAWING3D("module:LibSupporterProvider:supporterPosition", "field");

  libSupporter.getSupporterPosition = [this]() -> Vector2f {
    return getSupporterPosition();
  };

}

Vector2f LibSupporterProvider::getSupporterPosition() const {
  Vector2f ball_position = theFieldBall.recentBallPositionOnField();
  Vector2f target = Vector2f(ball_position.x() - 1500.f, ball_position.y());

  //* Clip the target position to the field boundaries -500 to avoid the robot to go out of the field
  if(target.x() < -1200.f) target = Vector2f(-1200.f, target.y());
  else if(target.x() > 0.f) target = Vector2f(0.f, target.y());
  if (target.y() < theFieldDimensions.yPosRightSideline + 500) target = Vector2f(target.x(), theFieldDimensions.yPosRightSideline + 500);
  else if (target.y() > theFieldDimensions.yPosLeftSideline - 500) target = Vector2f(target.x(), theFieldDimensions.yPosLeftSideline - 500);

  if(thePlayerRole.role == PlayerRole::supporter){  
    CYLINDER3D("module:LibSupporterProvider:supporterPosition", target.x(), target.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 20.0f, ColorRGBA::orange);
  }
  return target.hasNaN() ? Vector2f(1000.f,1000.f) : target;
}
