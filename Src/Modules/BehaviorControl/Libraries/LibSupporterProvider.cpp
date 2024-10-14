/**
 * @file LibSupporterProvider.cpp
 * 
 * This file implements a module that computes the supporter position.
 */

#include "LibSupporterProvider.h"
#include <iostream>


MAKE_MODULE(LibSupporterProvider, behaviorControl);

void LibSupporterProvider::update(LibSupporter& libSupporter)
{
  libSupporter.supporterPosition = getSupporterPosition();

  DECLARE_DEBUG_DRAWING3D("module:LibSupporterProvider:supporterPosition", "field");
  if(thePlayerRole.role == PlayerRole::supporter){
    CYLINDER3D("module:LibSupporterProvider:supporterPosition", libSupporter.supporterPosition.x(), libSupporter.supporterPosition.y(), 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 20.0f, ColorRGBA::orange);
  }
}

GlobalVector2f LibSupporterProvider::getSupporterPosition() const {
  // Set the target position to the ball position - 1500 on the x axis
  GlobalVector2f ball_position = theFieldBall.recentBallPositionOnField();
  GlobalVector2f target = GlobalVector2f(ball_position.x() - 1500.f, ball_position.y());

  /** Clip the target position to the field boundaries to avoid the robot to go out of the field
   * - The x axis is clipped to -1200 and 0
   * - The y axis is clipped to the field boundaries -500/+500
   */
  if(target.x() < -1200.f) target = GlobalVector2f(-1200.f, target.y());
  else if(target.x() > 0.f) target = GlobalVector2f(0.f, target.y());
  if (target.y() < theFieldDimensions.yPosRightSideline + 500) target = GlobalVector2f(target.x(), theFieldDimensions.yPosRightSideline + 500);
  else if (target.y() > theFieldDimensions.yPosLeftSideline - 500) target = GlobalVector2f(target.x(), theFieldDimensions.yPosLeftSideline - 500);

  return target.hasNaN() ? GlobalVector2f(1000.f,1000.f) : target;
}
