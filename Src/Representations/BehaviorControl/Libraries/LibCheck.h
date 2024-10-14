/**
 * @file LibCheck.h
 *
 * This file defines a representation that checks some behavior control properties
 *
 * @author Daniel Krause
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Function.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(LibCheck,
{
  /*
    =====================================================================================================================
    THE PREEXISTING STUFF
    =====================================================================================================================
  */
  ENUM(CheckedOutput,
  {,
    motionRequest,
    headMotionRequest,
    activity,
    passTarget,
    firstTeamCheckedOutput,
    teamActivity = firstTeamCheckedOutput,
    timeToReachBall,
    teammateRoles,
    role,
  });

  /** Increments one counter */
  FUNCTION(void(LibCheck::CheckedOutput outputToCheck)) inc;

  /** Indicates that an arm has been set */
  FUNCTION(void(Arms::Arm arm)) setArm;

  /** Checks whether an arm has been set */
  FUNCTION(bool(Arms::Arm arm)) wasSetArm;

  /** Performs checks for the individual behavior */
  FUNCTION(void(const MotionRequest& theMotionRequest)) performCheck;

  /** Performs checks for the team behavior */
  FUNCTION(void()) performTeamCheck;
  
  /*
    =====================================================================================================================
    OUR STUFF
    =====================================================================================================================
  */
  
  /** Returns -1 if the ball is in own goal, 1 if the ball is in the opponent goal, 0 if it isn't in any goal
   * NOTE: Check out the BallInGoal representation and study it.
   *       It seems to hold the time since the last goal (so if it's 0 or very small the ball is in a goal)
   *       and whether that goal was in our own goal or the opponents'
   *       If it works as expected, it may supplant this function...
   * NOTE: Check out also the LibPosition functions.
  */
  FUNCTION(int()) whichGoalIsTheBallIn,
});
