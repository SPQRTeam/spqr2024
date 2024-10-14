/**
 * @file LibTeam.h
 *
 * This file defines a representation that contains information about the team
 *
 * @author Daniel Krause
 */

#pragma once
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Tools/Function.h"

STREAMABLE(LibTeam,
{
  /*
    =====================================================================================================================
    THE PREEXISTING STUFF
    =====================================================================================================================
  */

  /** returns the pose of a team mate in field coordinates
      @player player number of team mate */
  FUNCTION(Pose2f(int player)) getTeammatePose;

  /** Returns the activity of the specified player */
  FUNCTION(BehaviorStatus::Activity(int player)) getActivity;

  /** Returns the status of the specified player */
  FUNCTION(Teammate::Status(int player)) getStatus;

  /** Returns the ball position estimate of the specified player */
  FUNCTION(Vector2f(int player)) getBallPosition;

  /** Returns the time to reach ball of the specified player. */
  FUNCTION(const TimeToReachBall*(int player)) getTimeToReachBall,

  (int) keeperPlayerNumber,
  (int) strikerPlayerNumber, //player number of striker or -1 if striker was not found
  (Pose2f) keeperPose, //the pose of the keeper
  (Pose2f) strikerPose, // the pose of the striker
  (bool) iAmClosestToBall,
  (float) minTeammateDistanceToBall,
  (int) numberOfNonKeeperTeammateInOwnGoalArea, /**< The number of the non keeper teammate in the goal area or -1 if there is none*/
  (int) numberOfBallPlayingTeammate,            /**< The number of the striker or keeper, depending on whether the keeper plays the ball. */

  /*
    =====================================================================================================================
    OUR STUFF
    =====================================================================================================================
  */

  /**
   * Time since the team last saw the ball, in milliseconds.
   * NOTE: The one provided by FieldBall is locally for this robot.
   *       Plus, there seems to be a belief that
   *       FieldBall is not updated under some rare, unknown circumstances.
   *       Is this still true in this new version of the framework? Who knows!
   */
  (int) timeSinceBallWasSeen,
});
