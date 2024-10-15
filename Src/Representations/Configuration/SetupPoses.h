/**
 * @file SetupPoses.h
 *
 * Declaration of a representation that contains information about
 * the pose from which the robots enter the pitch when the game state
 * switches from INITIAL to READY.
 *
 * @author Tim Laue
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

/**
 * @struct SetupPoses
 * A representation that contains a list of poses from which the robots enter
 * the pitch when the game state switches from INITIAL to READY.
 */
STREAMABLE(SetupPoses,
{
  /** The pose of a robot before entering the field */
  STREAMABLE(SetupPose,
  {,
    (int) playerNumber,           /*< The player number of the robot */
    (Vector2f) position,          /*< The position (in global field coordinates) at which the robot is placed */
    (Vector2f) turnedTowards,     /*< The position (in global field coordinates) at which the robot is turned (looking at) */
  });

  /** Convenience function to find the correct pose given the player number.
   *  The list of poses is not ordered by numbers.
   *  It has to be made sure that the config file contains the entry. Otherwise -> ASSERT!
   *  Exception (for demos and tests): If the list has only one entry, this entry is returned, no matter which number the robot has.
   *  @param number The player number (starting with 1, as the real number)
   *  @return A reference to the pose for setup
   */
  const SetupPose& getPoseOfRobot(int number, bool weAreAttacking) const,

  (std::vector<SetupPose>) attackingPoses, /*< A list of all available robot poses, not ordered by number */
  (std::vector<SetupPose>) defendingPoses,
});

inline const SetupPoses::SetupPose& SetupPoses::getPoseOfRobot(int number, bool weAreAttacking) const
{
  if(weAreAttacking){
    for(const auto& pose : attackingPoses)
      if(pose.playerNumber == number)
        return pose;
    return attackingPoses[0]; // Dummy line to avoid compiler complaints
  } else {
    for(const auto& pose : defendingPoses)
      if(pose.playerNumber == number)
        return pose;
    return defendingPoses[0]; // Dummy line to avoid compiler complaints
  }
  FAIL("No setup pose for player " << number << ".");
}