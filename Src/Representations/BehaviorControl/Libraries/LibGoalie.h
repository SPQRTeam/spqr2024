/**
 * @file LibGoalie.h
 *
 * This file defines a representation that holds some utilities (primarily) for the supporter.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibGoalie,
{,

  /** 
   * <no doc was available, guessing>
   * Holds the reference position for the goalie.
   */
  (Vector2f) goaliePosition,

  /** 
   * <no doc was available, guessing>
   * Holds whether the goalie is in a certain position,
   * determined by the SPQR parameters.
   */
  (bool) isGoalieInStartingPosition,

  /** 
   * <no doc was available, guessing>
   * Holds whether the goalie is facing roughly forward.
   */
  (bool) isGoalieInAngle,

  /** 
   * <no doc was available, guessing>
   * Holds whether the ball is within a certain distance of the goalie,
   * determined by the SPQR parameters.
   */
  (bool) isBallInKickAwayRange,

  /** 
   * <no doc was available, guessing>
   * Holds whether the ball is in a certain rectangle, with some 100 mm of tolerance.
   */
  (bool) isGoalieInKickAwayRange,

  /** 
   * <no doc was available, guessing>
   * The exact same thing as above, but w/o tolerance.
   * WARNING: DEPRECATED. isGoalieInKickAwayRange should cover its needs.
   *          Leaving this as a note for card porting.
   *          TODO: delete this at end of porting
   */
  // (bool) isBallInArea,
});
