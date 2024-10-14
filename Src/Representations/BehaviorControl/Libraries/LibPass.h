/**
 * @file LibPass.h
 *
 * This file defines a representation that holds some useful functions for passing.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibPass,
{
  /**
   * Find a free passing line to the target
   * Returns a valid target or goal center if fails
   */
  FUNCTION(Vector2f(Vector2f target, std::vector<Vector2f> mates)) findPassingLine;

  /**
   * This function orderes the team mates according to their distance w.r.t. the opponents
   * and then, starting from the most free one, tries to find a passing line
   * @return a Vector2f poiting to the passing target or the goal line if no passing is found
   */
  FUNCTION(Pose2f()) poseToPass;

  /**
   * TODO: documentazione
   */
  FUNCTION(float(Vector2f position, int timeSLR)) getPassUtility;

  /**
   * TODO: documentazione
   */
  FUNCTION(std::tuple<Vector2f, float>()) getBestPassage;

  /**
   * TODO: documentazione
   */
  FUNCTION(std::tuple<Vector2f, float>()) getBestPassageSpecial;

  /**
   * TODO: documentazione
   */
  FUNCTION(float(Vector2f position)) getInversePassUtility,

  (int) isTargetToPass,
});
