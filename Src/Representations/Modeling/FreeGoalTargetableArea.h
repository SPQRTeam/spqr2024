/**
 * @file Representations/Modeling/FreeGoalTargetableArea.h
 *
 * Declaration of struct FreeGoalTargetableArea, that holds informations about a single
 * discretized area on the goal line available as a target
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct FreeGoalTargetableArea
 *
 * Represents a free (from opponents) targetable area on the goal line
 */

STREAMABLE(FreeGoalTargetableArea,
{

  /**< Default constructor */
  FreeGoalTargetableArea() = default;

  /** Constructor with field initialization
   * @param begin Global y coord of the left limit of the free area
   * @param end Global y coord of the left limit of the free area
   * @param midpoint Global y coord of the middle of the free area
   * @param interval Length of the free area
   * @param areaValueHeuristic Heuristic function used to evaluate target area value
   */
  FreeGoalTargetableArea(float begin, float end, float value);

  /** Verifies that the model contains valid values. */
  void verify() const,

  //void draw() const, 

  (float)(0.0) begin,    /** Global y coord of the left limit of the free area*/
  (float)(0.0) end,      /** Global y coord of the right limit of the free area*/
  (float)(0.0) midpoint, /** Global y coord of the middle of the free area*/
  (float)(0.0) interval, /** Length of the free area*/
  (float)(0.0) value,    /** The evaluated value of targeting this area based on an arbitrary heuristic*/
});

//Constructor
inline FreeGoalTargetableArea::FreeGoalTargetableArea(float begin, float end, float value):
begin(begin),
end(end),
midpoint(begin-(begin-end)/2),
interval(std::abs(begin-end)),
value(value) {}