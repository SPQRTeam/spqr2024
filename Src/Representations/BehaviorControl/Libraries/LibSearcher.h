/**
 * @file LibSearcher.h
 *
 * This file defines a representation that holds some utilities (primarily) for the searcher.
 *
 * @author Daniele Affinita
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(LibSearcher,
{
  /** 
   * Provides the active searcher reference position based on the SearcherModel matrix
   */
  FUNCTION(Vector2f()) getActiveSearcherPosition,

});
