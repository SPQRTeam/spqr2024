/**
 * @file Representations/Modeling/FreeGoalTargetableArea.cpp
 * 
 * Implementation of the FreeGoalTargetableArea's verification function
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "FreeGoalTargetableArea.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"


void FreeGoalTargetableArea::verify() const
{
  ASSERT(std::isfinite(begin));
  ASSERT(std::isfinite(end));
  ASSERT(std::isfinite(midpoint));
  ASSERT(interval>=0.);
  ASSERT(std::isfinite(value));
}