/**
 * @file LibGoalieProvider.h
 * 
 * See LibGoalie
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/spqr_representations/ConfigurationParameters.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibGoalie.h"
#include "Tools/Module/Module.h"

MODULE(LibGoalieProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(BallModel),
  REQUIRES(LibPosition),
  REQUIRES(LibMisc),
  PROVIDES(LibGoalie),
  LOADS_PARAMETERS(
  {,
    (float) kickAwayTolerance,
  }),
});

class LibGoalieProvider : public LibGoalieProviderBase
{
public:
  // Constructor
  LibGoalieProvider();

private:
  
  /**
   * Updates LibGoalie
   * @param libGoalie The representation provided
   */
  void update(LibGoalie& libGoalie) override;


  // ===== IMPLEMENTATIONS OF LibGoalie =====

  // no functions (yet?)



  // ===== FOR INTERNAL USE =====

  /**
   * <no doc was available, guessing>
   * Calculates the reference position for the goalie.
   */
  Vector2f updateGoalie() const;

  // provides the homonymous parameter of LibGoalie
  bool isGoalieInStartingPosition() const;

  // provides the homonymous parameter of LibGoalie
  bool isGoalieInAngle() const;

  // provides the homonymous parameter of LibGoalie
  bool isBallInKickAwayRange() const;

  // provides the homonymous parameter of LibGoalie
  bool isGoalieInKickAwayRange() const;
};
