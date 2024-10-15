/**
 * @file LibDefenderProvider.h
 * 
 * See LibDefender
 *
 * @author @neverorfrog
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/spqr_representations/GameState.h"



using Line2 = Eigen::Hyperplane<float,2>;


MODULE(LibDefenderProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamBallModel),
  REQUIRES(FieldBall),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(GameState),
  USES(LibSpec),
  USES(LibStriker),
  USES(LibMisc),
  USES(PlayerRole),

  PROVIDES(LibDefender),
  // LOADS_PARAMETERS(
  // {,
  //   // (float) defenderoneMinRadius,
  //   // (float) defenderoneMaxRadius,
  //   // (float) defendertwoMinRadius,
  //   // (float) defendertwoMaxRadius, 
  // }),
});

class LibDefenderProvider : public LibDefenderProviderBase
{
private:
  
  /**
   * Updates LibDefender
   * @param libDefender The representation provided
   */
  void update(LibDefender& libDefender) override;


  // ===== IMPLEMENTATIONS OF LibDefender =====

  /** 
   * Checks some boolean conditions that return true if the defender is the one indicated to intercept
   */
  // bool hasToIntercept(interceptTarget) const;

  /** 
   * <no doc was available, this is a guess>
   * Returns the ideal y for the defender to block the most direct shot
   * from the ball to the goal.
   * TODO this is likely to end up unused, check after porting
   */
  float defenderDynamicY() const;     // this gets the libDefender parameter b/c it needs to read something


  /** 
   * Provides the defenderone reference position based on 
   * intersection between of the center of 
   * the ball and the center of the goal
   */
  Vector2f getDefenderonePosition() const;
  Vector2f getDefenderonePositionSpecial() const;


  /** 
   * Provides the defender reference position based on 
   * the position of the ball and the movements of the teams
   */
  Vector2f getDefendertwoPosition() const;
  Vector2f getDefendertwoPositionSpecial() const;

  /**
   * Computes the two Intersections of line and circle
   * solves the following system of equations and selecting the one inside the field:
   *
   * (x - x_c)^2 + (y - y_c)^2 = r^2
   * p + l * d = [x, y]
   *
   * where [x_m, y_m] is the center of the circle,
   * p is line.base and v is line.direction and
   * [x, y] is an intersection point.
   * 
   */
  Vector2f targetOnSemiCircle(Vector2f segmentStart, Vector2f segmentEnd, float minRadius, float maxRadius) const;
};
