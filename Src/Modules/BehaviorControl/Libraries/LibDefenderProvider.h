/**
 * @file LibDefenderProvider.h
 * 
 * This file defines a module that computes the defenders position.
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/spqr_representations/GameState.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"

using Line2 = Eigen::Hyperplane<float,2>;


MODULE(LibDefenderProvider,
{,
  REQUIRES(LibMisc),
  REQUIRES(LibSpec),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(GameState),
  USES(PlayerRole),
  PROVIDES(LibDefender),
  // LOADS_PARAMETERS(
  // {,
  // }),
});

class LibDefenderProvider : public LibDefenderProviderBase
{
private:
  
  /**
   * @brief Updates the LibDefender representation
   * 
   * @param libDefender The LibDefender representation to update
   */
  void update(LibDefender& libDefender) override;

  /**
   * @brief Computes the defenderone position (global coordinates)
   * The defenderone reference position is based on the 
   * intersection between the line from the ball to the goal and 
   * a semi-circle around the own goal
   * 
   * @return [GlobalVector2f] The defenderone position
   */
  GlobalVector2f getDefenderonePosition() const;
  
  /**
   * @brief Computes the defenderone position (global coordinates) in special cases
   * 
   * @return [GlobalVector2f] The defenderone position in special cases
   */
  GlobalVector2f getDefenderonePositionSpecial() const;

  /**
   * @brief Computes the defendertwo position (global coordinates)
   * The defendertwo reference position is based on the 
   * intersection between the line from the ball to the goal and 
   * a semi-circle around the own goal
   * 
   * @return [GlobalVector2f] The defendertwo position
   */
  GlobalVector2f getDefendertwoPosition() const;
  
  /**
   * @brief Computes the defendertwo position (global coordinates) in special cases
   * 
   * @return [GlobalVector2f] The defendertwo position in special cases
   */
  GlobalVector2f getDefendertwoPositionSpecial() const;

  /**
   * @brief Returns the target point on the semi-circle
   * 
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
   * @param segmentStart The start point of the segment
   * @param segmentEnd The end point of the segment
   * @param minRadius The minimum radius of the semi-circle
   * @param maxRadius The maximum radius of the semi-circle
   * 
   * @return [GlobalVector2f] The target point on the semi-circle
   */
  GlobalVector2f targetOnSemiCircle(GlobalVector2f segmentStart, GlobalVector2f segmentEnd, float minRadius, float maxRadius) const;
};
