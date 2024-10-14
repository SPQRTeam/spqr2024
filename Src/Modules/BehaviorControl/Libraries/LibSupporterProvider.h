/**
 * @file LibSupporterProvider.h
 * 
 * This file defines a module that computes the supporter position.
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibSupporter.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MODULE(LibSupporterProvider,
{,
  REQUIRES(LibMisc),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(FieldBall),
  USES(PlayerRole),
  PROVIDES(LibSupporter),
});

class LibSupporterProvider : public LibSupporterProviderBase
{
private:

  /**
   * @brief Updates the LibSupporter representation
   * 
   * @param libSupporter The LibSupporter representation to update
   */
  void update(LibSupporter& libSupporter) override;

  /**
   * @brief Computes the supporter position (global coordinates)
   * 
   * @return [GlobalVector2f] The supporter position
   */
  GlobalVector2f getSupporterPosition() const;

};
