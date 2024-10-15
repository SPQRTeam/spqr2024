/**
 * @file LibSupporterProvider.h
 * 
 * See LibSupporter
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSupporter.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MODULE(LibSupporterProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamData),
  REQUIRES(RobotInfo),
  USES(PlayerRole),
  REQUIRES(FieldBall),
  REQUIRES(LibObstacles),
  REQUIRES(LibMisc),
  REQUIRES(FieldDimensions),
  PROVIDES(LibSupporter),
});

class LibSupporterProvider : public LibSupporterProviderBase
{
private:
  
  /**
   * Updates LibSupporter
   * @param libSupporter The representation provided
   */
  void update(LibSupporter& libSupporter) override;

  /** 
   * Provides the supporter reference position
   */
  Vector2f getSupporterPosition() const;
  
};
