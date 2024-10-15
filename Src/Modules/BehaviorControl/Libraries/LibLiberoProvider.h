/**
 * @file LibLiberoProvider.h
 * 
 * See LibLibero
 *
 * @author Flavio Volpi
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibLibero.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/Communication/GameInfo.h"   
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/spqr_representations/Voronoi.h"
#include "Representations/spqr_representations/GameState.h"
#include "Platform/File.h"
#include "Modules/spqr_modules/ContextCoordinator/newCoordinator.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/SpqrTools/Graph.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"



MODULE(LibLiberoProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamData),
  REQUIRES(LibMisc),
  REQUIRES(LibPass),
  REQUIRES(FieldBall),
  REQUIRES(LibSpec),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(Voronoi),
  PROVIDES(LibLibero),

  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibLiberoProvider : public LibLiberoProviderBase
{
private:
  spqr::UndirectedGraph liberoPositionGraph, 
                        centralPositionGraph;

  Vector2f* current_target;
  float hysteresis_utility = 0.1;
  float hysteresis_time = 10000;
  unsigned last_time_modified_node = theFrameInfo.time;

  /**
   * Updates LibLibero
   * @param libLibero The representation provided
   */
  void update(LibLibero& libLibero) override;

  /** 
   * Provides the libero reference position
   */
  Vector2f getLiberoPosition();
  Vector2f getLiberoPositionGraph();
  Vector2f getLiberoPositionSpecial();

public:
  LibLiberoProvider();
};
