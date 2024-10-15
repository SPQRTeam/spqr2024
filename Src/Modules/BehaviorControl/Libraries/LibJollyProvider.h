/**
 * @file LibJollyProvider.h
 * 
 * See LibJolly
 *
 * @author Francesco Petri
 */

#pragma once

#include <algorithm>
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/MessageManagement.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibJolly.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/Communication/GameInfo.h"   
#include "Representations/spqr_representations/GameState.h"   
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Modules/spqr_modules/ContextCoordinator/newCoordinator.h"
#include "Platform/File.h"
#include "Tools/Module/Module.h"
#include "Tools/SpqrTools/Graph.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MODULE(LibJollyProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamData),
  REQUIRES(LibMisc),
  REQUIRES(FieldBall),
  REQUIRES(LibSpec),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(LibStriker),
  REQUIRES(BHumanMessageOutputGenerator),
  REQUIRES(MessageManagement),
  REQUIRES(FrameInfo),
  REQUIRES(LibPass),
  REQUIRES(GameState),
  PROVIDES(LibJolly),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibJollyProvider : public LibJollyProviderBase
{
private:
  spqr::UndirectedGraph jollyPositionGraph, 
                        centralPositionGraph;
  
  Vector2f* current_target;
  float hysteresis_utility = 0.1;
  float hysteresis_time = 10000;
  unsigned last_time_modified_node = theFrameInfo.time;
  
  /**
   * Updates LibJolly
   * @param libJolly The representation provided
   */
  void update(LibJolly& libJolly) override;

  /** 
   * Provides the jolly reference position
   */
  Vector2f getJollyPosition();
  Vector2f getJollyPositionGraph();
  Vector2f getJollyPositionSpecial();

public:
  LibJollyProvider();
};
