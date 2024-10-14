/**
 * @file LibJollyProvider.h
 * 
 * This file defines a module that computes the jolly position.
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibJolly.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/spqr_representations/GameState.h"   
#include "Platform/File.h"
#include "Tools/Module/Module.h"
#include "Tools/SpqrTools/Graph.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MODULE(LibJollyProvider,
{,
  REQUIRES(LibMisc),
  REQUIRES(LibPass),
  REQUIRES(FieldBall),
  REQUIRES(LibSpec),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  USES(PlayerRole),
  PROVIDES(LibJolly),
  LOADS_PARAMETERS(
  {,
    (float) hysteresis_utility, // Hysteresis for the utility of the nodes, to avoid oscillations.
    (float) hysteresis_time,    // Time to wait before changing the target again.
  }),
});

class LibJollyProvider : public LibJollyProviderBase
{
private:
  spqr::UndirectedGraph jollyPositionGraph,             // Graph of the jolly position (left on the field, looking at the opponent goal)
                        centralPositionGraph;           // Graph of the central position (center of the field)
  
  GlobalVector2f current_target;                        // Current target of the jolly (a node of the graph)
  unsigned last_time_modified_node = theFrameInfo.time; // Last time the target was changed (used for the hysteresis)
  
  /**
   * @brief Updates the LibJolly representation
   * 
   * @param libJolly The LibJolly representation to update
   */
  void update(LibJolly& libJolly) override;

  /**
   * @brief Computes the jolly position (global coordinates)
   * 
   * @return [GlobalVector2f] The jolly position
   */
  GlobalVector2f getJollyPosition();
  
  /**
   * @brief Computes the jolly position in the graph (global coordinates)
   * 
   * @return [GlobalVector2f] The jolly position in the graph
   */
  GlobalVector2f getJollyPositionGraph();
  
  /**
   * @brief Computes the jolly position in freeKick situations (global coordinates)
   * 
   * @return [GlobalVector2f] The jolly position in special cases
   */
  GlobalVector2f getJollyPositionSpecial();

public:
  LibJollyProvider();
};
