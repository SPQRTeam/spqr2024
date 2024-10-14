/**
 * @file LibLiberoProvider.h
 * 
 * This file defines a module that computes the libero position.
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibLibero.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/spqr_representations/GameState.h"
#include "Platform/File.h"
#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/SpqrTools/Graph.h"

MODULE(LibLiberoProvider,
{,
  REQUIRES(LibMisc),
  REQUIRES(LibPass),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  USES(PlayerRole), 
  PROVIDES(LibLibero),
  LOADS_PARAMETERS(
  {,
    (float) hysteresis_utility, // Hysteresis for the utility of the nodes, to avoid oscillations.
    (float) hysteresis_time,    // Time to wait before changing the target again.
  }),
});

class LibLiberoProvider : public LibLiberoProviderBase
{
private:
  spqr::UndirectedGraph liberoPositionGraph,            // Graph of the libero position (right on the field, looking at the opponent goal)
                        centralPositionGraph;           // Graph of the central position

  GlobalVector2f current_target;                        // Current target of the libero (a node of the graph)
  unsigned last_time_modified_node = theFrameInfo.time; // Last time the target was changed (used for the hysteresis)

  /**
   * @brief Updates the LibLibero representation
   * 
   * @param libLibero The LibLibero representation to update
   */
  void update(LibLibero& libLibero) override;

  /**
   * @brief Computes the libero position (global coordinates)
   * 
   * @return [GlobalVector2f] The libero position
   */
  GlobalVector2f getLiberoPosition();
  
  /**
   * @brief Computes the libero position in the graph (global coordinates)
   * 
   * @return [GlobalVector2f] The libero position in the graph
   */
  GlobalVector2f getLiberoPositionGraph();

  /**
   * @brief Computes the libero position in freeKick situations (global coordinates)
   * 
   * @return [GlobalVector2f] The libero position in special cases
   */
  GlobalVector2f getLiberoPositionSpecial();

public:
  LibLiberoProvider();
};
