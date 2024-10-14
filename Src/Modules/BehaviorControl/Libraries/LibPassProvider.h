/**
 * @file LibPassProvider.h
 * 
 * See LibPass
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Communication/MessageManagement.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/Libraries/LibPass.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/BHMath.h"


MODULE(LibPassProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamPlayersModel),
  REQUIRES(FieldDimensions),
  REQUIRES(KickInfo),
  REQUIRES(MessageManagement),
  REQUIRES(TeamData),
  REQUIRES(FieldBall),
  REQUIRES(LibMisc),
  USES(PlayerRole),
  PROVIDES(LibPass),
  LOADS_PARAMETERS(
  {,
    (float) bestPassageDistance, // the best distance to do a passage
    (float) passageVariance, // the variance of distance to do a passage
    (float) closestInterceptorThreshold, // the maximum distance of the closest opponent interceptor after which the passage line is considered completely free. 
    (float) minDistanceGain, // minimum euclidean distance difference between the sender and the receiver of the ball to consider a passage
  }),
});

class LibPassProvider : public LibPassProviderBase
{
private:
  
  /**
   * Updates LibPass
   * @param libPass The representation provided
   */
  void update(LibPass& libPass) override;


  // ===== DANIELE 2024 ======

  /**
  TODO: documentazione
  */
  float getPassUtility(Vector2f position, int timeSLR) const;

  /**
  TODO: documentazione
  */
  std::tuple<Vector2f, float> getBestPassage() const;

  std::tuple<Vector2f, float> getBestPassageSpecial() const;


  /**
  TODO: documentazione
  */
  float getInversePassUtility(Vector2f position) const;

  // ===== IMPLEMENTATIONS OF LibPass =====

  /**
   * Find a free passing line to the target
   * Returns a valid target or goal center if fails
   */
  Vector2f findPassingLine(Vector2f target, std::vector<Vector2f> mates);

  /**
   * This function orderes the team mates according to their distance w.r.t. the opponents
   * and then, starting from the most free one, tries to find a passing line
   * @return a Vector2f poiting to the passing target or the goal line if no passing is found
   */
  Pose2f poseToPass(LibPass& libPass);     // this gets the libPass parameter b/c it needs to modify something


  // ===== FOR INTERNAL USE =====

  /** 
   * An auxiliary for findPassingLine.
   * No other doc was available at the time of this port (2022->2023).
   */
  bool findPassingLineAux(std::vector<Obstacle> opponents, Vector2f& target, const Vector2f& translation);


};
