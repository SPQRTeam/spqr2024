/**
 * @file MessageManager.h
 *
 * Which provides the values for MessageManagement according to the
 * situation of the robot and any events.
 *
 * @author Francesco Petri
 */

#pragma once

#include "Representations/Communication/MessageManagement.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RefereeEstimator.h"
#include "Representations/Modeling/RobotPose.h"
#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include "Modules/Communication/TeamMessageHandler/TeamMessageHandler.h"

MODULE(MessageManager,
{,
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  USES(TeamData),
  USES(GameInfo),
  USES(BallModel),
  USES(RobotPose),
  USES(TeamBallModel),
  USES(RefereeEstimator),
  USES(Whistle),
  USES(PlayerRole),
  PROVIDES(MessageManagement),
  LOADS_PARAMETERS(
  {,
    (int) sendIntervalIfNoBall,
    (int) sendIntervalIfBallSeen,
    (int) ballSeenTimeThreshold,
    (unsigned) outOfPacketsThreshold,
    (int) teamBallNotSeenThreshold,
    (int) whistleEventCooldown,
    (int) readyGestureEventCooldown,
    (int) sendIntervalGoalie,
    (int) sendIntervalReadyStateDefenders,
    (int) sendIntervalReadyStateAttackers,
  }),
});

class MessageManager : public MessageManagerBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theMessageManagement The representation updated.
   */
  void update(MessageManagement& mm);


public:
  /** Initialize data and open socket. */
  MessageManager();

private:
  bool was_searcher = false;
  bool teamBall_isValid_prev = false;
  unsigned ballModel_timeLastSeen_prev = 0;
  unsigned teamBall_timeLastSeen_prev = 0;
  unsigned lastTeamBallEventTS = 0;
  unsigned lastWhistleEventTS = 0;
  unsigned lastGestureDetectedTS = 0;
};
