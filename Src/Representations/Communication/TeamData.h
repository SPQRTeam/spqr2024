/**
 * @file TeamData.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/TeamTalk.h"
#include "Representations/Infrastructure/RefereeEstimator.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"

#include "Tools/Communication/SPLStandardMessageBuffer.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"
#include "Tools/Streams/Enum.h"

#include "Tools/Communication/BNTP.h"

STREAMABLE(Teammate, COMMA public MessageHandler
{
  const SynchronizationMeasurementsBuffer* bSMB = nullptr;

  unsigned toLocalTimestamp(unsigned remoteTimestamp) const
  {
    if(bSMB)
      return bSMB->getRemoteTimeInLocalTime(remoteTimestamp);
    else
      return 0u;
  };

  // [torchipeppo 2023] set role from standardmessage (see teammessagehandler)
  void operator<<(const PlayerRole& pr) {
    role = pr.role;
  }

  bool roleIsGoalkeeper() const {
    return PlayerRole::roleIsGoalkeeper(role);
  }
  bool rolePlaysTheBall() const {
    return PlayerRole::rolePlaysTheBall(role);
  }

  Vector2f getEstimatedPosition(unsigned time) const;

  /** MessageHandler function */
  bool handleMessage(InMessage& message) override;

  ENUM(Status,
  {,
    PENALIZED,                        /** OK   : I receive packets, but robot is penalized */
    FALLEN,                           /** GOOD : Robot is playing but has fallen or currently no ground contact */
    PLAYING,                          /** BEST : Teammate is standing/walking and has ground contact :-) */
  });
  ,

  (int)(-1) number,
  (bool)(false) isGoalkeeper, /**< This is for a teammate what \c theRobotInfo.isGoalkeeper() is for the player itself. */
  (bool)(true) isPenalized,
  (bool)(true) isUpright,
  (bool)(true) hasGroundContact,

  (unsigned)(0) timeWhenLastPacketSent,
  (unsigned)(0) timeWhenLastPacketReceived,
  (Status)(PENALIZED) status,
  (unsigned)(0) timeWhenStatusChanged,
  (signed char)(0) sequenceNumber,
  (signed char)(0) returnSequenceNumber,

  (RobotPose) theRobotPose,
  (BallModel) theBallModel,
  (FrameInfo) theFrameInfo,
  (ObstacleModel) theObstacleModel,
  (DiscretizedObstacleModel) theDiscretizedObstacleModel,
  (BehaviorStatus) theBehaviorStatus,
  (Whistle) theWhistle,
  (RefereeEstimator) theRefereeEstimator,
  (TeamBehaviorStatus) theTeamBehaviorStatus,

  (TeamTalk) theTeamTalk,

  // OUR STUFF
  (PlayerRole::RoleType) role,
});

/**
 * @struct TeammateData
 * Collection of teammate information
 */
STREAMABLE(TeamData,
{
  void draw() const;
  FUNCTION(void(const SPLStandardMessageBufferEntry* const)) generate,

  (std::vector<Teammate>) teammates, /**< An unordered(!) list of all teammates that are currently communicating with me */
  (int)(0) numberOfActiveTeammates,  /**< The number of teammates (in the list) that are at not PENALIZED */
  (unsigned)(0) receivedMessages,    /**< The number of received (not self) team messages */
});
