/**
 * @file DebugMessageHandler.h
 *
 * Module declaration for Debug messages
 *
 * @author Eugenio Bugli
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Module.h"
#include "Tools/Communication/CompressedTeamCommunicationStreams.h"
#include "Representations/Communication/DebugMessage.h"
#include "Tools/Communication/RobotStatus.h"
#include "Tools/Communication/BNTP.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Communication/MessageManagement.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/RefereeEstimator.h"

#include "Tools/Communication/UdpComm.h"
#include <vector>


MODULE(DebugMessageHandler,
{,
    REQUIRES(FrameInfo),
    REQUIRES(MotionInfo),
    USES(OwnTeamInfo),
    USES(MotionRequest),
    USES(RawGameInfo),
    USES(GameInfo),
    USES(ArmContactModel),
    REQUIRES(MessageManagement),
    REQUIRES(FallDownState),
    REQUIRES(GroundContactState),
    REQUIRES(LibMisc),
    USES(RobotInfo),

    USES(BallModel),
    USES(TeamBallModel),
    USES(ObstacleModel),
    USES(DiscretizedObstacleModel),
    USES(RobotPose),
    USES(Whistle),
    USES(PlayerRole),
    USES(SystemSensorData),
    USES(RefereeEstimator),

    PROVIDES(DebugMessageOutputGenerator),

    LOADS_PARAMETERS(
    {,
       (int) messageInterval, //time in ms between two debug messages
    }),
});

/**
 * @class DebugMessageHandler
 * A modules for sending debug messages to TCM
 */
class DebugMessageHandler : public DebugMessageHandlerBase
{
public:
  DebugMessageHandler();

private:
    struct oldMessages{
    Vector2f RobotPose = Eigen::Vector2f::Zero(); 
    float Rob_movement_Treshold = 1000.0;
    Vector2f BallPosition = Eigen::Vector2f::Zero();
    float Ball_pos_movement_Threshold = 1000.0;
    int num_opponents = 0;
    };

    oldMessages oldMessagesInstance;

    UdpComm socket;
    UdpComm challengeSocket;

    CompressedTeamCommunication::TypeRegistry teamCommunicationTypeRegistry;
    const CompressedTeamCommunication::Type* teamMessageType;

    //BNTP theBNTP;
    mutable RobotStatus theRobotStatus;

    // output stuff
    mutable unsigned timeLastSent = 0;

    #ifdef TARGET_ROBOT
      const char* tcm_ip_addr = "10.0.19.90"; //LUIGI
    #else
      const char* tcm_ip_addr = "127.0.0.1";
    #endif

    void update(DebugMessageOutputGenerator& debugGenerator) override;
    void generateMessage(DebugMessageOutputGenerator& debugGenerator) const;
};