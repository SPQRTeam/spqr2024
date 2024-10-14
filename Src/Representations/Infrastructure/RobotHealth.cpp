/**
 * @file RobotHealth.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author <a href="mailto:fthielke@uni-bremen.de">Felix Thielke</a>
 */

#include "RobotHealth.h"
#include "Tools/Module/Blackboard.h"

// otherwise RobotHealthMessageID::getName() throws an unused function warning
struct WarningSuppressor
{
  ENUM(RobotHealthMessageId,
  {,
    idmotionFrameRate,
    idavgMotionTime,
    idmaxMotionTime,
    idminMotionTime,
    idcognitionFrameRate,
    idbatteryLevel,
    idtotalCurrent,
    idmaxJointTemperatureStatus,
    idjointWithMaxTemperature,
    idcpuTemperature,
    idload,
    idmemoryUsage,
    idwlan,
    idrobotName,
    idconfiguration,
    idlocation,
    idscenario,
  });
};

// Used to be sent over arbitrary message, but not anymore as of 2023.
// See any version of our (SPQR) code during RoboCup 2024 to recover communication code.
