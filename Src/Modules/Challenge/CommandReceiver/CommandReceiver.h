#pragma once


#include <iostream>
#include <arpa/inet.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <cstring>
#include <unistd.h>

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/KickoffState.h"
#include "Representations/Challenge/HumanCommand.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/Communication/UdpComm.h"
#include "Platform/SystemCall.h"
#include "Tools/Settings.h"


MODULE(CommandReceiver,
{,
REQUIRES(FrameInfo),
REQUIRES(RobotInfo),
REQUIRES(GameInfo),
REQUIRES(KickoffState),
PROVIDES(HumanCommand),

LOADS_PARAMETERS(
	{,
        (int) SEND_PORT,
        (int) RECEIVE_PORT,
        (std::string) SERVER_IP,
        (int) BUFFER_SIZE,
	}),
});

class CommandReceiver: public CommandReceiverBase
{
private:
    UdpComm socket_read;
    UdpComm socket_write;

    #ifndef TARGET_ROBOT
        std::string SERVER_IP = "127.0.0.1"; //LUIGI
    #endif

public:
	CommandReceiver();
    void update(HumanCommand& command);

};


