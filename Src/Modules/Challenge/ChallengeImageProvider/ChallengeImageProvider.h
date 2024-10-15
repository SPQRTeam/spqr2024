
#pragma once


#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Challenge/ChallengeImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"


#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cstring>    
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>   
#include "Platform/Thread.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"


MODULE(ChallengeImageProvider,
{,
    REQUIRES(FrameInfo),
    USES(CameraImage),
    REQUIRES(CameraInfo),
    REQUIRES(ECImage),
    PROVIDES(ChallengeImage),

    LOADS_PARAMETERS(
    {,
        (int) sendInterval, //time in ms between two debug images
    }),
});

class ChallengeImageProvider : public ChallengeImageProviderBase 
{    
    public:
        void update(ChallengeImage& theChallengeImage);

    private:
        Image<PixelTypes::GrayscaledPixel> frame;
        mutable unsigned timeLastSent = 0;
        int sock;
        void closeConnection();
        bool receiveMessage();
        bool establishConnection();
        bool sendImage();

        #ifdef TARGET_ROBOT
            const char* ip_addr = "10.0.19.90"; //LUIGI
        #else
            const char* ip_addr = "127.0.0.1";
        #endif
};
