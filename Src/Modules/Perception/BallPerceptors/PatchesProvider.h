#pragma once
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "opencv2/core/core.hpp"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/spqr_representations/SPQRPatches.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#ifdef TARGET_ROBOT
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#endif


MODULE(PatchesProvider,
{,
    USES(CameraMatrix),
    USES(FieldDimensions),
    USES(BallSpots),
    USES(FrameInfo),
    USES(BallSpecification),
    USES(CameraInfo),
    USES(CameraImage),
    USES(ObstaclesFieldPercept),
    USES(ObstaclesImagePercept),
    USES(TeamBallModel),
    USES(WorldModelPrediction),
    USES(PenaltyMarkPercept),
    USES(MotionRequest),
#ifdef TARGET_ROBOT
    REQUIRES(BodyContour),
#endif
    USES(ImageCoordinateSystem),
    USES(RobotInfo),
    USES(RobotPose),
    USES(BallPercept),
    PROVIDES(SPQRPatches),
    LOADS_PARAMETERS(
    {,
        (int)(2) KALMAN_ROI_SIZE,
        (bool) DEBUG,
    }),

});

class PatchesProvider : public PatchesProviderBase
{
private:

    unsigned state = 0;

    void update(SPQRPatches& imagePatches);


    /**
    * Compare the ball position and the players position
    * @param ball the coordinates of the ball
    * @return true if there's a player near the ball
    * else
    * @return false
    */
    bool isOnObstacle(Vector2i ball);

    bool isOnPenalty(Vector2i ball);


    /**
    * Determine if the ball is inside the camera's image
    * @param ball the coordinates of the ball
    * @return false if the ball is above horizon
    * else
    * @return true if the ball is in image limits
    */
    bool checkBallPosition(Vector2i ball);


    /**
    * Determine the offset, the width and the height of the patch
    * subject to the number of the frame not seen
    * @param patch the image patch
    * @param n_frame_not_seen number of the frame not seen
    * @param camera the camera which takes the frames
    * @return true if the operation was completed successfully
    */
    bool getPatrolingPatch(SPQRPatch& patch, int n_frame_not_seen, CameraInfo::Camera camera);


    /**
    * Determine an estimate of the min and the max of ball diameter
    * using the function computeBallDiameter
    * @param patch the image patch
    * @return assignments to patch.minBallSize and patch.maxBallSize
    */
    void computeMinMaxBallDiameter(SPQRPatch& patch);


    /**
    * Calculate the ball diameter using the CameraInfo
    * corrected by a scaleFactor
    * @param pointInImage the coordinates of the ball
    * @return 0 if the ball is above the horizon
    * else
    * @return a short which represent the ball diameter
    */
    short computeBallDiameter(Vector2i pointInImage);
};