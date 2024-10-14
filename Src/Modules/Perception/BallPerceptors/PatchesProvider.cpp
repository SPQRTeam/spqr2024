/**
 * @author Felix Thielke
 */
#include "PatchesProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <iostream>



//#ifdef TARGET_ROBOT
//    #define INCLUDE_PATCHES_ON_OBSTACLES
//#endif
//#define INCLUDE_PATCHES_ON_OBSTACLES

//#define UPPER_DIVIDE_4_PATROLING
#define UPPER_DIVIDE_6_PATROLING
//#define UPPER_DIVIDE_3_PATROLING


//#define USE_HIGH_RES_IMAGE


#define IS_GOALIE (theRobotInfo.number == 1)

// #define DEBUG true
// #define KALMAN_ROI_SIZE 2


#define ROI_RANGE_MULTIPLEX 4
#define NUM_FRAME_LAST_PREDICTION 6
#define KALMAN_PERSISTENCE 30
#define MIN_NEIGHBORS 13
#define OBS_NEIGHBORS 15

#define MIN_NEIGHBORS_GOALIE 6
#define OBS_NEIGHBORS_GOALIE 6

#ifdef PENALTY_STRIKER_GOALIE
#define MIN_NEIGHBORS_GOALIE 1
#define OBS_NEIGHBORS_GOALIE 1
#define MIN_NEIGHBORS 1
#define OBS_NEIGHBORS 1
#undef USE_HIGH_RES_IMAGE
#define UPPER_DIVIDE_3_PATROLING
#endif


MAKE_MODULE(PatchesProvider, perception);

void PatchesProvider::update(SPQRPatches& imagePatches)
{
    DECLARE_DEBUG_DRAWING("representation:BallPercept:image", "drawingOnImage");
    CameraInfo::Camera camera = theCameraInfo.camera;
    if(DEBUG)
        std::cerr << "start frame" << std::endl;
    //> First remove all the old patches
    imagePatches.patches.clear();
    int n_frame_not_seen = std::min(theBallPercept.n_frames_not_seen_up, theBallPercept.n_frames_not_seen_btm);

#ifdef PENALTY_STRIKER_GOALIE
    if (theRobotInfo.number == 1 && camera == CameraInfo::lower)
        return;
#endif

    if (DEBUG)
        std::cerr << "frame not seen " << n_frame_not_seen << std::endl;

    //>>> lastFramePrediction PATCH
    if (theBallPercept.camera == camera && n_frame_not_seen <= NUM_FRAME_LAST_PREDICTION) // seen in the same camera in the last step
    {
        if (DEBUG)
            std::cerr << "lastFramePrediction" << std::endl;

        //> take a ROI around the previous ball
        int roi_range = static_cast<int>(FROMBHUMANF(theBallPercept.radiusInImage * ROI_RANGE_MULTIPLEX));

        SPQRPatch patch;

        patch.offset = Vector2s(static_cast<short>(std::max((int)(FROMBHUMANF(theBallPercept.positionInImage.x()) - roi_range), 0)),
                                static_cast<short>(std::max((int)(FROMBHUMANF(theBallPercept.positionInImage.y()) - roi_range), 0)));
        if(DEBUG) {
            printf("roi_range: %d\n offset: %d %d\n",roi_range,patch.offset.x(),patch.offset.y());
        }
        patch.width = static_cast<short>(std::min(roi_range << 1, theCameraImage.CV_image.cols - (int)(FROMBHUMANF(theBallPercept.positionInImage.x())) + roi_range));

        patch.height = static_cast<short>(std::min(roi_range << 1, theCameraImage.CV_image.rows - (int)(FROMBHUMANF(theBallPercept.positionInImage.y())) + roi_range));
        patch.type = SPQRPatch::lastFramePrediction;
        //> if the ROI is on the predicted ball and on an obstacle/own body remove it
        Vector2i center = Vector2i(patch.offset.x() + (patch.width >> 1), patch.offset.y() + (patch.height >> 1));

        const bool isStable = theMotionRequest.motion == MotionRequest::Motion::stand;

        if (checkBallPosition(TOBHUMANVI(center)))
        {
            if(!IS_GOALIE)
            {
                if (isOnObstacle(center) || isOnPenalty(center))
                    patch.minNeighbors = OBS_NEIGHBORS;
                else
                    patch.minNeighbors = MIN_NEIGHBORS;
            }
            else
            {
                if (isOnObstacle(center) || isOnPenalty(center))
                    patch.minNeighbors = isStable ? static_cast<int>(OBS_NEIGHBORS_GOALIE*0.5) : OBS_NEIGHBORS_GOALIE;
                else
                    patch.minNeighbors = isStable ? static_cast<int>(MIN_NEIGHBORS_GOALIE*0.5) : MIN_NEIGHBORS_GOALIE;
            }
            //Vector2f pointInImage;
            if(camera == CameraInfo::upper)
                patch.isHighResolution = UPPER_SCALE_FACTOR == 1 ? true : false;
            else
                patch.isHighResolution = LOWER_SCALE_FACTOR == 1 ? true : false;

            computeMinMaxBallDiameter(patch);

            imagePatches.patches.push_back(patch);
            if (DEBUG)
            {
                std::cerr << "------------------- roi " << std::endl;
                std::cerr << "lastFramePrediction 2" << std::endl;
            }

        }
    }
    else if ((n_frame_not_seen > NUM_FRAME_LAST_PREDICTION*0.5 && n_frame_not_seen < KALMAN_PERSISTENCE) || theTeamBallModel.isValid )
    {
        //>>> kalmanPrediction PATCH
        Vector2f pointInImage;

        // TODO: IS IT RIGHT?
        Vector3f ball = Vector3f(theWorldModelPrediction.ballPosition.x(), theWorldModelPrediction.ballPosition.y(), theBallSpecification.radius);

        bool res = Transformation::robotToImage(ball, theCameraMatrix, theCameraInfo, pointInImage);

        if (res)
        {
            if (DEBUG)
                std::cerr << "kalmanPrediction" << std::endl;
            if(DEBUG) {
                Vector2f ret = Transformation::fieldToRobot(theRobotPose, theTeamBallModel.position);
                printf("Transformation::fieldToRobot %f %f\n",ret.x(),ret.y());
            }

            int ballDiameter = static_cast<int>(FROMBHUMANF(computeBallDiameter(Vector2i(pointInImage.x(), pointInImage.y()))));
            pointInImage = FROMBHUMANVF(pointInImage);
            pointInImage.y() -= ballDiameter >> 1;
            if (pointInImage.x() > 0 && pointInImage.x() < ((camera==CameraInfo::upper) ? UPPER_CAMERA_WIDTH : LOWER_CAMERA_WIDTH) &&
                    pointInImage.y() > 0 && pointInImage.y() < ((camera==CameraInfo::upper) ? UPPER_CAMERA_HEIGHT : LOWER_CAMERA_HEIGHT))
            {

                Vector2s min_point = Vector2s(
                                         static_cast<short>(std::max((int)(pointInImage.x() - ballDiameter * KALMAN_ROI_SIZE), 0)),
                                         static_cast<short>(std::max((int)(pointInImage.y() - (ballDiameter * KALMAN_ROI_SIZE)), 0))
                                     );
                Vector2s max_point = Vector2s(
                                         static_cast<short>(std::min((int)(pointInImage.x() + (ballDiameter * KALMAN_ROI_SIZE)), theCameraImage.CV_image.cols)),
                                         static_cast<short>(std::min((int)(pointInImage.y() + (ballDiameter * KALMAN_ROI_SIZE)), theCameraImage.CV_image.rows))
                                     );


                SPQRPatch patch;
                if(!IS_GOALIE)
                {
                    patch = SPQRPatch(
                                theCameraImage,
                                min_point,
                                max_point.x() - min_point.x(),
                                max_point.y() - min_point.y(),
                                SPQRPatch::PatchType::kalmanPrediction,
                                (isOnObstacle(Vector2i(pointInImage.x(), pointInImage.y())) ? OBS_NEIGHBORS : MIN_NEIGHBORS)
                            );

                }
                else
                {
                    patch = SPQRPatch(
                                theCameraImage,
                                min_point,
                                max_point.x() - min_point.x(),
                                max_point.y() - min_point.y(),
                                SPQRPatch::PatchType::kalmanPrediction,
                                (isOnObstacle(Vector2i(pointInImage.x(), pointInImage.y())) ? OBS_NEIGHBORS_GOALIE : MIN_NEIGHBORS_GOALIE)
                            );

                }
                if(camera == CameraInfo::upper)
                    patch.isHighResolution = UPPER_SCALE_FACTOR == 1 ? true : false;
                else
                    patch.isHighResolution = LOWER_SCALE_FACTOR == 1 ? true : false;
                computeMinMaxBallDiameter(patch);
                imagePatches.patches.push_back(patch);
                if (DEBUG)
                    std::cerr << "kalmanPrediction 2" << std::endl;

            }
            if(!(n_frame_not_seen > NUM_FRAME_LAST_PREDICTION/2 && n_frame_not_seen < KALMAN_PERSISTENCE) and theTeamBallModel.isValid)
                CROSS("representation:BallPercept:image", TOBHUMANF(pointInImage.x()), TOBHUMANF(pointInImage.y()), 20, 4, Drawings::solidPen, ColorRGBA::violet);
            else
                CROSS("representation:BallPercept:image", TOBHUMANF(pointInImage.x()), TOBHUMANF(pointInImage.y()), 20, 4, Drawings::solidPen, ColorRGBA::orange);
        }
    }

    else if(n_frame_not_seen > NUM_FRAME_LAST_PREDICTION + KALMAN_PERSISTENCE/2)
    {        
        if (DEBUG)
            std::cerr << "patroling" << std::endl;
        SPQRPatch patch;
        if (getPatrolingPatch(patch, n_frame_not_seen, camera))
        {
            if (DEBUG)
                printf("patch:[%f %f]",patch.offset.x(),patch.offset.y());
            // If the ROI we find is outside skip and find another ROI
            int height = theCameraInfo.height - 3;
            int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
            horizon = std::min(horizon, height);
            horizon = static_cast<int>(FROMBHUMANF(horizon));
            if (patch.offset.y() < horizon)
            {
                //std::cerr << "patch.offset.y():" << patch.offset.y() << " - patch.height:" << patch.height << " - horizon:" << horizon << std::endl;
                patch.height = static_cast<short>(std::max(((patch.offset.y() + patch.height) - horizon ),120));
                //std::cerr << "patch.height LATER:" <<patch.height << std::endl;
                patch.offset.y() = static_cast<short>(horizon);
                computeMinMaxBallDiameter(patch);
                imagePatches.patches.push_back(patch);
            }
            else
            {
                computeMinMaxBallDiameter(patch);
                imagePatches.patches.push_back(patch);
            }
        }



        //>> bottomObstacles PATCHES
// #ifdef INCLUDE_PATCHES_ON_OBSTACLES
//         if(std::min(theBallPercept.n_frames_not_seen_up, theBallPercept.n_frames_not_seen_btm) > NUM_FRAME_LAST_PREDICTION/2){
//           for(const auto player : thePlayersImagePercept.players)
//           {
//             ImagePatch patch;
//             int ballDiameter = FROMBHUMANF(computeBallDiameter(Vector2i(player.x1, player.y2)));
//             patch.offset = FROMBHUMANF(Vector2s(player.x1, player.y2 - ballDiameter));
//             patch.width = FROMBHUMANF(player.x2 - player.x1);
//             patch.height = ballDiameter * 1.5;
//             patch.type = ImagePatch::PatchType::bottomObstacles;
//             patch.minNeighbors = OBS_NEIGHBORS;
//             computeMinMaxBallDiameter(patch);
//             imagePatches.patches.push_back(patch);
//           }
//     }
// #endif

    }

}

bool PatchesProvider::getPatrolingPatch(SPQRPatch& patch, int n_frame_not_seen, CameraInfo::Camera camera)
{
    patch.type = SPQRPatch::framePatroling;
    if(!IS_GOALIE)
    {
        patch.minNeighbors = MIN_NEIGHBORS;
    }
    else
    {
        patch.minNeighbors = MIN_NEIGHBORS_GOALIE;
    }
    if (camera == CameraInfo::upper)
    {


        switch (PatchesProvider::state)
        {
        case 0:
            patch.offset = Vector2s(0, FROMBHUMANF(280));
            patch.width = static_cast<short>(FROMBHUMANF(360));
            patch.height = static_cast<short>(FROMBHUMANF(200));
            PatchesProvider::state = 1;
            break;

        case 1:
            patch.offset = Vector2s(FROMBHUMANF(280), FROMBHUMANF(280));
            patch.width = static_cast<short>(FROMBHUMANF(360));
            patch.height = static_cast<short>(FROMBHUMANF(200));
            PatchesProvider::state = 2;
            break;
        
        case 2:
            patch.offset = Vector2s(0, FROMBHUMANF(140));
            patch.width = static_cast<short>(FROMBHUMANF(360));
            patch.height = static_cast<short>(FROMBHUMANF(200));
            PatchesProvider::state = 3;
            break;

        case 3:
            patch.offset = Vector2s(FROMBHUMANF(280), FROMBHUMANF(140));
            patch.width = static_cast<short>(FROMBHUMANF(360));
            patch.height = static_cast<short>(FROMBHUMANF(200));
            PatchesProvider::state = 4;
            break;

        case 4:
            patch.offset = Vector2s(0, 0);
            patch.width = static_cast<short>(FROMBHUMANF(360));
            patch.height = static_cast<short>(FROMBHUMANF(200));
            PatchesProvider::state = 5;

        case 5:
            patch.offset = Vector2s(static_cast<short>(FROMBHUMANF(280)), static_cast<short>(FROMBHUMANF(0)));
            patch.width = static_cast<short>(FROMBHUMANF(360));
            patch.height = static_cast<short>(FROMBHUMANF(200));
            PatchesProvider::state = 0;
        
        default:
            break;
        }

        patch.isHighResolution = UPPER_SCALE_FACTOR == 1 ? true : false;
    }
    else // BOTTOM CAMERA
    {
        if (DEBUG) {
            std::cerr << "------------- LOWER CAMERA" << std::endl;
            std::cerr << theCameraInfo.height << std::endl;
            std::cerr << theCameraInfo.width << std::endl<< std::endl;

        }
        patch.offset = Vector2s(0, 0);
        patch.width = static_cast<short>(FROMBHUMANF(320));
        patch.height =  static_cast<short>(FROMBHUMANF(240));
        patch.isHighResolution = LOWER_SCALE_FACTOR == 1 ? true : false;
    }

    return true;
}

bool PatchesProvider::isOnObstacle(Vector2i ball)
{
    for(const auto player : theObstaclesImagePercept.obstacles)
    {
        if (ball.x() > player.left && ball.x() < player.right && ball.y() > player.bottom && ball.y() < player.top)
            return true;
    }
    return false;
}

bool PatchesProvider::isOnPenalty(Vector2i ball){
    return thePenaltyMarkPercept.wasSeen ? (thePenaltyMarkPercept.positionInImage - ball).norm() < theBallSpecification.radius : false ;
}
bool PatchesProvider::checkBallPosition(Vector2i ball)
{
#ifdef TARGET_ROBOT
    // check inside body contour
    if (!theBodyContour.isValidPoint(ball))
    {
        return false;
    }
#endif

    // calculate the image limits
    int height = theCameraInfo.height - 3;
    int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
    horizon = std::min(horizon, height);

    if (ball.y() <= horizon)
        return false;

#ifndef INCLUDE_PATCHES_ON_OBSTACLES
    if (isOnObstacle(ball))
        return false;
#endif

    return true;
}

void PatchesProvider::computeMinMaxBallDiameter(SPQRPatch& patch)
{

    std::vector<int> addCoords = {patch.offset.x(), patch.offset.y(), patch.offset.x() + patch.width, patch.offset.y() + patch.height};
    for (int i = 0; i < 2; ++i)
    {
        const Vector2i& spot = Vector2i(addCoords.at((i << 1) + 0), addCoords.at((i << 1) + 1));

        short diameter = static_cast<short>(FROMBHUMANF(computeBallDiameter(TOBHUMANVI(spot))));
        if (i == 0)
            patch.minBallSize = diameter;
        else
            patch.maxBallSize = diameter;

        CIRCLE("representation:BallPercept:image", TOBHUMANF(addCoords.at((i << 1) + 0)), TOBHUMANF(addCoords.at((i << 1) + 1)), TOBHUMANF((diameter >> 1)),
               3, Drawings::dottedPen, ColorRGBA::blue, Drawings::BrushStyle::noBrush, ColorRGBA::green);
    }

    /*  approxDiameter.at(0).width = max(approxDiameter.at(0).width, 24);
    approxDiameter.at(0).height = max(approxDiameter.at(0).height, 24);
    std::cerr << "min rad: " << approxDiameter.at(0)<< ", " << approxDiameter.at(0) << std::endl;
    std::cerr << "max rad: " << approxDiameter.at(1)<< ", " << approxDiameter.at(1) << std::endl;
    return approxDiameter;
    */
}

short PatchesProvider::computeBallDiameter(Vector2i pointInImage)
{
    Vector2f correctedStart = theImageCoordinateSystem.toCorrected(pointInImage);
    Vector3f cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x()
                           - correctedStart.x(), theCameraInfo.opticalCenter.y() - correctedStart.y());
    Vector3f unscaledField = theCameraMatrix.rotation * cameraToStart;
    if(unscaledField.z() >= 0.f)
    {
        return 0;// above horizon
    }
    const float scaleFactor = (theCameraMatrix.translation.z() - theBallSpecification.radius) / unscaledField.z();
    cameraToStart *= scaleFactor;
    unscaledField *= scaleFactor;
    /*if(unscaledField.topRows(2).squaredNorm() > sqrMaxBallDistance)
    {
        return false; // too far away, computed with respect to the field.
    }
    */
    cameraToStart.y() += cameraToStart.y() > 0 ? -theBallSpecification.radius : theBallSpecification.radius;
    cameraToStart /= scaleFactor;
    float appRad = std::abs(theCameraInfo.opticalCenter.x() - cameraToStart.y() - correctedStart.x());

    return static_cast<short>( appRad ) << 1;
}
