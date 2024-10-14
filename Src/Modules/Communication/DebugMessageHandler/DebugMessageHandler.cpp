/**
 * @file DebugMessageHandler.cpp
 *
 * @author Eugenio Bugli
 */

#include "DebugMessageHandler.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include "Platform/Time.h"

#include <iostream>
#include <fstream>


MAKE_MODULE(DebugMessageHandler, communication);


DebugMessageHandler::DebugMessageHandler()
{
  socket.setBlocking(false);
  socket.setBroadcast(false);
  socket.setTarget(tcm_ip_addr, Global::getSettings().teamPort);
  
}

void DebugMessageHandler::update(DebugMessageOutputGenerator& debugGenerator){

    // same as TeamMessageHandler.cpp
    theRobotStatus.hasGroundContact = theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionRequest.motion != MotionRequest::getUp;
    theRobotStatus.isUpright = theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting;
    if(theRobotStatus.hasGroundContact)
      theRobotStatus.timeOfLastGroundContact = theFrameInfo.time;
    if(theRobotStatus.isUpright)
      theRobotStatus.timeWhenLastUpright = theFrameInfo.time;


    debugGenerator.sendThisFrame = theFrameInfo.getTimeSince(timeLastSent) >= debugInterval || theFrameInfo.time < timeLastSent ;
    
    if (debugGenerator.sendThisFrame) {
      
      generateMessage(debugGenerator);

      // TCM connection   
      socket.write(reinterpret_cast<char*>(&debugGenerator.theDebugStandardMessage), debugGenerator.theDebugStandardMessage.numOfDataBytes + 8);
      // OUTPUT_TEXT(debugGenerator.theDebugStandardMessage << "\n");
    }

}

void DebugMessageHandler::generateMessage(DebugMessageOutputGenerator& debugGenerator) const
{ 

    debugGenerator.theDebugStandardMessage.playerNum = static_cast<uint8_t>(theRobotInfo.number);
    debugGenerator.theDebugStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);

    debugGenerator.timestamp = Time::getCurrentSystemTime();

    theRobotStatus.isPenalized = theRobotInfo.penalty != PENALTY_NONE;
    
    theRobotStatus.sequenceNumbers.fill(-1);

    debugGenerator.theDebugStandardMessage.fallen = static_cast<uint8_t>(!theRobotStatus.hasGroundContact || !theRobotStatus.isUpright);

    // Robot pose
    debugGenerator.theDebugStandardMessage.pose[0] = static_cast<float>(theRobotPose.translation.x()) ;
    debugGenerator.theDebugStandardMessage.pose[1] = static_cast<float>(theRobotPose.translation.y()) ;
    debugGenerator.theDebugStandardMessage.pose[2] = static_cast<float>(theRobotPose.rotation) ;

    // Ball age
    debugGenerator.theDebugStandardMessage.ballAge = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) / 1000.f;

    // Ball
    debugGenerator.theDebugStandardMessage.ball[0] = static_cast<float>(theBallModel.estimate.position[0]);
    debugGenerator.theDebugStandardMessage.ball[1] = static_cast<float>(theBallModel.estimate.position[1]);

    // Role
    debugGenerator.theDebugStandardMessage.role = thePlayerRole.role;

    // Obstacles types, centers and last seen
    if(!theObstacleModel.obstacles.empty()) {

      for(int i = 0; i < theObstacleModel.obstacles.size(); i++) {
        if(theObstacleModel.obstacles.size() > MAX_OBSTACLES) break;
        debugGenerator.theDebugStandardMessage.obsTypes[i] = theObstacleModel.obstacles[i].type;
        debugGenerator.theDebugStandardMessage.obsLastSeen[i] = theObstacleModel.obstacles[i].lastSeen;
      }
      
      for(int i = 0; i < 2*theObstacleModel.obstacles.size(); i+=2) {
        if(theObstacleModel.obstacles.size() > MAX_OBSTACLES) break;
        debugGenerator.theDebugStandardMessage.obsCenters[i] = theObstacleModel.obstacles[i/2].center[0]; // X coordinate
        debugGenerator.theDebugStandardMessage.obsCenters[i+1] = theObstacleModel.obstacles[i/2].center[1]; // Y coordinate
      }

      // Obstacle current size
      debugGenerator.theDebugStandardMessage.currentObsSize = theObstacleModel.obstacles.size();

      }

    debugGenerator.theDebugStandardMessage.messageBudget = theOwnTeamInfo.messageBudget; // does not work in simulation

    debugGenerator.theDebugStandardMessage.secsRemaining = theGameInfo.secsRemaining;

    // Arm Contact Model
    for(int i = 0; i < 2; i++) {
      debugGenerator.theDebugStandardMessage.armContact[i] = theArmContactModel.status[i].contact;
      debugGenerator.theDebugStandardMessage.armPushDirection[i] = theArmContactModel.status[i].pushDirection;
      debugGenerator.theDebugStandardMessage.armTimeOfLastContact[i] = theArmContactModel.status[i].timeOfLastContact;
    }

    // Whistle
    debugGenerator.theDebugStandardMessage.timeSinceLastWhistleDetection = theFrameInfo.time - theWhistle.lastTimeWhistleDetected;
    // you need to do this because theWhistle.detectionState is set equal to 2 only for one instant
    if (debugGenerator.theDebugStandardMessage.timeSinceLastWhistleDetection < 3000) {
      debugGenerator.theDebugStandardMessage.whistleDetectionState = 2; // whistle detected
    }
    else {
      debugGenerator.theDebugStandardMessage.whistleDetectionState = theWhistle.detectionState;
    }

    // TeamBall position and velocity
    debugGenerator.theDebugStandardMessage.teamBall[0] = theTeamBallModel.position[0];
    debugGenerator.theDebugStandardMessage.teamBall[1] = theTeamBallModel.position[1];
    debugGenerator.theDebugStandardMessage.teamBallVel[0] = theTeamBallModel.velocity[0];
    debugGenerator.theDebugStandardMessage.teamBallVel[1] = theTeamBallModel.velocity[1];

    // Battery level and Cpu Temperature
    debugGenerator.theDebugStandardMessage.batteryLevel = theSystemSensorData.batteryLevel;
    debugGenerator.theDebugStandardMessage.cpuTemperature = theSystemSensorData.cpuTemperature;

    // Referee Pose
    debugGenerator.theDebugStandardMessage.refereeDetection = theRefereeEstimator.isDetected; 

    // Size of the Message
    debugGenerator.theDebugStandardMessage.numOfDataBytes = static_cast<uint16_t>(debugGenerator.theDebugStandardMessage.sizeOfDebugMessage());

    timeLastSent = theFrameInfo.time;
}