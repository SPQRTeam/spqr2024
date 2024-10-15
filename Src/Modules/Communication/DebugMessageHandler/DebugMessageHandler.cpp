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
  File f("teamMessage.def", "r");
  ASSERT(f.exists());
  std::string source(f.getSize(), 0);
  f.read(source.data(), source.length());
  teamCommunicationTypeRegistry.addTypes(source);
  teamCommunicationTypeRegistry.compile();
  teamMessageType = teamCommunicationTypeRegistry.getTypeByName("TeamMessage");

  oldMessagesInstance.RobotPose = Eigen:: Vector2f::Zero(); 
  oldMessagesInstance.Rob_movement_Treshold = 1000.0;
  oldMessagesInstance.BallPosition = Eigen:: Vector2f::Zero();
  oldMessagesInstance.Ball_pos_movement_Threshold = 1000.0;
  oldMessagesInstance.num_opponents = 0;
  socket.setBlocking(false);
  socket.setBroadcast(false);
  socket.setTarget(tcm_ip_addr, Global::getSettings().teamPort);

  if(theRobotInfo.number == RobotInfo::RoleNumber::controlled){
    challengeSocket.setBlocking(false);
    challengeSocket.setBroadcast(false);
    challengeSocket.setTarget(tcm_ip_addr, 65305); //TODO hardcoded port
  }
  
}

void DebugMessageHandler::update(DebugMessageOutputGenerator& debugGenerator){

    // same as TeamMessageHandler.cpp
    theRobotStatus.hasGroundContact = theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionRequest.motion != MotionRequest::getUp;
    theRobotStatus.isUpright = theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting;
    if(theRobotStatus.hasGroundContact)
      theRobotStatus.timeOfLastGroundContact = theFrameInfo.time;
    if(theRobotStatus.isUpright)
      theRobotStatus.timeWhenLastUpright = theFrameInfo.time;


    debugGenerator.sendThisFrame = theFrameInfo.getTimeSince(timeLastSent) >= messageInterval || theFrameInfo.time < timeLastSent ;
    
    if (debugGenerator.sendThisFrame && theRobotInfo.number == RobotInfo::RoleNumber::controlled) {
      
      generateMessage(debugGenerator);

      // TCM connection   
      // socket.write(reinterpret_cast<char*>(&debugGenerator.theDebugStandardMessage), debugGenerator.theDebugStandardMessage.numOfDataBytes + 8);
      challengeSocket.write(reinterpret_cast<char*>(&debugGenerator.theDebugStandardMessage), debugGenerator.theDebugStandardMessage.numOfDataBytes + 8);
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

    // theRobotPose;
    debugGenerator.theDebugStandardMessage.pose[0] = static_cast<float>(theRobotPose.translation.x()) ;
    debugGenerator.theDebugStandardMessage.pose[1] = static_cast<float>(theRobotPose.translation.y()) ;
    debugGenerator.theDebugStandardMessage.pose[2] = static_cast<float>(theRobotPose.rotation) ;

    //ballAge
    debugGenerator.theDebugStandardMessage.ballAge = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) / 1000.f;

    //ball
    Pose2f ballPos = theLibMisc.rel2Glob(static_cast<float>(theBallModel.estimate.position[0]), static_cast<float>(theBallModel.estimate.position[1]));
    debugGenerator.theDebugStandardMessage.ball[0] = ballPos.translation.x();
    debugGenerator.theDebugStandardMessage.ball[1] = ballPos.translation.y();

    debugGenerator.theDebugStandardMessage.role = thePlayerRole.role;

    //obstacles
    if(!theObstacleModel.obstacles.empty()) {

      for(int i = 0; i < theObstacleModel.obstacles.size(); i++) {
        if(theObstacleModel.obstacles.size() > MAX_OBSTACLES) break;
        debugGenerator.theDebugStandardMessage.obsTypes[i] = theObstacleModel.obstacles[i].type;
        debugGenerator.theDebugStandardMessage.obsLastSeen[i] = theObstacleModel.obstacles[i].lastSeen;
      }
      
      for(int j = 0; j < theObstacleModel.obstacles.size(); j++) {
        if(theObstacleModel.obstacles.size() > MAX_OBSTACLES) break;
        Pose2f obsCenter = theLibMisc.rel2Glob(static_cast<float>(theObstacleModel.obstacles[j].center[0]), static_cast<float>(theObstacleModel.obstacles[j].center[1]));
        debugGenerator.theDebugStandardMessage.obsCenterX[j] = obsCenter.translation.x();
        debugGenerator.theDebugStandardMessage.obsCenterY[j] = obsCenter.translation.y();
      }

      // obstacle current size
      debugGenerator.theDebugStandardMessage.currentObsSize = theObstacleModel.obstacles.size();

      }

    debugGenerator.theDebugStandardMessage.messageBudget = theOwnTeamInfo.messageBudget;

    debugGenerator.theDebugStandardMessage.secsRemaining = theGameInfo.secsRemaining;

    // Arm Contact Model
    for(int i = 0; i < 2; i++) {
      debugGenerator.theDebugStandardMessage.armContact[i] = theArmContactModel.status[i].contact;
      debugGenerator.theDebugStandardMessage.armPushDirection[i] = theArmContactModel.status[i].pushDirection;
      debugGenerator.theDebugStandardMessage.armTimeOfLastContact[i] = theArmContactModel.status[i].timeOfLastContact;
    }

    // whistle
    debugGenerator.theDebugStandardMessage.timeSinceLastWhistleDetection = theFrameInfo.time - theWhistle.lastTimeWhistleDetected;
    if (debugGenerator.theDebugStandardMessage.timeSinceLastWhistleDetection < 3000) {
      debugGenerator.theDebugStandardMessage.whistleDetectionState = 2;
    }
    else {
      debugGenerator.theDebugStandardMessage.whistleDetectionState = theWhistle.detectionState;
    }


    // TeamBall
    debugGenerator.theDebugStandardMessage.teamBall[0] = theTeamBallModel.position[0];
    debugGenerator.theDebugStandardMessage.teamBall[1] = theTeamBallModel.position[1];
    debugGenerator.theDebugStandardMessage.teamBallVel[0] = theTeamBallModel.velocity[0];
    debugGenerator.theDebugStandardMessage.teamBallVel[1] = theTeamBallModel.velocity[1];

    // Battery 
    debugGenerator.theDebugStandardMessage.batteryLevel = theSystemSensorData.batteryLevel;
    debugGenerator.theDebugStandardMessage.cpuTemperature = theSystemSensorData.cpuTemperature;

    // Referee Pose
    debugGenerator.theDebugStandardMessage.refereeDetection = theRefereeEstimator.isDetected;

    debugGenerator.theDebugStandardMessage.numOfDataBytes = static_cast<uint16_t>(debugGenerator.theDebugStandardMessage.sizeOfDebugMessage());

    timeLastSent = theFrameInfo.time;
}