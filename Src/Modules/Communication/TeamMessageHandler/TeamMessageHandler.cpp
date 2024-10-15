/**
 * @file TeamMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "TeamMessageHandler.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include "Platform/Time.h"

#include <iostream>
#include <fstream>

//#define SITTING_TEST
//#define SELF_TEST

#define INCLUDE_BHUMAN_ARBITRARY_MESSAGE false
#define USE_SIMILARITY_FUNCTION false

using namespace std;


MAKE_MODULE(TeamMessageHandler, communication);

// BNTP, RobotStatus, RobotPose, FieldCoverage, RobotHealth and FieldFeatureOverview cannot be part of this for technical reasons.
#define FOREACH_TEAM_MESSAGE_REPRESENTATION(_) \
  _(FrameInfo); \
  _(BallModel); \
  _(DiscretizedObstacleModel); \
  _(Whistle); \
  _(RefereeEstimator); \
  _(HumanCommand);

struct TeamMessage{};

int TeamMessageHandler::count_opponents(vector<Obstacle> obstacles){
  int num_opponents = 0;
  for (auto obstacle:obstacles){
    if (obstacle.isOpponent()){
      ++num_opponents;
    }
  }
  return num_opponents;
}

bool TeamMessageHandler::isDifferent(BHumanMessageOutputGenerator& outputGenerator){
  Vector2f RobotPose_difference = oldMessagesInstance.RobotPose - theRobotPose.translation;
  Vector2f BallPose_difference = oldMessagesInstance.BallPosition - theBallModel.estimate.position;
  int new_num_opponents = count_opponents(theObstacleModel.obstacles);
  
  bool time_condition = (outputGenerator.timeLastSent < theFrameInfo.time);
  bool opponents_condition = (oldMessagesInstance.num_opponents != new_num_opponents);
  bool RobotPose_condition = (RobotPose_difference.norm() > oldMessagesInstance.Rob_movement_Treshold);
  bool BallPose_condition = (BallPose_difference.norm() > oldMessagesInstance.Ball_pos_movement_Threshold);
  
  if (time_condition && (opponents_condition|| RobotPose_condition || BallPose_condition)){
      
      oldMessagesInstance.num_opponents = new_num_opponents;
      oldMessagesInstance.RobotPose = theRobotPose.translation;
      oldMessagesInstance.BallPosition = theBallModel.estimate.position;   
      return true;
  }
  return false;
}

void TeamMessageHandler::regTeamMessage()
{
  PUBLISH(regTeamMessage);
  const char* name = typeid(TeamMessage).name();
  TypeRegistry::addClass(name, nullptr);
#define REGISTER_TEAM_MESSAGE_REPRESENTATION(x) TypeRegistry::addAttribute(name, typeid(x).name(), "the" #x)

  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotStatus);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotPose);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(PlayerRole);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(REGISTER_TEAM_MESSAGE_REPRESENTATION);
}

TeamMessageHandler::TeamMessageHandler() :
  theBNTP(theFrameInfo, theRobotInfo)
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
}

void TeamMessageHandler::update(BHumanMessageOutputGenerator& outputGenerator){
  
  outputGenerator.theBHumanArbitraryMessage.queue.clear();

  bool sendDueToConditionalEvent = theMessageManagement.lastEventTS > outputGenerator.timeLastSent;

  DEBUG_RESPONSE_ONCE("module:TeamMessageHandler:generateTCMPluginClass")
    teamCommunicationTypeRegistry.generateTCMPluginClass("BHumanStandardMessage.java", static_cast<const CompressedTeamCommunication::RecordType*>(teamMessageType));

  outputGenerator.sendThisFrame =
#ifndef SITTING_TEST
#ifdef TARGET_ROBOT
    theMotionRequest.motion != MotionRequest::playDead &&
    theMotionInfo.executedPhase != MotionPhase::playDead &&
#endif
#endif // !SITTING_TEST
    (
      theFrameInfo.getTimeSince(outputGenerator.timeLastSent) >= theMessageManagement.sendInterval ||      // periodic
      theFrameInfo.time < outputGenerator.timeLastSent ||    // for the first packet
      sendDueToConditionalEvent
    );
  // don't send ANYTHING if in out-of-packets emergency mode
  if (theMessageManagement.outOfPackets) {
    outputGenerator.sendThisFrame = false;
  }

  theRobotStatus.hasGroundContact = theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionRequest.motion != MotionRequest::getUp;
  theRobotStatus.isUpright = theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting;
  if(theRobotStatus.hasGroundContact)
    theRobotStatus.timeOfLastGroundContact = theFrameInfo.time;
  if(theRobotStatus.isUpright)
    theRobotStatus.timeWhenLastUpright = theFrameInfo.time;


  // if(sendDueToConditionalEvent == true || isDifferent(outputGenerator) || theFrameInfo.getTimeSince(timeLastSent) > 30000){
  //   outputGenerator.generate = [this, &outputGenerator,sendDueToConditionalEvent](RoboCup::SPLStandardMessage* const m)
  //   {
  //     generateMessage(outputGenerator);
  //     writeMessage(outputGenerator, m, sendDueToConditionalEvent);
  //   };
  // }

  outputGenerator.generate = [this, &outputGenerator,sendDueToConditionalEvent](RoboCup::SPLStandardMessage* const m)
  {
    generateMessage(outputGenerator);
    writeMessage(outputGenerator, m, sendDueToConditionalEvent);
  };
}

void TeamMessageHandler::generateMessage(BHumanMessageOutputGenerator& outputGenerator) const
{
#define SEND_PARTICLE(particle) \
  the##particle >> outputGenerator

  outputGenerator.theBSPLStandardMessage.playerNum = static_cast<uint8_t>(theRobotInfo.number);
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  outputGenerator.theBSPLStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  #endif

  #if BHUMAN_MESSAGE_INCLUDE_HEADER
  outputGenerator.theBHumanStandardMessage.magicNumber = Global::getSettings().magicNumber;
  #endif

  outputGenerator.theBHumanStandardMessage.timestamp = Time::getCurrentSystemTime();

  theRobotStatus.isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  // The other members of theRobotStatus are filled in the update method.
  theRobotStatus.sequenceNumbers.fill(-1);
  #if KEEP_FULL_ROBOT_STATUS
  for(const Teammate& teammate : theTeamData.teammates)
    theRobotStatus.sequenceNumbers[teammate.number - Settings::lowestValidPlayerNumber] = teammate.sequenceNumber;
  theRobotStatus.sequenceNumbers[theRobotInfo.number - Settings::lowestValidPlayerNumber] = outputGenerator.sentMessages % 15;
  #endif

  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  outputGenerator.theBSPLStandardMessage.fallen = !theRobotStatus.hasGroundContact || !theRobotStatus.isUpright;
  #endif

  outputGenerator.theBHumanStandardMessage.compressedContainer.reserve(SPL_STANDARD_MESSAGE_DATA_SIZE);
  CompressedTeamCommunicationOut stream(outputGenerator.theBHumanStandardMessage.compressedContainer, outputGenerator.theBHumanStandardMessage.timestamp,
                                        teamMessageType, !outputGenerator.sentMessages);
  outputGenerator.theBHumanStandardMessage.out = &stream;

  SEND_PARTICLE(BNTP);

  SEND_PARTICLE(RobotStatus);

  if(sendMirroredRobotPose)
  {
    RobotPose theMirroredRobotPose = theRobotPose;
    theMirroredRobotPose.translation *= -1.f;
    theMirroredRobotPose.rotation = Angle::normalize(theMirroredRobotPose.rotation + pi);
    SEND_PARTICLE(MirroredRobotPose);
  }
  else
    SEND_PARTICLE(RobotPose);
  
  // remember teamMessage.def exists, so only the fields specified there are sent
  SEND_PARTICLE(PlayerRole);  // SPQR

  FOREACH_TEAM_MESSAGE_REPRESENTATION(SEND_PARTICLE);

  // SEND_PARTICLE(FieldCoverage);

  //Send this last, because it is unimportant for robots, (so it is ok, if it gets dropped)
  // SEND_PARTICLE(RobotHealth);
  // SEND_PARTICLE(FieldFeatureOverview);   // Non è mai ricevuto?!

  outputGenerator.theBHumanStandardMessage.out = nullptr;

  #if INCLUDE_BHUMAN_ARBITRARY_MESSAGE
  outputGenerator.theBSPLStandardMessage.numOfDataBytes =
    static_cast<uint16_t>(outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage()
                          + outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage());
  #else
  outputGenerator.theBSPLStandardMessage.numOfDataBytes =
    static_cast<uint16_t>(outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage());
  #endif
}

void TeamMessageHandler::writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m, bool sendDueToConditionalEvent) const{ 
  string RobotNumber = to_string(theRobotInfo.number);
  string Filename = RobotNumber;
  ofstream SendPacketFile;
  SendPacketFile.open("/home/nao/logs/" + Filename,  std::ios::app);

  ASSERT(outputGenerator.sendThisFrame);

  outputGenerator.theBHumanStandardMessage.write(reinterpret_cast<void*>(m->data));

  const int offset = outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage();

  #if INCLUDE_BHUMAN_ARBITRARY_MESSAGE

  const int restBytes = SPL_STANDARD_MESSAGE_DATA_SIZE - offset;

  int sizeOfArbitraryMessage;
  if((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes)
  {
    OUTPUT_ERROR("outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage() > restBytes "
                 "-- with size of " << sizeOfArbitraryMessage << " and restBytes " << restBytes);

    do
      outputGenerator.theBHumanArbitraryMessage.queue.removeLastMessage();
    while((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes
          && !outputGenerator.theBHumanArbitraryMessage.queue.isEmpty());
  }

  ASSERT(sizeOfArbitraryMessage <= restBytes);

  outputGenerator.theBHumanArbitraryMessage.write(reinterpret_cast<void*>(m->data + offset));

  outputGenerator.theBSPLStandardMessage.numOfDataBytes = static_cast<uint16_t>(offset + sizeOfArbitraryMessage);

  #else

  outputGenerator.theBSPLStandardMessage.numOfDataBytes = static_cast<uint16_t>(offset);

  #endif

  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  outputGenerator.theBSPLStandardMessage.write(reinterpret_cast<void*>(&m->header[0]));
  #else
  outputGenerator.theBSPLStandardMessage.write(reinterpret_cast<void*>(&m->playerNum));
  #endif

  outputGenerator.sentMessages++;
  SendPacketFile << outputGenerator.sentMessages << " " << theFrameInfo.time << std::endl;

  SendPacketFile.close();

  // TEMP, TODO remove
  #if INCLUDE_BHUMAN_ARBITRARY_MESSAGE
  OUTPUT_TEXT("SIZE REPORT");
  OUTPUT_TEXT("theBHumanStandardMessage: " << outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage());
  OUTPUT_TEXT("theBHumanArbitraryMessage: " << outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage());
  OUTPUT_TEXT("theBSPLStandardMessage: " << outputGenerator.theBSPLStandardMessage.numOfDataBytes);
  //#else
  //OUTPUT_TEXT("MESSAGE BYTES: " << outputGenerator.theBSPLStandardMessage.numOfDataBytes);
  #endif

  outputGenerator.timeLastSent = theFrameInfo.time;    // SPQR simplified this, reasons for BHuman to do the other way are unclear.
                                      // At least this works with the striker sending at a different rate.
}

void TeamMessageHandler::update(TeamData& teamData)
{
  teamData.generate = [this, &teamData](const SPLStandardMessageBufferEntry* const m)
  {
    if(readSPLStandardMessage(m))
    {
      theBNTP << receivedMessageContainer;
      // Don't accept messages from robots to which we do not know a time offset yet.
      if(!theBNTP[receivedMessageContainer.theBSPLStandardMessage.playerNum]->isValid())
        return;

      return parseMessageIntoBMate(getBMate(teamData));
    }

    if(receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::myOwnMessage
#ifndef NDEBUG
       || receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::magicNumberDidNotMatch
#endif
      ) return;

    //the message had an parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruderAlert.wav"))
      timeWhenLastMimimi = theFrameInfo.time;

    ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
  };

  maintainBMateList(teamData);
}

void TeamMessageHandler::maintainBMateList(TeamData& teamData) const
{
  //@author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
  {
    // Iterate over deprecated list of teammate information and update some convenience information
    // (new information has already been coming via handleMessages)
    for(auto& teammate : teamData.teammates)
    {
      Teammate::Status newStatus = Teammate::PLAYING;
      if(teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        newStatus = Teammate::PENALIZED;
      else if(!teammate.isUpright || !teammate.hasGroundContact)
        newStatus = Teammate::FALLEN;

      if(newStatus != teammate.status)
      {
        teammate.status = newStatus;
        teammate.timeWhenStatusChanged = theFrameInfo.time;
      }

      teammate.isGoalkeeper = teammate.number == 1;
    }

    // Remove elements that are too old:
    auto teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout)
        teammate = teamData.teammates.erase(teammate);
      else
        ++teammate;
    }

    // Other stuff
    teamData.numberOfActiveTeammates = 0;
    teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(teammate->status != Teammate::PENALIZED)
        teamData.numberOfActiveTeammates++;
      ++teammate;
    }
  }
}

#define PARSING_ERROR(outputText) { OUTPUT_ERROR(outputText); receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::parsingError;  return false; }
bool TeamMessageHandler::readSPLStandardMessage(const SPLStandardMessageBufferEntry* const m)
{
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  if(!receivedMessageContainer.theBSPLStandardMessage.read(&m->message.header[0]))
    PARSING_ERROR("BSPL" " message part reading failed");
  #else
  if(!receivedMessageContainer.theBSPLStandardMessage.read(&m->message.playerNum))
    PARSING_ERROR("BSPL" " message part reading failed");
  #endif

#ifndef SELF_TEST
  if(receivedMessageContainer.theBSPLStandardMessage.playerNum == theRobotInfo.number)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage) && false;
#endif // !SELF_TEST

  if(receivedMessageContainer.theBSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.theBSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received " << receivedMessageContainer.theBSPLStandardMessage.playerNum);

  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER

  if(receivedMessageContainer.theBSPLStandardMessage.teamNum != static_cast<uint8_t>(Global::getSettings().teamNumber))
    PARSING_ERROR("Invalid team number received");

  #endif

  if(!receivedMessageContainer.theBHumanStandardMessage.read(m->message.data))
    PARSING_ERROR(BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

  #if BHUMAN_MESSAGE_INCLUDE_HEADER
  if(receivedMessageContainer.theBHumanStandardMessage.magicNumber != Global::getSettings().magicNumber)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch) && false;
  #endif

  #if INCLUDE_BHUMAN_ARBITRARY_MESSAGE
  const size_t offset = receivedMessageContainer.theBHumanStandardMessage.sizeOfBHumanMessage();
  if(!receivedMessageContainer.theBHumanArbitraryMessage.read(m->message.data + offset))
    PARSING_ERROR(BHUMAN_ARBITRARY_MESSAGE_STRUCT_HEADER " message part reading failed");
  #endif

  receivedMessageContainer.timestamp = m->timestamp;

  return true;
}

Teammate& TeamMessageHandler::getBMate(TeamData& teamData) const
{
  teamData.receivedMessages++;

  for(auto& teammate : teamData.teammates)
    if(teammate.number == receivedMessageContainer.theBSPLStandardMessage.playerNum)
      return teammate;

  teamData.teammates.emplace_back();
  return teamData.teammates.back();
}

#define RECEIVE_PARTICLE(particle) currentTeammate.the##particle << receivedMessageContainer
void TeamMessageHandler::parseMessageIntoBMate(Teammate& currentTeammate)
{
  currentTeammate.number = receivedMessageContainer.theBSPLStandardMessage.playerNum;

  receivedMessageContainer.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.timeWhenLastPacketSent = receivedMessageContainer.toLocalTimestamp(receivedMessageContainer.theBHumanStandardMessage.timestamp);
  currentTeammate.timeWhenLastPacketReceived = receivedMessageContainer.timestamp;

  CompressedTeamCommunicationIn stream(receivedMessageContainer.theBHumanStandardMessage.compressedContainer,
                                       receivedMessageContainer.theBHumanStandardMessage.timestamp, teamMessageType,
                                       [this](unsigned u) { return receivedMessageContainer.toLocalTimestamp(u); });
  receivedMessageContainer.theBHumanStandardMessage.in = &stream;

  RobotStatus robotStatus;
  robotStatus << receivedMessageContainer;

  currentTeammate.isPenalized = robotStatus.isPenalized;
  currentTeammate.isUpright = robotStatus.isUpright;
  currentTeammate.hasGroundContact = robotStatus.hasGroundContact;
  #if KEEP_FULL_ROBOT_STATUS
  currentTeammate.timeWhenLastUpright = robotStatus.timeWhenLastUpright;
  currentTeammate.timeOfLastGroundContact = robotStatus.timeOfLastGroundContact;
  currentTeammate.sequenceNumber = robotStatus.sequenceNumbers[currentTeammate.number - Settings::lowestValidPlayerNumber];
  currentTeammate.returnSequenceNumber = robotStatus.sequenceNumbers[theRobotInfo.number - Settings::lowestValidPlayerNumber];
  #endif

  RECEIVE_PARTICLE(RobotPose);

  // (PlayerRole cannot be received normally b/c Teammate intentionally only has role. So this is actually rather simple.)
  PlayerRole pr;
  pr << receivedMessageContainer;
  currentTeammate << pr;

  FOREACH_TEAM_MESSAGE_REPRESENTATION(RECEIVE_PARTICLE);
  // RECEIVE_PARTICLE(FieldCoverage);
  // RECEIVE_PARTICLE(RobotHealth);

  // Restore original representations from net-specific ones
  currentTeammate.theObstacleModel << currentTeammate.theDiscretizedObstacleModel;

  receivedMessageContainer.theBHumanStandardMessage.in = nullptr;

  #if INCLUDE_BHUMAN_ARBITRARY_MESSAGE
  receivedMessageContainer.theBHumanArbitraryMessage.queue.handleAllMessages(currentTeammate);
  #endif
}
