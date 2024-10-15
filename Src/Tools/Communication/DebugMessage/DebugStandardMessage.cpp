/**
 * @file DebugStandardMessage.cpp
 *
 * @author Eugenio Bugli
 */
#include "DebugStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Tools/FunctionList.h"
#include <cstddef>

size_t DebugStandardMessage::sizeOfDebugMessage() const
{
  return sizeof(struct DebugMessageTCM);
}

void DebugStandardMessage::write(void* data) const
{
  memcpy(data, &header, sizeOfDebugMessage());
}

bool DebugStandardMessage::read(const void* data)
{
  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != *reinterpret_cast<const char*&>(data)++)
      return false;

  version = *reinterpret_cast<const uint8_t*&>(data)++;

  if(version != DEBUG_MESSAGE_STRUCT_VERSION)
    return false;

  memcpy(reinterpret_cast<void*>(&playerNum), data, sizeOfDebugMessage() - (sizeof(header) + sizeof(version)));

  return true;
}

void DebugStandardMessage::read(In& stream)
{
  std::string headerRef(header, sizeof(header));
  STREAM(headerRef);// does not allow to change the header in any case, but makes it visible in a great way
  STREAM(version);
  STREAM(playerNum);
  STREAM(teamNum);
  STREAM(fallen);
  STREAM(pose);
  STREAM(ballAge);
  STREAM(ball);
  STREAM(numOfDataBytes);
  STREAM(role);
  STREAM(currentObsSize);
  STREAM(obsTypes);
  STREAM(obsCenterX);
  STREAM(obsCenterY);
  STREAM(obsLastSeen);
  STREAM(messageBudget);
  STREAM(secsRemaining);
  STREAM(armContact);
  STREAM(armPushDirection);
  STREAM(armTimeOfLastContact);
  STREAM(teamBall);
  STREAM(teamBallVel);
}

void DebugStandardMessage::write(Out& stream) const
{
  std::string headerRef(header, sizeof(header));
  STREAM(headerRef);// does not allow to change the header in any case, but makes it visible in a great way
  STREAM(version);
  STREAM(playerNum);
  STREAM(teamNum);
  STREAM(fallen);
  STREAM(pose);
  STREAM(ballAge);
  STREAM(ball);
  STREAM(numOfDataBytes);
  STREAM(role);
  STREAM(currentObsSize);
  STREAM(obsTypes);
  STREAM(obsCenterX);
  STREAM(obsCenterY);
  STREAM(obsLastSeen);
  STREAM(messageBudget);
  STREAM(secsRemaining);
  STREAM(armContact);
  STREAM(armPushDirection);
  STREAM(armTimeOfLastContact);
  STREAM(teamBall);
  STREAM(teamBallVel);
}

void DebugStandardMessage::reg()
{
  PUBLISH(reg);
  REG_CLASS(DebugStandardMessage);
  REG(std::string, headerRef);
  REG(version);
  REG(playerNum);
  REG(teamNum);
  REG(fallen);
  REG(pose);
  REG(ballAge);
  REG(ball);
  REG(numOfDataBytes);
  REG(role);
  REG(currentObsSize);
  REG(obsTypes);
  REG(obsCenterX);
  REG(obsCenterY);
  REG(obsLastSeen);
  REG(messageBudget);
  REG(secsRemaining);
  REG(armContact);
  REG(armPushDirection);
  REG(armTimeOfLastContact);
  REG(teamBall);
  REG(teamBallVel);
}
