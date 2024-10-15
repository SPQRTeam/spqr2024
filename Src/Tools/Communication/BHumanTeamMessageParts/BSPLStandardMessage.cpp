/**
 * @file BSPLStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */
#include "BSPLStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Tools/FunctionList.h"
#include <cstddef>

size_t BSPLStandardMessage::sizeOfBSPLMessage() const
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  return offsetof(RoboCup::SPLStandardMessage, data);
}

void BSPLStandardMessage::write(void* data) const
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  memcpy(data, &header, sizeOfBSPLMessage());
  #else
  memcpy(data, &playerNum, sizeOfBSPLMessage());
  #endif
}

bool BSPLStandardMessage::read(const void* data)
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER

  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != *reinterpret_cast<const char*&>(data)++)
      return false;

  version = *reinterpret_cast<const uint8_t*&>(data)++;

  if(version != SPL_STANDARD_MESSAGE_STRUCT_VERSION)
    return false;

  memcpy(reinterpret_cast<void*>(&playerNum), data, sizeOfBSPLMessage() - (sizeof(header) + sizeof(version)));

  #else

  memcpy(reinterpret_cast<void*>(&playerNum), data, sizeOfBSPLMessage());

  #endif


  return true;
}

void BSPLStandardMessage::read(In& stream)
{
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  std::string headerRef(header, sizeof(header));
  STREAM(headerRef);// does not allow to change the header in any case, but makes it visible in a great way
  STREAM(version);
  #endif
  STREAM(playerNum);
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  STREAM(teamNum);
  STREAM(fallen);
  STREAM(pose);
  STREAM(ballAge);
  STREAM(ball);
  #endif
  STREAM(numOfDataBytes);
}

void BSPLStandardMessage::write(Out& stream) const
{
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  std::string headerRef(header, sizeof(header));
  STREAM(headerRef);// does not allow to change the header in any case, but makes it visible in a great way
  STREAM(version);
  #endif
  STREAM(playerNum);
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  STREAM(teamNum);
  STREAM(fallen);
  STREAM(pose);
  STREAM(ballAge);
  STREAM(ball);
  #endif
  STREAM(numOfDataBytes);
}

void BSPLStandardMessage::reg()
{
  PUBLISH(reg);
  REG_CLASS(BSPLStandardMessage);
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  REG(std::string, headerRef);
  REG(version);
  #endif
  REG(playerNum);
  #if SPL_MESSAGE_INCLUDE_STANDARD_HEADER
  REG(teamNum);
  REG(fallen);
  REG(pose);
  REG(ballAge);
  REG(ball);
  #endif
  REG(numOfDataBytes);
}
