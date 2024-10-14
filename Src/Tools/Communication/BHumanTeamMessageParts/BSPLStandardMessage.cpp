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

  memcpy(data, &playerNum, sizeOfBSPLMessage());
}

bool BSPLStandardMessage::read(const void* data)
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  memcpy(reinterpret_cast<void*>(&playerNum), data, sizeOfBSPLMessage());

  return true;
}

void BSPLStandardMessage::read(In& stream)
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  STREAM(playerNum);
  STREAM(numOfDataBytes);
}

void BSPLStandardMessage::write(Out& stream) const
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  STREAM(playerNum);
  STREAM(numOfDataBytes);
}

void BSPLStandardMessage::reg()
{
  PUBLISH(reg);
  REG_CLASS(BSPLStandardMessage);
  REG(playerNum);
  REG(numOfDataBytes);
}
