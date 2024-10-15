/**
 * @file PlayerRole.cpp
 *
 * @author Arne Hasselbring
 */

#include "PlayerRole.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/Debugging.h"

std::string PlayerRole::getName() const
{
  return TypeRegistry::getEnumName(role);
}

// Removed b/c now the role is sent in the standard message.

// void PlayerRole::operator>>(BHumanMessage& m) const
// {
//   unsigned size_before = m.theBHumanArbitraryMessage.sizeOfArbitraryMessage();

//   m.theBHumanArbitraryMessage.queue.out.bin << role;

//   m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());

//   unsigned size_after = m.theBHumanArbitraryMessage.sizeOfArbitraryMessage();
//   OUTPUT_TEXT("role size-" << (size_after - size_before));
// }

// PlayerRole::RoleType PlayerRole::specialHandlingOfArbitraryMessage(InMessage& m)
// {
//   ASSERT(m.getMessageID() == id());

//   RoleType receivedRole;
//   m.bin >> receivedRole;

//   return receivedRole;
// }

int PlayerRole::getUtilityMatrixRowIndex(PlayerRole::RoleType role){
  switch (role)
  {
  case striker:
    return 2;
    break;
  case defenderone:
    return 3;
    break;
  case defendertwo:
    return 4;
    break;
  case supporter:
    return 5;
    break;
  case jolly:
    return 6;
    break;
  case libero:
    return 7;
    break;
  default:
    ASSERT(0 && " Role not used in Utility Matrix Computation");
    return -1;
    break;
  }
};

