/**
 * @file RoleAndContext.cpp
 */

#include "RoleAndContext.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Blackboard.h"

//#define PLOT_SINGE_TSL(name) \
//  PLOT("representation:FieldFeatureOverview:TimeSinceLast:" #name, theFrameInfo.getTimeSince(statuses[name].lastSeen));
//



void RoleAndContext::operator >> (BHumanMessage& m) const
{ 
  
  
  // m.theBSPLStandardMessage.role = myRole;
  
  
  //m.theBSPLStandardMessage.role = (float)role;
  //std::cout<<"role ="<<role<<std::endl;
  //m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
}

bool RoleAndContext::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());


  m.bin >> myRole;
  m.bin >> atk_def_Context;
  m.bin >> ball_holding_Context;
  m.bin >> winning_losing_Context;
  m.bin >> robots_number_Context;
  m.bin >> robot_action_number_Context;
  


  return true;
}
