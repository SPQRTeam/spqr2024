/**
 * @file Modules/Modeling/MessageManager/MessageManager.cpp
 *
 * Implementation of the .h
 * 
 * INCOMING CHANGES
 * - Nuovo evento di Graziano
 * 
 * @author Francesco Petri
 */

#include "MessageManager.h"

MessageManager::MessageManager() {
}

void MessageManager::update(MessageManagement& mm){
  ///// those who see the ball send at a higher rate
  // (also those who heard a whistle, b/c the set->play switch is not immediate for some reason)

  if(theGameInfo.state != STATE_PLAYING){
    if (thePlayerRole.isGoalkeeper()) {
      mm.sendInterval = sendIntervalReadyStateDefenders;   // was sendIntervalGoalie;
    }
    else if(theRobotPose.translation.x() < -1500.f) {
      mm.sendInterval = sendIntervalReadyStateDefenders;
    }
    else
      mm.sendInterval = sendIntervalReadyStateAttackers;
  }
  else {
    unsigned timeWhenTeammateLastSawBall = 0;
    for (Teammate mate : theTeamData.teammates) {
      unsigned twls = mate.theBallModel.timeWhenLastSeen;
      if (twls > timeWhenTeammateLastSawBall) {
        timeWhenTeammateLastSawBall = twls;
      }
    }
    bool iSawBall = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < ballSeenTimeThreshold;
    bool nooneElseSawBall = theFrameInfo.getTimeSince(timeWhenTeammateLastSawBall) > sendIntervalIfBallSeen;
    bool iHeardWhistle = theFrameInfo.getTimeSince(lastWhistleEventTS) < sendIntervalIfBallSeen + 1000;
    if ( (iSawBall && nooneElseSawBall) || iHeardWhistle ) {
      mm.sendInterval = sendIntervalIfBallSeen;
    }
    else if (thePlayerRole.isGoalkeeper()){
      mm.sendInterval = sendIntervalGoalie;
    }
    else {
      float rate = sendIntervalIfNoBall;
      
      #ifdef TARGET_ROBOT
      int16_t secondsRemaining = (theGameInfo.firstHalf ? theGameInfo.secsRemaining+600 : theGameInfo.secsRemaining);
      int numAvailablePackets = (theOwnTeamInfo.messageBudget - outOfPacketsThreshold) -  // packets remaining  
                       (secondsRemaining/4) -  // packets allocated for striker
                       (secondsRemaining/20);  // packets allocated for goalie
      int numActivePlayers = theTeamData.numberOfActiveTeammates;
      if(numAvailablePackets > 0)
        rate = (secondsRemaining * numActivePlayers / numAvailablePackets) * 1000;
      OUTPUT_TEXT("SecsRemaining:   " << secondsRemaining);
      OUTPUT_TEXT("numAvailablePackets:     " << numAvailablePackets);
      OUTPUT_TEXT("numActivePlayers:      " << numActivePlayers);
      
      #endif

      mm.sendInterval = rate;

    }
  }

  ///// check conditional events
  bool is_searcher = thePlayerRole.role==PlayerRole::activeSearcher || thePlayerRole.role==PlayerRole::passiveSearcher;

  // striker sends a message when it detects a goal (UNUSED)
  bool conditionalEvent_strikerDetectedGoal = (theWhistle.lastTimeGoalDetected > mm.lastEventTS && (thePlayerRole.role == PlayerRole::striker || thePlayerRole.role == PlayerRole::activeSearcher));
  // anyone heard a whistle
  bool conditionalEvent_whistle = theWhistle.lastTimeWhistleDetected > mm.lastEventTS && theFrameInfo.getTimeSince(lastWhistleEventTS) > whistleEventCooldown;
  // anyone sends a message when it exits search mode after seeing the ball
  bool conditionalEvent_searcherSawBall = (was_searcher && !is_searcher && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 500);
  // gesture check
  bool conditionalEvent_readyGesture = (theRefereeEstimator.timeOfLastDetection > mm.lastEventTS && theFrameInfo.getTimeSince(lastGestureDetectedTS) > readyGestureEventCooldown);
  // but goalie is never searcher, so it gets a special event
  bool conditionalEvent_goalieSawBall = (
    thePlayerRole.isGoalkeeper() &&
    theFrameInfo.getTimeSince(ballModel_timeLastSeen_prev) > 8000 &&
    theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 500
  );
  // EVERYONE, except the goalie, sends a message after the ball is first seen by the team after being unseen for a while
  bool conditionalEvent_teamSawBallAfterAWhile = (
    !thePlayerRole.isGoalkeeper() &&
    !teamBall_isValid_prev &&
    theTeamBallModel.isValid &&
    theFrameInfo.getTimeSince(teamBall_timeLastSeen_prev) > teamBallNotSeenThreshold &&
    theFrameInfo.getTimeSince(lastTeamBallEventTS) > teamBallNotSeenThreshold
  );
  if (conditionalEvent_whistle) {
    lastWhistleEventTS = theFrameInfo.time;
  }
  if (conditionalEvent_teamSawBallAfterAWhile) {
    lastTeamBallEventTS = theFrameInfo.time;
  }
  if (conditionalEvent_readyGesture) {
    lastGestureDetectedTS = theFrameInfo.time;
  }

  // so we have only one place where to list conditional events
  bool anyConditionalEvent = false;
  //  // conditionalEvent_strikerDetectedGoal ||
  //  conditionalEvent_whistle ||
  //  conditionalEvent_searcherSawBall ||
  //  conditionalEvent_goalieSawBall ||
  //  conditionalEvent_teamSawBallAfterAWhile ||
  //  conditionalEvent_readyGesture
  //);

  // Fire event!
  if (anyConditionalEvent) {
    mm.lastEventTS = theFrameInfo.time;
    // SystemCall::say("Zan zan zan");
  }
  
  // if (conditionalEvent_strikerDetectedGoal) {anyConditionalEvent
  //   SystemCall::say("Goal event triggered");
  // }

  // if (conditionalEvent_searcherSawBall) {
  //   SystemCall::say("Searcher event activated");
  // }


  ///// notify the emergency that there are no packets left
  // ...but only for real robots w/ a real gamecontroller, otherwise it's a strikerfest in the sim
  #ifdef TARGET_ROBOT
  mm.outOfPackets = theOwnTeamInfo.messageBudget <= outOfPacketsThreshold;
  #else
  mm.outOfPackets = false;
  #endif

  was_searcher = is_searcher;
  ballModel_timeLastSeen_prev = theBallModel.timeWhenLastSeen;
  teamBall_isValid_prev = theTeamBallModel.isValid;
  teamBall_timeLastSeen_prev = theTeamBallModel.timeWhenLastSeen;
}

MAKE_MODULE(MessageManager, modeling);