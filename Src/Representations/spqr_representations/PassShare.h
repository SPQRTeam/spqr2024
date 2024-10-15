/**
 * @file PassShare.h
 * PassShare representation
 */

// TODO Uncomment the BHumanMessageParticle functions below

#pragma once


#include "Tools/Math/Pose2f.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"

STREAMABLE(PassShare, COMMA public PureBHumanArbitraryMessageParticle<idPassShare>
{
  void draw() const;

  /** BHumanMessageParticle functions */
  // void operator >> (BHumanMessage& m) const override;
  /* bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override*/ ,

  (bool) GRAPHICAL_DEBUG,

  (float) MAX_ALLOWED_OPPONENT_DISTANCE_FROM_PASS_TARGET,

  (Pose2f) passTarget,
  (int) myNumber,
  (int) readyReceive,
  (int) readyPass,
  (bool) tooManyOpponentsNearTarget,

  //(float) sharedGoalUtil,
  (int) (0) passingTo,
  (int) role,

});
/*
STREAMABLE(PassShareDebug,
{
  void draw() const,
});*/