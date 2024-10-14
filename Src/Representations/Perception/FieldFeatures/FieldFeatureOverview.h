/**
 * @file FieldFeatureOverview.h
 * Declaration of a struct gives an overview over the percepted fieldFeature.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"
#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/EnumIndexedArray.h"

// Used to be sent over arbitrary message, but not anymore as of 2023.
// See any version of our (SPQR) code during RoboCup 2024 to recover communication code.
STREAMABLE(FieldFeatureOverview,
{
  ENUM(Feature,
  {,
    penaltyArea,
    midCircle,
    penaltyMarkWithPenaltyAreaLine,
  });

  STREAMABLE_WITH_BASE(FieldFeatureStatus, Pose2f,
  {
    /**
     * Assignment operator for Pose2f objects.
     * @param other A Pose2f object
     * @return A reference to the object after the assignment
     */
    FieldFeatureStatus& operator=(const Pose2f& other)
    {
      static_cast<Pose2f&>(*this) = other;
      return *this;
    };
    ,
    (bool)(false) isValid,       //< Seen in current Frame
    (unsigned)(0) lastSeen,      //< the timestamp, when this pose was valid
  });
  void draw() const,

  (FieldFeatureStatus) combinedStatus, // Pose will not be set
  (ENUM_INDEXED_ARRAY(FieldFeatureStatus, Feature)) statuses,
});
