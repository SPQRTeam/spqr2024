#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(RefereeEstimator, COMMA public BHumanCompressedMessageParticle<RefereeEstimator>
{,
    (unsigned int)(0) timeOfLastDetection,
    (unsigned char)(0x0) measures,
    (bool)(false) isDetected,
});
