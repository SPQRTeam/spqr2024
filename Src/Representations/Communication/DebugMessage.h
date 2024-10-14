/**
 * @file DebugMessage.h
 *
 * @author Eugenio Bugli
 */

#pragma once

#include "Tools/Communication/DebugMessage/DebugStandardMessage.h"
#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * A representation of DebugMessageTCM a B-Human robot would send.
 */
STREAMABLE(DebugMessage,
{
  virtual unsigned toLocalTimestamp(unsigned remoteTimestamp) const,

  (unsigned) timestamp,
  (DebugStandardMessage) theDebugStandardMessage,
});

inline unsigned DebugMessage::toLocalTimestamp(unsigned) const { return 0u; }

/**
 * A struct that can generate the DebugMessage,
 *     that this robot wants to send to the TCM.
 */
STREAMABLE_WITH_BASE(DebugMessageOutputGenerator, DebugMessage,
{
  FUNCTION(void(DebugMessageTCM* const m)) generate,
  (bool)(false) sendThisFrame, //< should send this frame -> is allowed to send this frame considering the spl rules
});
