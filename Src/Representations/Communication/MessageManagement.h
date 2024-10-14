/**
 * @file MessageManagement.h
 *
 * Which dictates some rules to be followed by the TeamMessageHandler.
 *
 * @author Francesco Petri
 */

#pragma once

//disabling warnings while importing so I don't see irrelevant messages when compiling
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wenum-enum-conversion"
#pragma GCC diagnostic ignored "-Wenum-float-conversion"
#pragma GCC diagnostic ignored "-Wunused-function"

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <iostream>

//see above
#pragma GCC diagnostic pop

STREAMABLE(MessageManagement,
{,
  (int)(10000) sendInterval,
  (unsigned)(0) lastEventTS,
  (bool)(false) outOfPackets,
});
