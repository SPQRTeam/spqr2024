/**
 * @file TeamPossesion.h
 * @author Daniele Affinita

 This representation holds useful information to determine whether our team
 is in ball possession or not. 
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/BHMath.h"

STREAMABLE(TeamPossession,
{
    ,
    (float) (0) (possessionBalance),
    (float) (0) (laggyPossessionBalance),
});
