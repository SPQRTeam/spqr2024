/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(Whistle, COMMA public BHumanCompressedMessageParticle<Whistle>
{
    ENUM(DetectionState,
    {,
        dontKnow,
        notDetected,
        isDetected,
    });

    ENUM(Microphone,
    {,
        rearLeft,
        rearRight,
        frontLeft,
        frontRight,
    });
    ,
    (float)(0) confidenceOfLastWhistleDetection,            /**< Confidence based on hearing capability. */
    (unsigned char)(0) channelsUsedForWhistleDetection,     /**< Number of channels the robot used to listen. */
    (Vector2f)  lastLocalWhistlePosition,                   /** Position of the last detected whistle (NOTICE: only the direction is reliable) */
    (unsigned)(0) lastTimeGoalDetected,                     /**< Timestamp when a goal was detected. Only the striker has a relevant value here. */
    (unsigned) goalScoredPacketTimeout,                     /**< Exposes the cfg param of the WhistleRecognizer to the others */
    
    // New stuff for WhistleNet
    (Microphone)(rearLeft) currentMic,
    (int)(0) minFrequency,
    (int)(0) maxFrequency,
    (int)(0) detectedWhistleFrequency,
    (float)(0.f) lastConfidence,

    //STUFF WE NEED TO SEND TO TO THE TEAMMATES
    (DetectionState)(dontKnow) detectionState,              /**< Was detected? */
    (unsigned int)(0)  lastTimeWhistleDetected,             /**< Timestamp */
});