/**
 * @file DebugMessageTCM.h
 *
 * Debug Message sent to TCM on external computer
 *
 * @author Eugenio Bugli
 */

#ifndef DEBUGMESSAGE_H
#define DEBUGMESSAGE_H
#include <stdint.h>
#include <vector>

#define DEBUG_MESSAGE_HEADER "SPQR"
#define DEBUG_MESSAGE_STRUCT_VERSION     4  // GAMECONTROLLER_RETURN_STRUCT_VERSION
#define MAX_OBSTACLES 20 // max number of obstacles

struct DebugMessageTCM {
    
    char header[4];  // DEBUG_MESSAGE_HEADER
    uint8_t version; // DEBUG_MESSAGE_STRUCT_VERSION

    uint8_t playerNum; 
    uint8_t teamNum;
    uint8_t fallen; // 1 if fallen, 0 otherwise

    float pose[3]; // coordinates in millimeters (x,y,theta)
    float ballAge; //seconds since this robot last saw the ball (-1.f if we haven't seen it)
    float ball[2]; // position of the ball relative to the robot

    uint16_t numOfDataBytes;

    uint8_t role; // current role of the robot

    uint8_t currentObsSize;

    uint8_t obsTypes[MAX_OBSTACLES]; // contains the type of the obstacles

    float obsCenters[2*MAX_OBSTACLES];  // contains both x and y position of the centers of each obstacle  x|y
    
    unsigned int obsLastSeen[MAX_OBSTACLES]; // contains the timestamp of the last measurement

    uint16_t messageBudget; // useful for challenge 24
    uint16_t secsRemaining; // useful for challenge 24

    uint8_t armContact[2]; // 0 if no contact

    uint8_t armPushDirection[2]; //  direction in which the arm is being pushed 

    unsigned int armTimeOfLastContact[2]; 

    unsigned int timeSinceLastWhistleDetection; // timestamp last detection
    uint8_t whistleDetectionState; // whistle detect
    
    float teamBall[2];
    float teamBallVel[2];

    float batteryLevel;
    float cpuTemperature;

    uint8_t refereeDetection; // referee pose 

    #ifdef __cplusplus
        // constructor
        DebugMessageTCM() :
            version(DEBUG_MESSAGE_STRUCT_VERSION),
            playerNum(0),
            teamNum(0),
            fallen(255), // placeholder to detect in the TCM if this field is not set
            ballAge(-1.f),
            numOfDataBytes(0),
            role(0),
            currentObsSize(0),
            messageBudget(0),
            secsRemaining(0),
            timeSinceLastWhistleDetection(0),
            whistleDetectionState(0),
            batteryLevel(0),
            cpuTemperature(0),
            refereeDetection(0)
        {
            const char* init = DEBUG_MESSAGE_HEADER;
            for(unsigned int i = 0; i < sizeof(header); ++i)
                header[i] = init[i];

            pose[0] = 0.f;
            pose[1] = 0.f;
            pose[2] = 0.f;
            ball[0] = 0.f;
            ball[1] = 0.f;

            armContact[0] = 0;
            armContact[1] = 0;

            armPushDirection[0] = 0;
            armPushDirection[1] = 0;

            armTimeOfLastContact[0] = 0;
            armTimeOfLastContact[1] = 0;

            teamBall[0] = 0.f;
            teamBall[1] = 0.f;
            teamBallVel[0] = 0.f;
            teamBallVel[1] = 0.f;

            for(unsigned int i = 0; i < MAX_OBSTACLES; ++i) {
                obsTypes[i] = 0;
                obsLastSeen[i] = 0;
            }
        }

    #endif
};
#endif