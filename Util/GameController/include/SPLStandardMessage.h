#ifndef SPLSTANDARDMESSAGE_H
#define SPLSTANDARDMESSAGE_H

#include <stdint.h>

#define SPL_STANDARD_MESSAGE_STRUCT_HEADER  "SPL "
#define SPL_STANDARD_MESSAGE_STRUCT_VERSION 7

#define SPL_STANDARD_MESSAGE_DATA_SIZE      126
                                         // was 474, now 128 by rules

/*
 * Important remarks about units:
 *
 * For each parameter, the respective comments describe its unit.
 * The following units are used:
 *
 * - Distances:  Millimeters (mm)
 * - Angles:     Radian
 * - Time:       Seconds (s)
 */
struct SPLStandardMessage
{
  uint8_t playerNum;

  // number of bytes that is actually used by the data array
  uint8_t numOfDataBytes;

  // buffer for arbitrary data, teams do not need to send more than specified in numOfDataBytes
  uint8_t data[SPL_STANDARD_MESSAGE_DATA_SIZE];

#ifdef __cplusplus
  // constructor
  SPLStandardMessage() :
    playerNum(0),
    numOfDataBytes(0)
  {;}
#endif
};

#endif // SPLSTANDARDMESSAGE_H
