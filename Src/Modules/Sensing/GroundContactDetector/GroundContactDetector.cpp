/**
 * @file GroundContactDetector.cpp
 * Implementation of a module that detects ground contact based on FSR measurements.
 * @author Thomas Röfer
 */

#include "GroundContactDetector.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(GroundContactDetector, sensing);

void GroundContactDetector::update(GroundContactState& groundContactState)
{
  if(groundContactState.contact)
  {
    if(theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] >= minPressureToKeepContact)
      lastTimeWithPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithPressure) < maxTimeWithoutPressure;
    if(!groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot) {
      if (use_pikmin_sounds) {
        //SystemCall::playSound("high_pikmin.wav");
        SystemCall::playSound("lasciami.wav");
      }
      else {
        SystemCall::say("High");
      }
    }
  }
  else
  {
    if(std::abs(theInertialSensorData.gyro.y()) > maxGyroYToRegainContact
       || theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] < minPressureToRegainContact
       || theFsrSensorData.totals[Legs::left] < minPressurePerFootToRegainContact
       || theFsrSensorData.totals[Legs::right] < minPressurePerFootToRegainContact)
      lastTimeWithoutPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithoutPressure) > minTimeWithPressure;
    if(groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot) {
      if (use_pikmin_sounds) {
        SystemCall::playSound("ground_pikmin.wav");
      }
      else {
        SystemCall::say("Ground");
      }
    }
  }
}
