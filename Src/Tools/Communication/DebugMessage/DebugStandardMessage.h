/**
 * @file DebugMessageTCM.h
 *
 * Debug Message sent to TCM on external computer
 *
 * @author Eugenio Bugli
 */

#pragma once

#include "Tools/Communication/DebugMessageTCM.h"
#include "Tools/Streams/Streamable.h"

struct DebugStandardMessage : public DebugMessageTCM, public Streamable
{

  DebugStandardMessage() : DebugMessageTCM(), Streamable() {
    numOfDataBytes = 0;
  }

  // returns the size of this struct when it is written
  size_t sizeOfDebugMessage() const;

  // Method to convert this struct for communication usage
  // @param data point to dataspace,
  //        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBSPLMessage()
  // -asserts: writing sizeOfBHMessage() bytes
  void write(void* data) const;

  // Method to reads the message from data.
  // @param data the message
  // @return the header and the versions are convertible
  bool read(const void* data);

protected:
  void read(In& stream) override;
  void write(Out& stream) const override;

private:
  static void reg();

};