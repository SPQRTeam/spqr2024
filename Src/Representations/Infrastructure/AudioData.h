/**
 * @file AudioData.h
 * The file declares a struct that stores audio data of up to four channels.
 * 0: back left microphone
 * 1: back right microphone
 * 2: front left microphone
 * 3: front right microphone
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(AudioData,
{
  using Sample = float,

  (unsigned)(4) channels,
  (unsigned)(8000) sampleRate,
  (std::vector<Sample>) samples, /**< Samples are interleaved. */
  (bool)(false) isValid,
  (std::string)("default") device,
  (std::string)("portaudio") api,
  (double)(0.0) latency,
  (int)(512) framesPerBuffer,
});