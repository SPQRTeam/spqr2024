/**
 * @file AudioRecorder.h
 * This file declares a module that provides audio samples using PortAudio.
 */

#pragma once

#include <portaudio.h>
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <alsa/asoundlib.h>
#include <mutex>

MODULE(AudioRecorder,
{,
  REQUIRES(FrameInfo),
  PROVIDES(AudioData),
  LOADS_PARAMETERS(
  {,
    (std::string)("") deviceName, /**< Name of audio device. */
    (unsigned)(4) channels, /**< Number of channels to capture. */
    (unsigned)(22050) sampleRate, /**< Sample rate to capture. */
    (float)(0.1f) latency, /**< Latency in seconds. */
  }),
});

class AudioRecorder : public AudioRecorderBase {
    private:
        void update(AudioData& audioData) override;
        
        PaStream* stream = nullptr;
        PaStreamParameters inputParameters;
        unsigned noDataCount = 0;
        static std::mutex mutex;

        int counter = 0;

    public:
        AudioRecorder();
        ~AudioRecorder();
        
        /**
         * @brief Checks if a PortAudio error occurred. If so, it prints the error
         * message and exits the program.
         *
         * @param err PortAudio error code
        */
        void checkErr(PaError err);
};

