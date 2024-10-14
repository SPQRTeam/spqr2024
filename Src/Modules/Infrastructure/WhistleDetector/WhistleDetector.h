#pragma once

#include <kissfft/kiss_fft.h>

#include "Tools/Module/Module.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <cstdio>
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Platform/SystemCall.h"
#include <string>
#include "Tools/Math/Constants.h"
#include "Tools/OnnxHelper/OnnxHelper.h"
#include "Tools/RingBufferWithSum.h"
#include <onnxruntime_cxx_api.h>


MODULE(WhistleDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(AudioData),
  REQUIRES(MotionInfo),
  PROVIDES(Whistle),
  LOADS_PARAMETERS(
  {,
    (std::string) whistleNetPath,
    (unsigned int) micBrokenThreshold,
    (float) limit,
    (float) threshold,
    (bool) useAdaptiveThreshold,
    (int) adaptiveWindowSize,
    (float) nnWeight,
    (float) pmWeight,
    (float) limitWeight,
    (bool) useWeightedPMConfidence,
    (int) release,
    (int) attack,
    (int) attackTimeout, // in ms
    (bool) useHannWindowing,
    (bool) useNuttallWindowing,
    (int) minFreq, 
    (int) maxFreq,
    (bool) freqCalibration,
    (bool) eval
  }),
});


class WhistleDetector : public WhistleDetectorBase {

  std::string oldWhistleNetPath;

  // onnx stuff
  std::array<float, 513> input{};
  std::array<float, 1> output{};
  OnnxHelper<float, float> *whistleNet;

  //physical model detection variables
  int windowSize = 0;
  int ampSize = 0;
  int peakPos = -1;
  int overtonePeakPos = -1;
  float currentMinAmp = 0;
  float currentMaxAmp = 0;
  float currentMeanAmp = 0;
  int channels = 0;
  int sampleRate = 0;

  float pmConfidence = 0.f; //confidence of overtone peak detection
  float nnConfidence = 0.f; //confidence of neural network
  float relLimitCount = 0.f;

  RingBufferWithSum<float, 20> thresholdBuffer;
  RingBufferWithSum<float, 10> confidenceBuffer;
  int prevAdaptiveWindowSize = 20;

  unsigned int currentMic = 0;
  unsigned int micBrokenCount = 0;
  bool alert = false;

  unsigned int ringPos, samplesLeft;
  unsigned int releaseCount, attackCount;
  int detectedWhistleFrequency = 0;
  RingBufferWithSum<int, 10> whistleFreqBuffer;
  int currentMinFreq = 0;
  int currentMaxFreq = 0;
  int oldMinFreq = 0;
  int oldMaxFreq = 0;
  std::vector<float> buffer;
  std::vector<kiss_fft_cpx> in, out;
  std::vector<float> amplitudes;
  std::vector<float> gradients;
  RingBufferWithSum<float, 200> maxAmpHist;

  unsigned lastAttackTime = 0;
  bool detectionProcessed = false;
  bool evaluationStarted = false;

public:
  WhistleDetector();
  void update(Whistle& whistle);

private:
  void setup();

  void detect_whistle(Whistle& whistle);

  //computes FFT with kissfft (populates out buffer)
  void compute_fft();

  //compute amplitudes and returns the amplitude sum
  float compute_amplitudes();

  //detects first overtone peak among the last 200 amplitudes (populated pm_confidence)
  void overtone_detection();

  //switches to another mic if the current mic is probably broken
  void checkMics(float ampSum);
};