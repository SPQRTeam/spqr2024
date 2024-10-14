#include "WhistleDetector.h"
#include "Tools/Debugging/Annotation.h"
#include "Platform/File.h"
#include <iostream>
using std::cout, std::endl;

WhistleDetector::WhistleDetector() {
  setup();

  ringPos = 0;

  releaseCount = static_cast<unsigned int>(release);
  attackCount = 1;

  thresholdBuffer.reserve(adaptiveWindowSize);
  confidenceBuffer.reserve((size_t)std::ceil(adaptiveWindowSize / 2));
  prevAdaptiveWindowSize = adaptiveWindowSize;

  currentMinFreq = minFreq;
  currentMaxFreq = maxFreq;
  oldMinFreq = minFreq;
  oldMaxFreq = maxFreq;
  detectedWhistleFrequency = minFreq + ((maxFreq - minFreq) / 2);
}

void WhistleDetector::update(Whistle& whistle){

  sampleRate = theAudioData.sampleRate;
  channels = theAudioData.channels;

  if (adaptiveWindowSize != prevAdaptiveWindowSize) {
    thresholdBuffer.reserve(adaptiveWindowSize);
    thresholdBuffer.clear();
    confidenceBuffer.reserve((size_t)std::ceil(adaptiveWindowSize / 2));
    confidenceBuffer.clear();
    prevAdaptiveWindowSize = adaptiveWindowSize;
  }

  if (oldMinFreq != minFreq) {
    oldMinFreq = minFreq;
    currentMinFreq = minFreq;
    whistleFreqBuffer.clear();
  }

  if (oldMaxFreq != maxFreq) {
    oldMaxFreq = maxFreq;
    currentMaxFreq = maxFreq;
    whistleFreqBuffer.clear();
  }

  if (oldWhistleNetPath != whistleNetPath)
    setup();

  if (!useAdaptiveThreshold) {
    thresholdBuffer.reserve(1);
    confidenceBuffer.reserve(1);
  }

  peakPos = -1;
  overtonePeakPos = -1;
  currentMinAmp = 0;
  currentMaxAmp = 0;
  currentMeanAmp = 0;

  if (theAudioData.isValid) {
    detect_whistle(whistle);
  } else {
    whistle.detectionState = Whistle::DetectionState::dontKnow;
    attackCount = 1;
  }
  whistle.currentMic = static_cast<Whistle::Microphone>(currentMic);
}

void WhistleDetector::detect_whistle(Whistle& whistle) {
  unsigned int audioDataPos = currentMic;

  while (audioDataPos < theAudioData.samples.size()) {

    // filling the ring buffer
    while (audioDataPos < theAudioData.samples.size() && samplesLeft > 0) {
      buffer[ringPos] = theAudioData.samples[audioDataPos];
      audioDataPos += channels;
      samplesLeft--;
      ringPos = (ringPos + 1) % windowSize;
    }

    if (samplesLeft == 0) {
      whistle.detectionState = Whistle::DetectionState::notDetected;
      samplesLeft = windowSize / 2;
      float ampSum = compute_amplitudes();
      checkMics(ampSum);
      compute_fft(); 
      overtone_detection(); 
      
      whistleNet->infer(input.data(), output.data());
      nnConfidence = output[0];

      //Merge NN, PM and limit information
      float confidence = ((nnWeight * nnConfidence + pmWeight * pmConfidence) / (nnWeight + pmWeight)) * (1 - (limitWeight * relLimitCount));
      confidenceBuffer.push_front(confidence);
      whistle.lastConfidence = confidenceBuffer.back();
      if (useAdaptiveThreshold)
        thresholdBuffer.push_front(threshold + ((1 - threshold) * (limitWeight * relLimitCount)));
      else
        thresholdBuffer.push_front(threshold);

      confidence = confidenceBuffer.back();

      // whistle detected this frame, min of #attack detections needed
      if (confidenceBuffer.back() > thresholdBuffer.average()) {
        detectedWhistleFrequency = peakPos * sampleRate / windowSize;

        //add attack to prevent short noises to be detected as whistles
        if (theFrameInfo.getTimeSince(lastAttackTime) < attackTimeout)
          attackCount++;
        else
          attackCount = 1;

        if (attackCount >= static_cast<unsigned int>(attack))
        {
          OUTPUT_TEXT("Whistle detected with confidence " << confidence);
          ANNOTATION("Whistle", "Whistle was detected");
          whistle.detectionState = Whistle::DetectionState::isDetected;
          whistle.lastTimeWhistleDetected = theFrameInfo.time;
        }

        if (freqCalibration && detectedWhistleFrequency > minFreq && detectedWhistleFrequency < maxFreq)
        {
          if (confidenceBuffer.back() > thresholdBuffer.average() * 1.5f)
          {
            ANNOTATION("Whistle", "Update whistle frequency.");
            whistleFreqBuffer.push_front(detectedWhistleFrequency);
            int minDiff = std::abs(whistleFreqBuffer.average() - currentMinFreq);
            int maxDiff = std::abs(whistleFreqBuffer.average() - currentMaxFreq);

            currentMinFreq = std::min(whistleFreqBuffer.average() - minDiff / 2, whistleFreqBuffer.average() - 250);
            currentMaxFreq = std::max(whistleFreqBuffer.average() + maxDiff / 2, whistleFreqBuffer.average() + 250);
          }
        }
        whistle.minFrequency = currentMinFreq;
        whistle.maxFrequency = currentMaxFreq;
        whistle.detectedWhistleFrequency = detectedWhistleFrequency;
        lastAttackTime = theFrameInfo.time;
      }
    }
  }
}

float WhistleDetector::compute_amplitudes() {
  float prevAmp = 0;
  float ampSum = 0;
  float limitCount = 0;
  for (int i = 0; i < ampSize; i++) {
    float amp = std::sqrt((out[i].r * out[i].r) + (out[i].i * out[i].i));
    input[i] = 20 * std::log10(amp);
    amplitudes[i] = amp;
    gradients[i] = amp - prevAmp;
    prevAmp = amp;

    if (amp > currentMaxAmp)
      currentMaxAmp = amp;

    if (amp < currentMinAmp)
      currentMinAmp = amp;

    if (amp > limit)
      limitCount++;

    ampSum = ampSum + amp;
  }
  relLimitCount = limitCount / ampSize;
  maxAmpHist.push_front(currentMaxAmp);
  currentMeanAmp = ampSum / ampSize;
  return ampSum;
}

void WhistleDetector::overtone_detection() {
  //do WHistleDetection PM
  int minPos = currentMinFreq * windowSize / sampleRate;
  int maxPos = currentMaxFreq * windowSize / sampleRate;

  //find whistle peak between min. freq. position and  max. freq. position
  peakPos = minPos;
  for (int i = minPos; i <= maxPos; i++) {
    if (amplitudes[i] > amplitudes[peakPos])
      peakPos = i;
  }

  float ampWeight = 1.f;
  if (useWeightedPMConfidence)
    ampWeight = (amplitudes[peakPos] / currentMaxAmp);

  //get min/max gradients around peakPos
  float maxGrad = gradients[peakPos];
  float minGrad = gradients[std::min(peakPos + 1, ampSize - 1)];
  maxGrad = std::abs(maxGrad / currentMaxAmp);
  minGrad = std::abs(minGrad / currentMaxAmp);

  if (amplitudes[peakPos] >= maxAmpHist.average()) {
    minPos = static_cast<int>(minPos * 2);
    maxPos = static_cast<int>(maxPos * 2);

    //find first overtone peak between peak freq. position * overtoneMultiplierMin and peak freq. position * overtoneMultiplierMax
    overtonePeakPos = minPos;
    for (int i = minPos; i <= maxPos; i++){
      if (amplitudes[i] > amplitudes[overtonePeakPos])
        overtonePeakPos = i;
    }

    //detect whistle
    if (amplitudes[overtonePeakPos] >= currentMeanAmp) 
      pmConfidence = std::max(minGrad, maxGrad) * ampWeight;
    else
      pmConfidence = std::min(minGrad, maxGrad) * ampWeight;
  }
  else {
    pmConfidence = std::min(minGrad, maxGrad) * ampWeight;
  }
}

void WhistleDetector::checkMics(float ampSum) {
  if (ampSum < 0.2f && currentMic < static_cast<unsigned int>(channels - 1)) {
    micBrokenCount++;
    if (micBrokenCount >= micBrokenThreshold)
    {
      currentMic++;
      micBrokenCount = 0;
    }
  }
  else if (ampSum < 0.2f && currentMic == static_cast<unsigned int>(channels - 1) && !alert){
    #ifdef TARGET_ROBOT
      SystemCall::say("All microphones are probably broken.");
      alert = true;
    #endif
  }
  else {
    micBrokenCount = 0;
  }
}

void WhistleDetector::compute_fft() {
  for (unsigned int i = 0; i < static_cast<unsigned int>(windowSize); i++){
    in[i].r = buffer[(i + ringPos) % windowSize];
    in[i].i = 0.f;

    //apply Hann window
    if (useHannWindowing)
      in[i].r *= std::pow(std::sin(pi * i / windowSize), 2.f);
    else if (useNuttallWindowing)
      in[i].r *= 0.355768f - 0.487396f * std::sin(1 * pi * i / windowSize) + 0.144232f * std::sin(2 * pi * i / windowSize) - 0.012604f * std::sin(3 * pi * i / windowSize);
    else { //apply Hamming window -> default
      in[i].r *= (0.54f - 0.46f * std::cos(2.f * pi * i / windowSize));
    } 
  }

  //do FFT analysis
  kiss_fft_cfg cfg;

  if ((cfg = kiss_fft_alloc(windowSize, 0 /*is_inverse_fft*/, NULL, NULL)) != NULL)
  {
    kiss_fft(cfg, in.data(), out.data()); // in and out must have the same size as given by param size
    free(cfg);
  }
  else
  {
    OUTPUT_WARNING("Not enough memory for KissFFT?");
  }
}


void WhistleDetector::setup() {
  std::string filename = std::string(File::getBHDir()) + whistleNetPath;
  oldWhistleNetPath = whistleNetPath;

  whistleNet = new OnnxHelper<float, float>(filename);

  // Setup buffers for pre- and post-processing
  int input_size = 513;
  windowSize = (input_size * 2) - 2;
  ampSize = input_size;
  samplesLeft = windowSize / 2;
  buffer = std::vector<float>(windowSize);

  in = std::vector<kiss_fft_cpx>(windowSize);
  out = std::vector<kiss_fft_cpx>(windowSize);
  amplitudes = std::vector<float>(ampSize);
  gradients = std::vector<float>(ampSize);
}

MAKE_MODULE(WhistleDetector, modeling);