#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Infrastructure/RefereeEstimator.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/ImageProcessing/Image.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <onnxruntime_cxx_api.h>

MODULE(RefereeEstimatorProvider,
{,
  REQUIRES(GameInfo),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  USES(CameraImage),  // TODO FLAVIO: prima era REQUIRES, ma in simulata dava errore
  PROVIDES(RefereeEstimator),
});

class RefereeEstimatorProvider : public RefereeEstimatorProviderBase{
  Ort::Env env;

  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
  std::unique_ptr<Ort::Session> movenetSession;
  const static int input_size = 192;
  std::array<int, 3*input_size*input_size> movenetInput{};
  std::array<float, 51> movenetOutput{};
  std::array<int64_t, 4> movenetInputShape{1,input_size,input_size,3};
  std::array<int64_t, 4> movenetOutputShape{1,1,17,3};
  const char* movenetInputNames[1] = {"input"};
  size_t movenetNumInputNodes;
  const char* movenetOutputNames[1] = {"output_0"};
  size_t movenetNumOutputNodes;

  std::unique_ptr<Ort::Session> classifierSession;
  std::array<float, 4> classifierInput{};
  std::array<float, 1> classifierOutput{};
  std::array<int64_t, 2> classifierInputShape{1, 4};
  std::array<int64_t, 2> classifierOutputShape{1, 1};
  const char* classifierInputNames[1] = {"input"};
  size_t classifierNumInputNodes;
  const char* classifierOutputNames[1] = {"output"};
  size_t classifierNumOutputNodes;

  public:
    RefereeEstimatorProvider();
    void update(GameInfo& gameInfo);
    void update(RefereeEstimator& estimator);

  private:
    const float confidence_threshold = 0.2;
    std::vector<std::vector<int>> triples = {{12, 6, 8}, {11, 5, 7}, {6, 8, 10}, {5, 7, 9}};
    int computeAngles();
    const float classifier_threshold = 0.4; // 0.4 equals to 60%

    int lefthandy = 9*3;
    int leftshouldery = 5*3;
    int righthandy = 10*3;
    int rightshouldery = 6*3;

    std::vector<std::vector<int>> debug_lines = {{15, 13}, {16, 14}, {11, 13}, {12, 14}, {11, 12}, {5, 11}, {6, 12}, {11, 12}, {5, 7}, {6, 8}, {7, 9}, {8, 10}};
    void drawKeypoints(cv::Rect ROI);
};