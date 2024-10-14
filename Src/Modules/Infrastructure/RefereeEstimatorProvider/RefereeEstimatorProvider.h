/**
 * @file RefereeEstimatorProvider.cpp
 *
 * @author Filippo Ansalone
 */

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
#include "Tools/OnnxHelper/OnnxHelper.h"

MODULE(RefereeEstimatorProvider,
{,
  REQUIRES(GameInfo),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  USES(CameraImage),
  PROVIDES(RefereeEstimator),
});

class RefereeEstimatorProvider : public RefereeEstimatorProviderBase{
  const static int input_size = 192;
  OnnxHelper<int, float> *movenet;

  OnnxHelper<float, float> *classifier;

  public:
    RefereeEstimatorProvider();
    void update(GameInfo& gameInfo);
    void update(RefereeEstimator& estimator);

  private:
    const float confidence_threshold = 0.2;
    std::vector<std::vector<int>> triples = {{12, 6, 8}, {11, 5, 7}, {6, 8, 10}, {5, 7, 9}};
    int computeAngles(std::array<float, 51> movenetOutput, std::array<float, 4>& classifierInput);
    const float classifier_threshold = 0.4;

    int lefthandy = 9*3;
    int leftshouldery = 5*3;
    int righthandy = 10*3;
    int rightshouldery = 6*3;

    std::vector<std::vector<int>> debug_lines = {{15, 13}, {16, 14}, {11, 13}, {12, 14}, {11, 12}, {5, 11}, {6, 12}, {11, 12}, {5, 7}, {6, 8}, {7, 9}, {8, 10}};
    void drawKeypoints(cv::Rect ROI, std::array<float, 51> movenetOutput);
};