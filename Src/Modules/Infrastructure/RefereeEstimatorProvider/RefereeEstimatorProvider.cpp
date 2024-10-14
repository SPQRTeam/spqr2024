/**
 * @file RefereeEstimatorProvider.cpp
 *
 * @author Filippo Ansalone, Daniele Affinita
 */

#include "RefereeEstimatorProvider.h"

#include "Platform/File.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/RefereeEstimator.h"

#include "Eigen/Dense"

#include <math.h>
#include <iomanip>
#include "Tools/OnnxHelper/OnnxHelper.h"

RefereeEstimatorProvider::RefereeEstimatorProvider()
{
  std::string movenetFilename = std::string(File::getBHDir()) + "/Config/NeuralNets/RefereeEstimator/movenet_singlepose_lightning_4.onnx";
  std::string classifierFilename = std::string(File::getBHDir()) + "/Config/NeuralNets/RefereeEstimator/classifier.onnx";

  movenet = new OnnxHelper<int, float>(movenetFilename);
  classifier = new OnnxHelper<float, float>(classifierFilename);
}

void RefereeEstimatorProvider::update(RefereeEstimator& estimator)
{
  DECLARE_DEBUG_DRAWING("representation:Referee:image", "drawingOnImage");
  if (theGameInfo.state != STATE_STANDBY){
    estimator.isDetected = false;
    return;
  }

  cv::Mat outputImage(theCameraImage.height, theCameraImage.width, CV_8UC3);
  std::string text = "";
  for (int x = 0; x < theCameraImage.width; ++x){
    for (int y = 0; y < theCameraImage.height; ++y){
      auto& px = theCameraImage[Vector2i(x,y)];
      std::vector<uchar> rgb_px = px.rgb();
      outputImage.at<cv::Vec3b>(y, x) = cv::Vec3b(rgb_px[2], rgb_px[1], rgb_px[0]);
    }
  }

  int crop_width = theCameraImage.width/4;
  int crop_height = theCameraImage.height/2;
  cv::Rect ROI((theCameraImage.width - crop_width)/2, theCameraImage.height/2 - 30, crop_width, crop_height); // x, y, width, height
  cv::Mat croppedImage = outputImage(ROI);

  RECTANGLE("representation:Referee:image", ROI.x*2, ROI.y, ROI.x*2+ROI.width*2, ROI.y+ROI.height, 3, Drawings::solidPen, ColorRGBA::black);

  cv::Mat resizedImage;
  int resized_width;
  int resized_height;
  if (crop_width*2 > crop_height)
  {
    resized_width = input_size;
    resized_height = input_size*ROI.height/(ROI.width*2);
  }
  else
  {
    resized_height = input_size;
    resized_width = input_size*(ROI.width*2)/ROI.height;
  }

  cv::resize(croppedImage, resizedImage, cv::Size(resized_width, resized_height));

  cv::Mat paddedImage;
  int top = crop_width*2 > crop_height ? (resized_width - resized_height) / 2 : 0; // (192 - 144) / 2
  int bottom = top;
  int left = crop_width*2 <= crop_height ? (resized_height - resized_width) / 2 : 0;
  int right = left;

  if (resized_width + 2*left == input_size-1)
    left++;
  if (resized_height + 2*bottom == input_size-1)
    bottom++;

  cv::copyMakeBorder(resizedImage, paddedImage, top, bottom, left, right, cv::BorderTypes::BORDER_CONSTANT, 0);
  paddedImage.convertTo(paddedImage, CV_32SC3);

  std::array<float, 51> movenetOutput;
  movenet->infer(paddedImage.data, movenetOutput.data());

  drawKeypoints(ROI, movenetOutput);

  std::array<float, 4> classifierInput;
  int num_angles = computeAngles(movenetOutput, classifierInput);

  if (num_angles < 3){
    estimator.measures = 0;
    DRAW_TEXT("representation:Referee:image", 0, 50, 50, ColorRGBA::yellow, "Don't know");
    return;
  }

  std::array<float, 1> classifierOutput;
  classifier->infer(classifierInput.data(), classifierOutput.data());

  if (movenetOutput[lefthandy] < movenetOutput[leftshouldery] && movenetOutput[righthandy] < movenetOutput[rightshouldery] && classifierOutput[0] > classifier_threshold){
    estimator.measures <<= 1;
    estimator.measures |= 0x1;
    DRAW_TEXT("representation:Referee:image", 0, 50, 50, ColorRGBA::green, "Ready " + std::to_string(estimator.measures));
  }else{
    estimator.measures = 0;
    DRAW_TEXT("representation:Referee:image", 0, 50, 50, ColorRGBA::red, "Not ready" + std::to_string(estimator.measures));
  }

  if (estimator.measures >= 0xF)
    estimator.timeOfLastDetection = theFrameInfo.time;

  estimator.isDetected = theFrameInfo.getTimeSince(estimator.timeOfLastDetection) < 40000;
}

int RefereeEstimatorProvider::computeAngles(std::array<float, 51> movenetOutput, std::array<float, 4>& classifierInput){
  int angles_count = 0;
  float angle_rad;
  std::stringstream stream;
  for(int i = 0; i < 4; i++){
    if(movenetOutput[triples[i][0]*3+2] < confidence_threshold || movenetOutput[triples[i][1]*3+2] < confidence_threshold || movenetOutput[triples[i][2]*3+2] < confidence_threshold){
      classifierInput[i] = -1.0;
      continue;
    }
    Eigen::Vector2f kp1(movenetOutput[triples[i][0]*3], movenetOutput[triples[i][0]*3+1]);
    Eigen::Vector2f kp2(movenetOutput[triples[i][1]*3], movenetOutput[triples[i][1]*3+1]);
    Eigen::Vector2f kp3(movenetOutput[triples[i][2]*3], movenetOutput[triples[i][2]*3+1]);

    Eigen::Vector2f BA = kp1 - kp2;
    Eigen::Vector2f BC = kp3 - kp2;

    angle_rad = std::atan2(BC[1], BC[0]) - std::atan2(BA[1], BA[0]);
    angle_rad = fmodf(angle_rad, (2 * M_PI));
    if (angle_rad < 0)
      angle_rad += 2 * M_PI;

    classifierInput.data()[i] = angle_rad;

    angles_count++;

    // debug angles
    int x = static_cast<int>(((kp2[1]-0.125)/0.75) * theCameraImage.height);
    int y = static_cast<int>(kp2[0] * theCameraImage.width * 2) - 40;
    stream << std::fixed << std::setprecision(3) << angle_rad;
    DRAW_TEXT("representation:Referee:image", x, y, 15, ColorRGBA::blue, stream.str());
  }
  return angles_count; 
}

void RefereeEstimatorProvider::drawKeypoints(cv::Rect ROI, std::array<float, 51> movenetOutput){
  int base_x = ROI.x * 2;
  int base_y = ROI.y;
  float shift_x = ((input_size - ROI.width)/2)/input_size;
  float shift_y = ((input_size - ROI.height)/2)/input_size;
  for(int i = 0; i < movenetOutput.size(); i += 3){
    if (movenetOutput[i+2] >= confidence_threshold){
      MID_DOT("representation:Referee:image", base_x + static_cast<int>(((movenetOutput[i+1]-shift_x)/((input_size-2*shift_x)/input_size)) * ROI.width * 2), base_y + static_cast<int>(((movenetOutput[i]-shift_y)/((input_size-2*shift_y)/input_size)) * ROI.height), ColorRGBA::red, ColorRGBA::red);
    }
  }
  for(int i = 0; i < debug_lines.size(); i++){
    int x1 = base_x + static_cast<int>(((movenetOutput[debug_lines[i][0]*3+1]-shift_x)/((input_size-2*shift_x)/input_size)) * ROI.width * 2);
    int y1 = base_y + static_cast<int>(((movenetOutput[debug_lines[i][0]*3]-shift_y)/((input_size-2*shift_y)/input_size)) * ROI.height);
    int x2 = base_x + static_cast<int>(((movenetOutput[debug_lines[i][1]*3+1]-shift_x)/((input_size-2*shift_x)/input_size)) * ROI.width * 2);
    int y2 = base_y + static_cast<int>(((movenetOutput[debug_lines[i][1]*3]-shift_y)/((input_size-2*shift_y)/input_size)) * ROI.height);
    LINE("representation:Referee:image", x1, y1, x2, y2, 3, Drawings::solidPen, ColorRGBA::black);
  }
}

MAKE_MODULE(RefereeEstimatorProvider, modeling);